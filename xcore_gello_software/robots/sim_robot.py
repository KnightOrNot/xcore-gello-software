import pickle
import threading
import time
from typing import Any, Dict, Optional

import mujoco
import mujoco.viewer
import numpy as np
import zmq
from dm_control import mjcf

from xcore_gello_software.robots.robot import Robot

assert mujoco.viewer is mujoco.viewer


def attach_hand_to_arm(
    arm_mjcf: mjcf.RootElement,
    hand_mjcf: mjcf.RootElement,
) -> None:
    """Attaches a hand to an arm.

    The arm must have a site named "attachment_site".

    Taken from https://github.com/deepmind/mujoco_menagerie/blob/main/FAQ.md#how-do-i-attach-a-hand-to-an-arm

    Args:
      arm_mjcf: The mjcf.RootElement of the arm.
      hand_mjcf: The mjcf.RootElement of the hand.

    Raises:
      ValueError: If the arm does not have a site named "attachment_site".
    """
    physics = mjcf.Physics.from_mjcf_model(hand_mjcf)

    attachment_site = arm_mjcf.find("site", "attachment_site")
    if attachment_site is None:
        raise ValueError("No attachment site found in the arm model.")

    # Expand the ctrl and qpos keyframes to account for the new hand DoFs.
    arm_key = arm_mjcf.find("key", "home")
    if arm_key is not None:
        hand_key = hand_mjcf.find("key", "home")
        if hand_key is None:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, np.zeros(physics.model.nu)])
            arm_key.qpos = np.concatenate([arm_key.qpos, np.zeros(physics.model.nq)])
        else:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, hand_key.ctrl])
            arm_key.qpos = np.concatenate([arm_key.qpos, hand_key.qpos])

    attachment_site.attach(hand_mjcf)


def build_scene(robot_xml_path: str, gripper_xml_path: Optional[str] = None):
    # assert robot_xml_path.endswith(".xml")

    arena = mjcf.RootElement()
    arm_simulate = mjcf.from_path(robot_xml_path)
    # arm_copy = mjcf.from_path(xml_path)

    if gripper_xml_path is not None:
        # attach gripper to the robot at "attachment_site"
        gripper_simulate = mjcf.from_path(gripper_xml_path)
        attach_hand_to_arm(arm_simulate, gripper_simulate)

    arena.worldbody.attach(arm_simulate)
    # arena.worldbody.attach(arm_copy)

    return arena


class ZMQServerThread(threading.Thread):
    def __init__(self, server):
        super().__init__()
        self._server = server

    def run(self):
        self._server.serve()

    def terminate(self):
        self._server.stop()


class ZMQRobotServer:
    """A class representing a ZMQ server for a robot."""

    def __init__(self, robot: Robot, host: str = "127.0.0.1", port: int = 5556):
        self._robot = robot
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        addr = f"tcp://{host}:{port}"
        self._socket.bind(addr)
        self._stop_event = threading.Event()

    def serve(self) -> None:
        """Serve the robot state and commands over ZMQ."""
        self._socket.setsockopt(zmq.RCVTIMEO, 1000)  # Set timeout to 1000 ms
        while not self._stop_event.is_set():
            try:
                message = self._socket.recv()
                request = pickle.loads(message)

                # Call the appropriate method based on the request
                method = request.get("method")
                args = request.get("args", {})
                result: Any
                if method == "num_dofs":
                    result = self._robot.num_dofs()
                elif method == "get_joint_state":
                    result = self._robot.get_joint_state()
                elif method == "get_ctrl":
                    result = self._robot.get_ctrl()
                elif method == "command_joint_state":
                    result = self._robot.command_joint_state(**args)
                elif method == "get_observations":
                    result = self._robot.get_observations()
                else:
                    result = {"error": "Invalid method"}
                    print(result)
                    raise NotImplementedError(
                        f"Invalid method: {method}, {args, result}"
                    )

                self._socket.send(pickle.dumps(result))
            except zmq.error.Again:
                print("Timeout in ZMQLeaderServer serve")
                # Timeout occurred, check if the stop event is set

    def stop(self) -> None:
        self._stop_event.set()
        self._socket.close()
        self._context.term()


class MujocoRobotServer:
    def __init__(
        self,
        xml_path: str,
        gripper_xml_path: Optional[str] = None,
        host: str = "127.0.0.1",
        port: int = 5556,
        print_joints: bool = False,
        arm_dofs: Optional[int] = None,
        gripper_reverse: bool = False,
    ):
        """MuJoCo robot server.

        Args:
            xml_path: Path to the robot MJCF XML file.
            gripper_xml_path: Optional path to a separate gripper XML file.
            host: ZMQ server host.
            port: ZMQ server port.
            print_joints: Whether to print joint states to console.
            arm_dofs: Number of arm joints (excluding gripper). If None,
                      defaults to num_joints (no gripper). If set to a value
                      less than total actuators, the remaining actuators are
                      treated as grippers with [0,1] normalization.
            gripper_reverse: Invert the gripper [0,1] value before scaling.
        """
        self._has_gripper_xml = gripper_xml_path is not None
        arena = build_scene(xml_path, gripper_xml_path)

        assets: Dict[str, str] = {}
        for asset in arena.asset.all_children():
            if asset.tag == "mesh":
                f = asset.file
                assets[f.get_vfs_filename()] = asset.file.contents

        xml_string = arena.to_xml_string()
        # save xml_string to file
        with open("arena.xml", "w") as f:
            f.write(xml_string)

        self._model = mujoco.MjModel.from_xml_string(xml_string, assets)
        self._data = mujoco.MjData(self._model)

        self._num_joints = self._model.nu

        # arm_dofs: how many of the actuators are actual arm joints
        # the rest (if any) are treated as gripper
        if arm_dofs is None:
            self._arm_dofs = self._num_joints  # no gripper
        else:
            self._arm_dofs = min(arm_dofs, self._num_joints)

        self._has_gripper_joint = self._arm_dofs < self._num_joints

        # Read gripper actuator ctrlrange for proper scaling
        if self._has_gripper_joint:
            self._gripper_ctrl_range = self._model.actuator_ctrlrange[-1].copy()

        self._joint_state = np.zeros(self._num_joints)
        self._joint_cmd = self._joint_state

        self._zmq_server = ZMQRobotServer(robot=self, host=host, port=port)
        self._zmq_server_thread = ZMQServerThread(self._zmq_server)

        self._print_joints = print_joints
        self._gripper_reverse = gripper_reverse

    def num_dofs(self) -> int:
        return self._num_joints

    def get_joint_state(self) -> np.ndarray:
        return self._joint_state

    def get_ctrl(self) -> np.ndarray:
        return self._joint_cmd

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        assert len(joint_state) == self._num_joints, (
            f"Expected joint state of length {self._num_joints}, "
            f"got {len(joint_state)}."
        )
        _joint_state = joint_state.copy()
        if self._has_gripper_xml:
            # Separate gripper XML (e.g. Robotiq 2F-85): scale [0,1] → [0,255]
            _joint_state[-1] = _joint_state[-1] * 255
        elif self._has_gripper_joint:
            # Gripper joint embedded in arm XML: scale [0,1] → ctrlrange
            g_val = _joint_state[-1]
            if self._gripper_reverse:
                g_val = 1.0 - g_val
            g_min, g_max = self._gripper_ctrl_range
            _joint_state[-1] = g_val * (g_max - g_min) + g_min
        # else: no gripper at all (pure arm), pass through as-is
        self._joint_cmd = _joint_state

    def freedrive_enabled(self) -> bool:
        return True

    def set_freedrive_mode(self, enable: bool):
        pass

    def get_observations(self) -> Dict[str, np.ndarray]:
        joint_positions = self._data.qpos.copy()[: self._num_joints]
        joint_velocities = self._data.qvel.copy()[: self._num_joints]
        ee_site = "attachment_site"
        try:
            ee_pos = self._data.site_xpos.copy()[
                mujoco.mj_name2id(self._model, 6, ee_site)
            ]
            ee_mat = self._data.site_xmat.copy()[
                mujoco.mj_name2id(self._model, 6, ee_site)
            ]
            ee_quat = np.zeros(4)
            mujoco.mju_mat2Quat(ee_quat, ee_mat)
        except Exception:
            ee_pos = np.zeros(3)
            ee_quat = np.zeros(4)
            ee_quat[0] = 1
        if self._has_gripper_joint or self._has_gripper_xml:
            gripper_pos = self._data.qpos.copy()[self._num_joints - 1]
        else:
            gripper_pos = 0.0
        return {
            "joint_positions": joint_positions,
            "joint_velocities": joint_velocities,
            "ee_pos_quat": np.concatenate([ee_pos, ee_quat]),
            "gripper_position": np.array(gripper_pos),
        }

    def serve(self) -> None:
        # start the zmq server
        self._zmq_server_thread.start()
        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            # Track wall-clock time to maintain real-time simulation speed
            # despite viewer.sync() blocking at display refresh rate (~60Hz).
            wall_start = time.time()
            sim_start = self._data.time
            frame_count = 0
            total_steps = 0

            while viewer.is_running():
                # Advance simulation to catch up with elapsed wall-clock time.
                # viewer.sync() blocks at vsync (~16ms), so we need multiple
                # physics steps per frame to maintain 1x real-time speed.
                elapsed_wall = time.time() - wall_start
                target_sim_time = sim_start + elapsed_wall

                steps_this_frame = 0
                while self._data.time < target_sim_time:
                    self._data.ctrl[:] = self._joint_cmd
                    mujoco.mj_step(self._model, self._data)
                    steps_this_frame += 1

                total_steps += steps_this_frame
                frame_count += 1

                self._joint_state = self._data.qpos.copy()[: self._num_joints]

                if self._print_joints and frame_count % 60 == 0:
                    avg_steps = total_steps / frame_count
                    sim_rate = avg_steps * self._model.opt.timestep * frame_count / max(elapsed_wall, 0.001)
                    print(f"[SIM] frame={frame_count} steps_per_frame={steps_this_frame} "
                          f"avg_steps={avg_steps:.1f} sim_rate={sim_rate:.2f}x "
                          f"wall={elapsed_wall:.1f}s sim={self._data.time:.1f}s "
                          f"DIFF_J1={self._data.ctrl[0] - self._joint_state[0]:.3f}")

                # Example modification of a viewer option: toggle contact points every two seconds.
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(
                        self._data.time % 2
                    )

                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()

    def stop(self) -> None:
        self._zmq_server_thread.join()

    def __del__(self) -> None:
        self.stop()
