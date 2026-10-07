import glob
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import tyro

from xcore_gello_software.env import RobotEnv
from xcore_gello_software.robots.robot import PrintRobot
from xcore_gello_software.utils.launch_utils import instantiate_from_dict
from xcore_gello_software.zmq_core.robot_node import ZMQClientRobot


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "none"
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"
    robot_type: Optional[str] = None  # only needed for quest agent or spacemouse agent
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False
    read_gripper: bool = True
    gripper_only: bool = False
    """Hold the current arm pose and follow only the GELLO gripper."""
    gripper_open_deg: Optional[float] = None
    gripper_close_deg: Optional[float] = None

    def __post_init__(self):
        if self.start_joints is not None:
            self.start_joints = np.array(self.start_joints)


class GripperOnlyAgent:
    has_gripper = True

    def __init__(self, leader, arm_joints):
        self.leader = leader
        self.arm_joints = arm_joints.copy()
        self._last_report = 0.0

    def act(self, obs):
        closure = self.leader.act(obs)[-1]
        if time.monotonic() - self._last_report >= 0.5:
            state = self.leader.get_gripper_state()
            actual = float(obs.get("gripper_position", 0))
            print(
                f"[gripper] trigger={state['raw_degrees']:.1f} deg "
                f"leader_closure={closure:.3f} sim_closure={actual:.3f}"
            )
            self._last_report = time.monotonic()
        return np.concatenate([self.arm_joints, [closure]])

    def close(self):
        self.leader.close()


def main(args):
    if args.gripper_only and (
        args.agent != "gello" or args.bimanual or not args.read_gripper
    ):
        raise ValueError(
            "--gripper-only requires a single GELLO agent with gripper reading enabled"
        )
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    agent_cfg = {}
    if args.bimanual:
        if args.agent == "gello":
            # dynamixel control box port map (to distinguish left and right gello)
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0"
            agent_cfg = {
                "_target_": "xcore_gello_software.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "xcore_gello_software.agents.gello_agent.GelloAgent",
                    "port": left,
                },
                "agent_right": {
                    "_target_": "xcore_gello_software.agents.gello_agent.GelloAgent",
                    "port": right,
                },
            }
        elif args.agent == "quest":
            agent_cfg = {
                "_target_": "xcore_gello_software.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "xcore_gello_software.agents.quest_agent.SingleArmQuestAgent",
                    "robot_type": args.robot_type,
                    "which_hand": "l",
                },
                "agent_right": {
                    "_target_": "xcore_gello_software.agents.quest_agent.SingleArmQuestAgent",
                    "robot_type": args.robot_type,
                    "which_hand": "r",
                },
            }
        elif args.agent == "spacemouse":
            left_path = "/dev/hidraw0"
            right_path = "/dev/hidraw1"
            agent_cfg = {
                "_target_": "xcore_gello_software.agents.agent.BimanualAgent",
                "agent_left": {
                    "_target_": "xcore_gello_software.agents.spacemouse_agent.SpacemouseAgent",
                    "robot_type": args.robot_type,
                    "device_path": left_path,
                    "verbose": args.verbose,
                },
                "agent_right": {
                    "_target_": "xcore_gello_software.agents.spacemouse_agent.SpacemouseAgent",
                    "robot_type": args.robot_type,
                    "device_path": right_path,
                    "verbose": args.verbose,
                    "invert_button": True,
                },
            }
        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([0, -90, -90, -90, 90, 0, 0])
        reset_joints_right = np.deg2rad([0, -90, 90, -90, -90, 0, 0])
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
        curr_joints = env.get_obs()["joint_positions"]
        max_delta = (np.abs(curr_joints - reset_joints)).max()
        steps = min(int(max_delta / 0.01), 100)

        for jnt in np.linspace(curr_joints, reset_joints, steps):
            env.step(jnt)
    else:
        if args.agent == "gello":
            gello_port = args.gello_port
            if gello_port is None:
                usb_ports = glob.glob("/dev/serial/by-id/*")
                print(f"Found {len(usb_ports)} ports")
                if len(usb_ports) > 0:
                    gello_port = usb_ports[0]
                    print(f"using port {gello_port}")
                else:
                    raise ValueError(
                        "No gello port found, please specify one or plug in gello"
                    )
            agent_cfg = {
                "_target_": "xcore_gello_software.agents.gello_agent.GelloAgent",
                "port": gello_port,
                "start_joints": args.start_joints,
                "read_gripper": args.read_gripper,
                "gripper_open_deg": args.gripper_open_deg,
                "gripper_close_deg": args.gripper_close_deg,
            }
            if args.start_joints is None:
                reset_joints = np.deg2rad(
                    # [0, -90, 90, -90, -90, 0, 0]
                    [0, 0, 0, 0, 0, 0, 0]
                )  # Change this to your own reset joints
            else:
                reset_joints = np.array(args.start_joints)

            curr_joints = env.get_obs()["joint_positions"]
            if not args.gripper_only and reset_joints.shape == curr_joints.shape:
                max_delta = (np.abs(curr_joints - reset_joints)).max()
                steps = min(int(max_delta / 0.01), 100)

                for jnt in np.linspace(curr_joints, reset_joints, steps):
                    env.step(jnt)
                    time.sleep(0.001)
            if args.start_joints is None:
                # Resolve the leader's accumulated turns against the follower
                # after any reset, while keeping its calibrated zero offsets.
                agent_cfg["start_joints"] = env.get_obs()["joint_positions"].copy()
        elif args.agent == "quest":
            agent_cfg = {
                "_target_": "xcore_gello_software.agents.quest_agent.SingleArmQuestAgent",
                "robot_type": args.robot_type,
                "which_hand": "l",
            }
        elif args.agent == "spacemouse":
            agent_cfg = {
                "_target_": "xcore_gello_software.agents.spacemouse_agent.SpacemouseAgent",
                "robot_type": args.robot_type,
                "verbose": args.verbose,
            }
        elif args.agent == "dummy" or args.agent == "none":
            agent_cfg = {
                "_target_": "xcore_gello_software.agents.agent.DummyAgent",
                "num_dofs": robot_client.num_dofs(),
            }
        elif args.agent == "policy":
            raise NotImplementedError("add your imitation policy here if there is one")
        else:
            raise ValueError("Invalid agent name")

    agent = instantiate_from_dict(agent_cfg)
    try:
        has_gripper = getattr(agent, "has_gripper", False)
        if args.gripper_only:
            if not has_gripper:
                raise ValueError("The selected GELLO configuration has no gripper")
            if agent.num_dofs() != len(env.get_obs()["joint_positions"]):
                raise ValueError(
                    "--gripper-only requires a follower with six arm joints and one gripper"
                )
            agent = GripperOnlyAgent(agent, env.get_obs()["joint_positions"][:-1])
        # going to start position
        print("Going to start position")
        start_pos = agent.act(env.get_obs())
        obs = env.get_obs()
        joints = obs["joint_positions"]

        if len(start_pos) != len(joints):
            raise ValueError(
                f"GELLO output has {len(start_pos)} channels; server has {len(joints)}. "
                "Launch sim_cr7 with its gripper, or disable both sides with "
                "--no-with-gripper / --no-read-gripper."
            )
        arm_dofs = len(joints) - int(has_gripper)
        abs_deltas = np.abs(start_pos[:arm_dofs] - joints[:arm_dofs])
        id_max_joint_delta = np.argmax(abs_deltas)

        max_joint_delta = 0.8
        if abs_deltas[id_max_joint_delta] > max_joint_delta:
            id_mask = abs_deltas > max_joint_delta
            print()
            ids = np.arange(len(id_mask))[id_mask]
            for i, delta, joint, current_j in zip(
                ids,
                abs_deltas[id_mask],
                start_pos[:arm_dofs][id_mask],
                joints[:arm_dofs][id_mask],
            ):
                print(
                    f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
                )
            return

        print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
        assert len(start_pos) == len(joints), (
            f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"
        )

        max_delta = 0.05
        for _ in range(25):
            obs = env.get_obs()
            command_joints = agent.act(obs)
            current_joints = obs["joint_positions"]
            delta = command_joints - current_joints
            max_joint_delta = np.abs(delta).max()
            if max_joint_delta > max_delta:
                delta = delta / max_joint_delta * max_delta
            env.step(current_joints + delta)

        obs = env.get_obs()
        joints = obs["joint_positions"]
        action = agent.act(obs)
        if (action[:arm_dofs] - joints[:arm_dofs] > 0.5).any():
            print("Action is too big")

            # print which joints are too big
            joint_index = np.where(action[:arm_dofs] - joints[:arm_dofs] > 0.8)
            for j in joint_index:
                print(
                    f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
                )
            exit()

        from xcore_gello_software.utils.control_utils import (
            SaveInterface,
            run_control_loop,
        )

        save_interface = None
        if args.use_save_interface:
            save_interface = SaveInterface(
                data_dir=args.data_dir, agent_name=args.agent, expand_user=True
            )

        run_control_loop(env, agent, save_interface, use_colors=True)
    finally:
        close = getattr(agent, "close", None)
        if close is not None:
            close()


if __name__ == "__main__":
    main(tyro.cli(Args))
