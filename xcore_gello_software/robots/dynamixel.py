from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from xcore_gello_software.robots.robot import Robot


class DynamixelRobot(Robot):
    """A class representing a UR robot."""

    def __init__(
        self,
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        real: bool = False,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        from xcore_gello_software.dynamixel.driver import (
            DynamixelDriver,
            DynamixelDriverProtocol,
            FakeDynamixelDriver,
        )

        print(f"attempting to connect to port: {port}")
        self._raw_gripper_degrees = None
        self.gripper_open_close: Optional[Tuple[float, float]]
        if gripper_config is not None:
            if (
                not np.isfinite(gripper_config[1:]).all()
                or gripper_config[1] == gripper_config[2]
            ):
                raise ValueError(
                    "Gripper open/closed angles must be finite and different"
                )
            if gripper_config[0] in joint_ids:
                raise ValueError("Gripper ID must differ from the arm joint IDs")
            assert joint_offsets is not None
            assert joint_signs is not None

            # joint_ids.append(gripper_config[0])
            # joint_offsets.append(0.0)
            # joint_signs.append(1)
            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
        else:
            self.gripper_open_close = None

        self._joint_ids = joint_ids
        self._driver: DynamixelDriverProtocol

        if joint_offsets is None:
            self._joint_offsets = np.zeros(len(joint_ids))
        else:
            self._joint_offsets = np.array(joint_offsets)

        if joint_signs is None:
            self._joint_signs = np.ones(len(joint_ids))
        else:
            self._joint_signs = np.array(joint_signs)

        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(np.abs(self._joint_signs) == 1), (
            f"joint_signs: {self._joint_signs}"
        )

        if real:
            self._driver = DynamixelDriver(
                joint_ids, port=port, baudrate=baudrate, use_fake_fallback=False
            )
            self._driver.set_torque_mode(False)
        else:
            self._driver = FakeDynamixelDriver(joint_ids)
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99

        if start_joints is not None:
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            new_joint_offsets = []
            try:
                current_joints = self.get_joint_state()
            except BaseException:
                self.close()
                raise
            assert current_joints.shape == start_joints.shape
            if gripper_config is not None:
                current_joints = current_joints[:-1]
                start_joints = start_joints[:-1]
            for idx, (c_joint, s_joint, joint_offset) in enumerate(
                zip(current_joints, start_joints, self._joint_offsets)
            ):
                new_joint_offsets.append(
                    np.pi
                    * 2
                    * np.round((-s_joint + c_joint) / (2 * np.pi))
                    * self._joint_signs[idx]
                    + joint_offset
                )
            if gripper_config is not None:
                new_joint_offsets.append(self._joint_offsets[-1])
            self._joint_offsets = np.array(new_joint_offsets)
            # The cached position was measured before the turn correction.
            # Seed smoothing again in the corrected coordinate frame.
            self._last_pos = None

    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()

        if self.gripper_open_close is not None:
            self._raw_gripper_degrees = float(np.rad2deg(pos[-1]))
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )
            g_pos = min(max(0, g_pos), 1)
            pos[-1] = g_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        # debug: log gripper raw angle and normalized value
        if self.gripper_open_close is not None:
            if not hasattr(self, "_gripper_debug_counter"):
                self._gripper_debug_counter = 0
            self._gripper_debug_counter += 1
            if self._gripper_debug_counter % 50 == 0:
                raw_deg = np.rad2deg(
                    (self._driver.get_joints()[-1] - self._joint_offsets[-1])
                    * self._joint_signs[-1]
                )
                print(
                    f"[gripper] raw: {raw_deg:6.1f}° | "
                    f"normalized: {pos[-1]:.3f} | "
                    f"open={np.rad2deg(self.gripper_open_close[0]):.0f}° "
                    f"close={np.rad2deg(self.gripper_open_close[1]):.0f}°"
                )

        return pos

    def get_gripper_state(self):
        """Return the latest trigger reading, without another serial transaction."""
        if self.gripper_open_close is None:
            return None
        if self._last_pos is None:
            self.get_joint_state()
        closure = float(self._last_pos[-1])
        return {
            "raw_degrees": self._raw_gripper_degrees,
            "closure": closure,
            "openness": 1.0 - closure,
        }

    def close(self):
        close = getattr(self._driver, "close", None)
        if close is not None:
            close()

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._driver.set_joints((joint_state + self._joint_offsets).tolist())

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}
