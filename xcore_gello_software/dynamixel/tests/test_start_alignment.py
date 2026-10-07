import contextlib
import io
import unittest
from unittest.mock import Mock, patch

import numpy as np

from xcore_gello_software.commands import run_env
from xcore_gello_software.agents.gello_agent import PORT_CONFIG_MAP
from xcore_gello_software.robots.dynamixel import DynamixelRobot


CR7_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"


class StartAlignmentTests(unittest.TestCase):
    def make_robot(self, raw, **kwargs):
        driver = Mock()
        driver.get_joints.return_value = np.array(raw, dtype=float)
        with patch(
            "xcore_gello_software.dynamixel.driver.DynamixelDriver", return_value=driver
        ):
            robot = DynamixelRobot(real=True, **kwargs)
        return robot, driver

    def test_reported_cr7_angles_align_on_first_read(self):
        offsets = np.array(PORT_CONFIG_MAP[CR7_PORT].joint_offsets)
        reported = np.array([0, 0, 0, 0, 6.381, 12.551])
        robot, _ = self.make_robot(
            offsets + reported,
            joint_ids=(1, 2, 3, 4, 5, 6),
            joint_offsets=offsets,
            start_joints=np.zeros(6),
        )
        expected = reported - np.array([0, 0, 0, 0, 2 * np.pi, 4 * np.pi])
        np.testing.assert_allclose(robot.get_joint_state(), expected, atol=1e-12)

    def test_negative_signs_and_nonzero_reference(self):
        reference = np.array([0.4, -0.6])
        signs = np.array([-1, 1])
        offsets = np.array([np.pi, -np.pi / 2])
        reported = reference + 2 * np.pi * np.array([2, -3])
        robot, _ = self.make_robot(
            offsets + reported * signs,
            joint_ids=(1, 2),
            joint_offsets=offsets,
            joint_signs=signs,
            start_joints=reference,
        )
        np.testing.assert_allclose(robot.get_joint_state(), reference, atol=1e-12)

    def test_alignment_preserves_gripper_normalization(self):
        robot, _ = self.make_robot(
            [2 * np.pi + 0.1, -4 * np.pi - 0.2, 0.75 * np.pi],
            joint_ids=(1, 2),
            joint_offsets=(0, 0),
            joint_signs=(1, 1),
            gripper_config=(3, 0, 180),
            start_joints=np.array([0, 0, 0.75]),
        )
        np.testing.assert_allclose(
            robot.get_joint_state(), [0.1, -0.2, 0.75], atol=1e-12
        )

    def test_motion_remains_continuous_across_pi(self):
        robot, driver = self.make_robot(
            [2 * np.pi + 3.13],
            joint_ids=(1,),
            start_joints=np.array([3.13]),
        )
        robot.get_joint_state()
        driver.get_joints.return_value += 0.04
        self.assertAlmostEqual(robot.get_joint_state()[0], 3.13 + 0.99 * 0.04)

    def run_cr7_startup(
        self, reported, *, gripper_deg=194.8, gripper_only=False, read_gripper=True
    ):
        steps = []

        class TestEnv:
            def __init__(self, *args, **kwargs):
                self.joints = np.zeros(7 if read_gripper else 6)
                if gripper_only:
                    self.joints[:6] = 0.3

            def get_obs(self):
                return {
                    "joint_positions": self.joints.copy(),
                    "gripper_position": self.joints[-1] if read_gripper else 0,
                }

            def step(self, joints):
                self.joints = joints.copy()
                steps.append(joints.copy())

        driver = Mock()
        driver.get_joints.return_value = (
            np.array(PORT_CONFIG_MAP[CR7_PORT].joint_offsets) + reported
        )
        if read_gripper:
            driver.get_joints.return_value = np.append(
                driver.get_joints.return_value, np.deg2rad(gripper_deg)
            )
        with (
            patch.object(run_env, "ZMQClientRobot"),
            patch.object(run_env, "RobotEnv", TestEnv),
            patch.object(run_env.glob, "glob", return_value=[CR7_PORT]),
            patch(
                "xcore_gello_software.agents.gello_agent.os.path.exists",
                return_value=True,
            ),
            patch(
                "xcore_gello_software.dynamixel.driver.DynamixelDriver",
                return_value=driver,
            ),
            patch("xcore_gello_software.utils.control_utils.run_control_loop") as loop,
        ):
            run_env.main(
                run_env.Args(
                    agent="gello", gripper_only=gripper_only, read_gripper=read_gripper
                )
            )
            self.startup_steps = steps
            return loop.called

    def test_default_command_accepts_equivalent_cr7_pose(self):
        self.assertTrue(self.run_cr7_startup(np.array([0, 0, 0, 0, 6.381, 12.551])))

    def test_genuine_pose_mismatch_still_blocks_startup(self):
        self.assertFalse(
            self.run_cr7_startup(np.array([0, 0, 0, 0, 2 * np.pi + 1.2, 4 * np.pi]))
        )

    def test_closed_trigger_does_not_fail_arm_alignment_gate(self):
        self.assertTrue(self.run_cr7_startup(np.zeros(6), gripper_deg=153.0))

    def test_gripper_only_holds_existing_arm_pose_despite_leader_mismatch(self):
        self.assertTrue(
            self.run_cr7_startup(np.full(6, 2.0), gripper_deg=153.0, gripper_only=True)
        )
        self.assertTrue(self.startup_steps)
        for command in self.startup_steps:
            np.testing.assert_allclose(command[:6], 0.3)
        self.assertGreater(self.startup_steps[-1][-1], 0.95)

    def test_explicit_arm_only_mode_remains_six_channels(self):
        self.assertTrue(self.run_cr7_startup(np.zeros(6), read_gripper=False))


if __name__ == "__main__":
    with contextlib.redirect_stdout(io.StringIO()):
        unittest.main()
