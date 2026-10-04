import contextlib
import io
import unittest
from unittest.mock import Mock, patch

import numpy as np

from experiments import run_env
from gello.agents.gello_agent import PORT_CONFIG_MAP
from gello.robots.dynamixel import DynamixelRobot


CR7_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"


class StartAlignmentTests(unittest.TestCase):
    def make_robot(self, raw, **kwargs):
        driver = Mock()
        driver.get_joints.return_value = np.array(raw, dtype=float)
        with patch("gello.dynamixel.driver.DynamixelDriver", return_value=driver):
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
            joint_ids=(1, 2), joint_offsets=offsets, joint_signs=signs,
            start_joints=reference,
        )
        np.testing.assert_allclose(robot.get_joint_state(), reference, atol=1e-12)

    def test_alignment_preserves_gripper_normalization(self):
        robot, _ = self.make_robot(
            [2 * np.pi + 0.1, -4 * np.pi - 0.2, 0.75 * np.pi],
            joint_ids=(1, 2), joint_offsets=(0, 0), joint_signs=(1, 1),
            gripper_config=(3, 0, 180), start_joints=np.array([0, 0, 0.75]),
        )
        np.testing.assert_allclose(robot.get_joint_state(), [0.1, -0.2, 0.75], atol=1e-12)

    def test_motion_remains_continuous_across_pi(self):
        robot, driver = self.make_robot(
            [2 * np.pi + 3.13], joint_ids=(1,), start_joints=np.array([3.13]),
        )
        robot.get_joint_state()
        driver.get_joints.return_value += 0.04
        self.assertAlmostEqual(robot.get_joint_state()[0], 3.13 + 0.99 * 0.04)

    def run_cr7_startup(self, reported):
        class TestEnv:
            def __init__(self, *args, **kwargs):
                self.joints = np.zeros(6)

            def get_obs(self):
                return {"joint_positions": self.joints.copy()}

            def step(self, joints):
                self.joints = joints.copy()

        driver = Mock()
        driver.get_joints.return_value = (
            np.array(PORT_CONFIG_MAP[CR7_PORT].joint_offsets) + reported
        )
        with (
            patch.object(run_env, "ZMQClientRobot"),
            patch.object(run_env, "RobotEnv", TestEnv),
            patch.object(run_env.glob, "glob", return_value=[CR7_PORT]),
            patch("gello.agents.gello_agent.os.path.exists", return_value=True),
            patch("gello.dynamixel.driver.DynamixelDriver", return_value=driver),
            patch("gello.utils.control_utils.run_control_loop") as loop,
        ):
            run_env.main(run_env.Args(agent="gello"))
            return loop.called

    def test_default_command_accepts_equivalent_cr7_pose(self):
        self.assertTrue(self.run_cr7_startup(np.array([0, 0, 0, 0, 6.381, 12.551])))

    def test_genuine_pose_mismatch_still_blocks_startup(self):
        self.assertFalse(self.run_cr7_startup(np.array([0, 0, 0, 0, 2 * np.pi + 1.2, 4 * np.pi])))


if __name__ == "__main__":
    with contextlib.redirect_stdout(io.StringIO()):
        unittest.main()
