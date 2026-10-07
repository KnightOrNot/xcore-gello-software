from pathlib import Path
from unittest.mock import Mock, patch

import mujoco
import numpy as np
import pytest

from xcore_gello_software.agents.gello_agent import GelloAgent, PORT_CONFIG_MAP
from xcore_gello_software.robots.dynamixel import DynamixelRobot
from xcore_gello_software.robots.sim_robot import MujocoRobotServer
from xcore_gello_software.dynamixel.driver import DynamixelDriver


PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"
GRIPPER_XML = (
    Path(__file__).resolve().parents[1]
    / "xcore_gello_software/robots/assets/cr7_parallel_gripper.xml"
)


def test_missing_encoder_data_reports_the_configured_ids_instead_of_waiting_forever():
    driver = DynamixelDriver.__new__(DynamixelDriver)
    driver._is_fake = False
    driver._joint_angles = None
    driver._ids = (1, 2, 3, 4, 5, 6, 7)
    with patch(
        "xcore_gello_software.dynamixel.driver.time.monotonic", side_effect=[0.0, 6.0]
    ):
        with pytest.raises(RuntimeError, match="1, 2, 3, 4, 5, 6, 7"):
            driver.get_joints()


@pytest.mark.parametrize(
    "degrees,closure",
    [(194.8, 0.0), (173.9, 0.5), (153.0, 1.0), (210.0, 0.0), (140.0, 1.0)],
)
def test_cr7_reads_id7_and_reports_normalized_trigger(degrees, closure):
    config = PORT_CONFIG_MAP[PORT]
    driver = Mock()
    driver.get_joints.return_value = np.append(
        config.joint_offsets, np.deg2rad(degrees)
    )
    with patch(
        "xcore_gello_software.dynamixel.driver.DynamixelDriver", return_value=driver
    ) as create:
        robot = config.make_robot(PORT)
    assert tuple(create.call_args.args[0]) == (1, 2, 3, 4, 5, 6, 7)
    assert create.call_args.kwargs["use_fake_fallback"] is False
    assert robot.get_joint_state().shape == (7,)
    state = robot.get_gripper_state()
    assert state["closure"] == pytest.approx(closure)
    assert state["openness"] == pytest.approx(1 - closure)
    assert state["raw_degrees"] == pytest.approx(degrees)


def test_invalid_trigger_endpoints_are_rejected_before_opening_serial():
    with patch("xcore_gello_software.dynamixel.driver.DynamixelDriver") as create:
        with pytest.raises(ValueError, match="finite and different"):
            DynamixelRobot((1,), (0,), (1,), real=True, gripper_config=(7, 100, 100))
        create.assert_not_called()


def test_endpoint_overrides_do_not_change_global_port_configuration():
    config = PORT_CONFIG_MAP[PORT]
    driver = Mock()
    driver.get_joints.return_value = np.append(config.joint_offsets, np.deg2rad(180))
    with patch(
        "xcore_gello_software.dynamixel.driver.DynamixelDriver", return_value=driver
    ):
        agent = GelloAgent(PORT, config, gripper_open_deg=200, gripper_close_deg=160)
    assert agent.act({})[-1] == pytest.approx(0.5)
    assert PORT_CONFIG_MAP[PORT].gripper_config == (7, 194.8, 153.0)


@pytest.fixture
def arm_xml(tmp_path):
    bodies = (
        "".join(
            f'<body name="link{i}" pos="0 0 0.1"><joint name="j{i}" '
            f'type="hinge" axis="0 0 1"/><geom type="capsule" size="0.02 0.04"/>'
            for i in range(6)
        )
        + "</body>" * 6
    )
    actuators = "".join(f'<position joint="j{i}" kp="100" kv="10"/>' for i in range(6))
    xml = tmp_path / "arm.xml"
    xml.write_text(
        f'<mujoco><option gravity="0 0 0"/><worldbody>{bodies}</worldbody>'
        f"<actuator>{actuators}</actuator></mujoco>"
    )
    return xml


@pytest.mark.parametrize("closure", [0.0, 0.5, 1.0])
def test_attached_gripper_physics_and_feedback_use_the_same_closure_units(
    arm_xml, closure
):
    with patch("xcore_gello_software.robots.sim_robot.ZMQRobotServer"):
        server = MujocoRobotServer(
            arm_xml,
            GRIPPER_XML,
            arm_dofs=6,
            gripper_body="link5",
            normalize_gripper=True,
        )
    try:
        assert server._model.nq == 8  # Six arm joints and two coupled fingers.
        assert server.num_dofs() == 7  # One scalar closure command for both fingers.
        server.command_joint_state(np.array([0.0] * 6 + [closure]))
        assert server.get_ctrl()[-1] == pytest.approx(closure * 0.0425)
        server._data.ctrl[:] = server.get_ctrl()
        mujoco.mj_step(server._model, server._data, nstep=1000)
        obs = server.get_observations()
        assert obs["joint_positions"].shape == (7,)
        assert obs["joint_positions"][-1] == pytest.approx(closure, abs=0.01)
        assert float(obs["gripper_position"]) == pytest.approx(closure, abs=0.01)
        assert server.get_joint_state()[-1] == pytest.approx(closure, abs=0.01)
        assert server._data.qpos[-2] == pytest.approx(server._data.qpos[-1], abs=1e-4)
    finally:
        server.stop()


def test_arm_only_simulation_keeps_six_channels(arm_xml):
    with patch("xcore_gello_software.robots.sim_robot.ZMQRobotServer"):
        server = MujocoRobotServer(arm_xml, arm_dofs=6)
    try:
        assert server.num_dofs() == 6
        assert server.get_observations()["joint_positions"].shape == (6,)
        assert float(server.get_observations()["gripper_position"]) == 0
    finally:
        server.stop()
