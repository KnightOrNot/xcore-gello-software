import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Optional[Tuple[int, int, int]] = None
    """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position). Set to None for robots without a gripper."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
        )


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # xArm
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            3 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
            4 * np.pi / 2,
            -2 * np.pi / 2 + 2 * np.pi,
            3 * np.pi / 2,
            4 * np.pi / 2,
        ),
        joint_signs=(1, -1, 1, 1, 1, -1, 1),
        gripper_config=(8, 195, 152),
    ),

    #Piper
    "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_85CE3ABA503059384C2E3120FF070B32-if00": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
        ),
        joint_signs=(1, 1, 1, 1, -1, 1),
        gripper_config=(7, 165, 123),
    ),

    # FR3
    "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_85CE3ABA503059384C2E3120FF070B32-if00": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
            3 * np.pi / 2,
        ),
        joint_signs=(1, 1, 1, 1, 1, -1, 1),
        gripper_config=(8, 165, 123),
    ),

     # AR5 left
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBHZC6C-if00-port0": DynamixelRobotConfig(
            joint_ids=(1, 2, 3, 4, 5, 6, 7),
            joint_offsets=(
                2 * np.pi / 2,  # J1: π (180°)
                2 * np.pi / 2,  # J2: π (180°)
                2 * np.pi / 2,  # J3: π (180°)
                3 * np.pi / 2,  # J4: 3π/2 (270°) 
                2 * np.pi / 2,  # J5: π (180°)
                2 * np.pi / 2,  # J6: π (180°)
                2 * np.pi / 2,  # J7: π (180°)
            ),
            joint_signs=(1, 1, 1, -1, 1, 1, 1),  # J4 sign 反向
            #gripper_config=(8, 165, 123),
        ),

        # AR5 right
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBA1G9M-if00-port0": DynamixelRobotConfig(
            joint_ids=(1, 2, 3, 4, 5, 6, 7),
            joint_offsets=(
                2 * np.pi / 2,  # J1: π (180°)
                2 * np.pi / 2,  # J2: π (180°)
                2 * np.pi / 2,  # J3: π (180°)
                3 * np.pi / 2,  # J4: 3π/2 (270°) 
                2 * np.pi / 2,  # J5: π (180°)
                2 * np.pi / 2,  # J6: π (180°)
                2 * np.pi / 2,  # J7: π (180°)
            ),
            joint_signs=(1, 1, 1, -1, 1, 1, 1),  # J4 sign 反向
            #gripper_config=(8, 165, 123),
        ),

    # yam
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBTCIGC-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=[
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
        ],
        joint_signs=(1, -1, -1, -1, 1, 1),
        gripper_config=(7, 200, 157),  # calibrated: open(rest)≈200°, close(squeezed)≈157°
    ),

    # UR10e
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            2 * np.pi / 2,
            2 * np.pi / 2 ,
            2 * np.pi / 2 ,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 20, -22),
    ),

    #iimt5
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB5RUGG-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            3 * np.pi / 2,
            3 * np.pi / 2 ,
            2 * np.pi / 2 ,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=None #(7, 198, 156),
    ),


    #CR7
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            3 * np.pi / 2,
            2 * np.pi / 2 ,
            2 * np.pi / 2 ,
            2 * np.pi / 2,
            2 * np.pi / 2,
            -2 * np.pi / 2,
        ),
        joint_signs=(1, 1, 1, 1, 1, 1),
        gripper_config=None #(7, 194.8, 153),
    ),


    #nova2
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB5RUGG-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            3 * np.pi / 2,
            2 * np.pi / 2 ,
            2 * np.pi / 2 ,
            1 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=None #(7, 198, 156),
    ),



    # Left UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            0,
            1 * np.pi / 2 + np.pi,
            np.pi / 2 + 0 * np.pi,
            0 * np.pi + np.pi / 2,
            np.pi - 2 * np.pi / 2,
            -1 * np.pi / 2 + 2 * np.pi,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 20, -22),
    ),
    # Right UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            np.pi + 0 * np.pi,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            1 * np.pi,
            3 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 286, 248),
    ),
    # FR5
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBNWWUF-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            3 * np.pi / 2,
            3 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=None #(7, 20, -22),
    ),
    # lebai_lm3
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB7WPRN-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            1 * np.pi / 2,
            3 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=None #(7, 20, -22),
    ),
    #feixi
    #"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8X5NZ-if00-port0": DynamixelRobotConfig(
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTC2K1S1-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
        ),
        joint_signs=(1, 1, 1, 1, 1, 1, 1),
        gripper_config=None #(7, 20, -22),
    ),

    
}


class GelloAgent(Agent):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        # Ensure start_joints is a numpy array if provided
        if start_joints is not None and not isinstance(start_joints, np.ndarray):
            start_joints = np.array(start_joints)
        if dynamixel_config is not None:
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        else:
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

            config = PORT_CONFIG_MAP[port]
            self._robot = config.make_robot(port=port, start_joints=start_joints)

    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        return self._robot.get_joint_state()
