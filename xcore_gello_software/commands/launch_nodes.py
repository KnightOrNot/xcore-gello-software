from dataclasses import dataclass
from pathlib import Path

import tyro

from xcore_gello_software.robots.robot import BimanualRobot, PrintRobot
from xcore_gello_software.zmq_core.robot_node import ZMQServerRobot


@dataclass
class Args:
    robot: str = "xarm"
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    robot_ip: str = "192.168.1.10"
    with_gripper: bool = True
    """Include the parallel gripper when launching sim_cr7."""


def launch_robot_server(args: Args):
    port = args.robot_port
    if args.robot == "sim_ur":
        MENAGERIE_ROOT: Path = (
            Path(__file__).resolve().parents[2] / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "universal_robots_ur5e" / "ur5e.xml"
        gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()
    elif args.robot == "sim_ur10":
            MENAGERIE_ROOT: Path = (
                Path(__file__).resolve().parents[2] / "third_party" / "mujoco_menagerie"
            )
            xml = MENAGERIE_ROOT / "universal_robots_ur10e" / "ur10e.xml"
            gripper_xml = MENAGERIE_ROOT / "robotiq_2f85" / "2f85.xml"
            from xcore_gello_software.robots.sim_robot import MujocoRobotServer

            server = MujocoRobotServer(
                xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
            )
            server.serve()
    elif args.robot == "sim_iimt5":
            from xcore_gello_software.robots.sim_robot import MujocoRobotServer

            xml = Path(__file__).resolve().parents[2] / "third_party" / "iimt5" / "iimt5_scene_position.xml"
            gripper_xml = None
            server = MujocoRobotServer(
                xml_path=xml, gripper_xml_path=None, port=port, host=args.hostname,
                print_joints=False,
            )
            server.serve()

    elif args.robot == "sim_nova2":
            from xcore_gello_software.robots.sim_robot import MujocoRobotServer

            xml = Path(__file__).resolve().parents[2] / "third_party" / "nova2" / "nova2_position.xml"
            gripper_xml = None
            server = MujocoRobotServer(
                xml_path=xml, gripper_xml_path=None, port=port, host=args.hostname,
                print_joints=False,
            )
            server.serve()

    elif args.robot == "sim_cr7":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "cr7" / "cr7_scene.xml"
        if not xml.is_file():
            raise FileNotFoundError(f"模型不存在：{xml}")

        server = MujocoRobotServer(
            xml_path=xml,
            gripper_xml_path=(
                Path(__file__).resolve().parents[1] / "robots/assets/cr7_parallel_gripper.xml"
                if args.with_gripper else None
            ),
            port=port,
            host=args.hostname,
            arm_dofs=6,
            gripper_body="XMC7-R850-W4X3B4_link6" if args.with_gripper else None,
            normalize_gripper=args.with_gripper,
            print_joints=False,
        )
        try:
            server.serve()
        finally:
            server.stop()

    elif args.robot == "sim_yam":
        MENAGERIE_ROOT: Path = (
            Path(__file__).resolve().parents[2] / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "i2rt_yam" / "yam.xml"
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=None, port=port, host=args.hostname,
            arm_dofs=6, gripper_reverse=True,
        )
        server.serve()
    elif args.robot == "sim_piper":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).resolve().parents[2] / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "agilex_piper" / "scene.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_panda":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).resolve().parents[2] / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "franka_emika_panda" / "panda.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_fr3":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).resolve().parents[2] / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "franka_fr3" / "fr3.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_xarm":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        MENAGERIE_ROOT: Path = (
            Path(__file__).resolve().parents[2] / "third_party" / "mujoco_menagerie"
        )
        xml = MENAGERIE_ROOT / "ufactory_xarm7" / "xarm7.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_ar5":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "ar5" / "ar5_scene.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname,
            print_joints=True,
        )
        server.serve()

    elif args.robot == "sim_ar5r":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "ar5r" / "ar5r_scene.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname
        )
        server.serve()

    elif args.robot == "sim_cr5":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "cr5" / "cr5_scene.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname,
            arm_dofs=6,  # CR5: 6 arm joints, no gripper
        )
        server.serve()

    elif args.robot == "sim_rm65":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "rm65" / "rm65_scene.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname,
            arm_dofs=6,  # RM65: 6 arm joints, no gripper
        )
        server.serve()

    elif args.robot == "sim_fr5v6":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "fr5v6" / "fr5v6_scene.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname,
            arm_dofs=6,  # FR5V6: 6 arm joints, no gripper
        )
        server.serve()

    elif args.robot == "sim_fr5":
        # alias: FR5 仿真使用 FR5V6 模型 (frcobot_ros 中 fr5v6 为最新版)
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "fr5v6" / "fr5v6_scene.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname,
            arm_dofs=6,  # FR5V6: 6 arm joints, no gripper
        )
        server.serve()

    elif args.robot == "sim_rizon4":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "rizon4" / "rizon4_scene_position.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname,
            arm_dofs=7,  # Rizon4: 7 arm joints, no gripper
        )
        server.serve()

    elif args.robot == "sim_lebailm3":
        from xcore_gello_software.robots.sim_robot import MujocoRobotServer

        xml = Path(__file__).resolve().parents[2] / "third_party" / "lebai_lm3" / "lebai_lm3_scene.xml"
        gripper_xml = None
        server = MujocoRobotServer(
            xml_path=xml, gripper_xml_path=gripper_xml, port=port, host=args.hostname,
            arm_dofs=6,  # Lebai LM3: 6 arm joints, no gripper
        )
        server.serve()

    else:
        if args.robot == "xarm":
            from xcore_gello_software.robots.xarm_robot import XArmRobot

            robot = XArmRobot(ip=args.robot_ip)
        elif args.robot == "ur":
            from xcore_gello_software.robots.ur import URRobot

            robot = URRobot(robot_ip=args.robot_ip)
        elif args.robot == "panda":
            from xcore_gello_software.robots.panda import PandaRobot

            robot = PandaRobot(robot_ip=args.robot_ip)
        elif args.robot == "bimanual_ur":
            from xcore_gello_software.robots.ur import URRobot

            # IP for the bimanual robot setup is hardcoded
            _robot_l = URRobot(robot_ip="192.168.2.10")
            _robot_r = URRobot(robot_ip="192.168.1.10")
            robot = BimanualRobot(_robot_l, _robot_r)
        elif args.robot == "yam":
            from xcore_gello_software.robots.yam import YAMRobot

            robot = YAMRobot(channel="can0")
        elif args.robot == "none" or args.robot == "print":
            robot = PrintRobot(8)

        else:
            raise NotImplementedError(
                f"Robot {args.robot} not implemented, choose one of: sim_ur, sim_yam, sim_panda, sim_fr3, sim_xarm, sim_ar5, sim_ar5r, sim_cr5, sim_rm65, sim_fr5v6, sim_rizon4, sim_marvin, sim_lebailm3, xarm, ur, panda, bimanual_ur, yam, none"
            )
        server = ZMQServerRobot(robot, port=port, host=args.hostname)
        print(f"Starting robot server on port {port}")
        server.serve()


def main(args):
    launch_robot_server(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
