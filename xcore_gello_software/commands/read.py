"""Read the GELLO leader without a follower server or a physical gripper."""

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import tyro

from xcore_gello_software.agents.gello_agent import GelloAgent


@dataclass
class Args:
    gello_port: str = (
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"
    )
    hz: float = 10.0
    samples: int = 0
    """Number of readings; zero means until Ctrl+C."""
    gripper_open_deg: Optional[float] = None
    gripper_close_deg: Optional[float] = None


def main(args):
    if not np.isfinite(args.hz) or args.hz <= 0 or args.samples < 0:
        raise ValueError("hz must be positive and finite; samples must be nonnegative")
    agent = GelloAgent(
        args.gello_port,
        gripper_open_deg=args.gripper_open_deg,
        gripper_close_deg=args.gripper_close_deg,
    )
    try:
        count = 0
        while args.samples == 0 or count < args.samples:
            started = time.monotonic()
            joints = agent.act({})
            gripper = agent.get_gripper_state()
            arm = joints[:-1] if agent.has_gripper else joints
            print(f"arm_rad={np.round(arm, 3).tolist()}", end=" ")
            if gripper is not None:
                print(
                    f"gripper_raw_deg={gripper['raw_degrees']:.2f} "
                    f"closure={gripper['closure']:.3f} openness={gripper['openness']:.3f}",
                    flush=True,
                )
            else:
                print("gripper=disabled", flush=True)
            count += 1
            if args.samples == 0 or count < args.samples:
                time.sleep(max(0, 1.0 / args.hz - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        agent.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
