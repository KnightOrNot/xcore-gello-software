"""Read the GELLO leader without a follower server or a physical gripper."""

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import tyro

from xcore_gello_software.agents.gello_agent import GelloAgent
from xcore_gello_software.configuration import load_config


@dataclass
class Args:
    gello_port: Optional[str] = None
    """Stable by-id serial port; defaults to config/default.toml."""
    hz: Optional[float] = None
    samples: Optional[int] = None
    """Number of readings; zero means until Ctrl+C."""
    gripper_open_deg: Optional[float] = None
    gripper_close_deg: Optional[float] = None


def main(args):
    defaults = load_config().read
    gello_port = args.gello_port or defaults.gello_port
    hz = defaults.hz if args.hz is None else args.hz
    samples = defaults.samples if args.samples is None else args.samples
    gripper_open_deg = args.gripper_open_deg
    gripper_close_deg = args.gripper_close_deg
    if (
        gripper_open_deg is None
        and gripper_close_deg is None
        and not defaults.gripper_use_library_endpoints
    ):
        gripper_open_deg = defaults.gripper_open_deg
        gripper_close_deg = defaults.gripper_close_deg
    if not np.isfinite(hz) or hz <= 0 or samples < 0:
        raise ValueError("hz must be positive and finite; samples must be nonnegative")
    agent = GelloAgent(
        gello_port,
        gripper_open_deg=gripper_open_deg,
        gripper_close_deg=gripper_close_deg,
    )
    try:
        count = 0
        while samples == 0 or count < samples:
            started = time.monotonic()
            joints = agent.act({})
            gripper = agent.get_gripper_state()
            arm = joints[:-1] if agent.has_gripper else joints
            print(f"arm_rad={np.round(arm, 3).tolist()}", end=" ")
            if gripper is not None:
                print(
                    f"gripper_raw_deg={gripper['raw_degrees']:.2f} "
                    f"closure={gripper['closure']:.3f} "
                    f"openness={gripper['openness']:.3f}",
                    flush=True,
                )
            else:
                print("gripper=disabled", flush=True)
            count += 1
            if samples == 0 or count < samples:
                time.sleep(max(0, 1.0 / hz - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        agent.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
