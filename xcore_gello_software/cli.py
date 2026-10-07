"""Dispatch the existing GELLO experiments through one installed command."""

import runpy
import sys


COMMANDS = {
    "read": "read",
    "launch-nodes": "launch_nodes",
    "run-env": "run_env",
    "launch-yaml": "launch_yaml",
    "quick-run": "quick_run",
    "camera-server": "launch_camera_nodes",
    "camera-client": "launch_camera_clients",
}


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in {"-h", "--help"}:
        print("usage: xcore-gello-software COMMAND [OPTIONS]\n")
        print("commands: " + ", ".join(COMMANDS))
        print("\nRun `xcore-gello-software COMMAND --help` for options.")
        return 0
    command = args.pop(0)
    if command not in COMMANDS:
        print(f"xcore-gello-software: unknown command: {command!r}", file=sys.stderr)
        return 2
    original = sys.argv
    try:
        sys.argv = [f"xcore-gello-software {command}", *args]
        runpy.run_module(
            f"xcore_gello_software.commands.{COMMANDS[command]}",
            run_name="__main__",
        )
    finally:
        sys.argv = original
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
