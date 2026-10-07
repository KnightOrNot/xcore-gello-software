"""Script entry point for xcore-gello-software camera-client."""

import runpy

if __name__ == "__main__":
    runpy.run_module(
        "xcore_gello_software.commands.launch_camera_clients", run_name="__main__"
    )
