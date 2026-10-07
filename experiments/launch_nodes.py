"""Script entry point for xcore-gello-software launch-nodes."""

import runpy

if __name__ == "__main__":
    runpy.run_module("xcore_gello_software.commands.launch_nodes", run_name="__main__")
