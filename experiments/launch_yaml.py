"""Script entry point for xcore-gello-software launch-yaml."""

import runpy

if __name__ == "__main__":
    runpy.run_module("xcore_gello_software.commands.launch_yaml", run_name="__main__")
