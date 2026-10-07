"""Script entry point for xcore-gello-software run-env."""

import runpy

if __name__ == "__main__":
    runpy.run_module("xcore_gello_software.commands.run_env", run_name="__main__")
