#!/usr/bin/env python3

import runpy


def main():
    # Delegate to the simulator entrypoint
    runpy.run_module("sim.main", run_name="__main__")


if __name__ == "__main__":
    main()
