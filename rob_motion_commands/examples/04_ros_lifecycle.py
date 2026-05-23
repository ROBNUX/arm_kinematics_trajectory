#!/usr/bin/env python3
"""Example 04 — ROS 2 context lifecycle managed by the rob_motion_commands module.

CreateRobot constructs a rclcpp::Node and publishers in its constructor;
without a running context that would throw at the first create_publisher
call. The pybind module's PYBIND11_MODULE init calls rclcpp::init and
registers a Python atexit handler that calls rclcpp::shutdown — so import
alone is enough to make CreateRobot's ctor work, and the interpreter's exit
hook performs clean teardown after the Python destructors finish.

This script just demonstrates the helpers and the idempotency of init.
"""

import rob_motion_commands as m


def main() -> int:
    print(f"ros_ok at import:    {m.ros_ok()}")

    # ros_init is a no-op when the context is already running.
    m.ros_init()
    m.ros_init()
    print(f"ros_ok after repeated ros_init: {m.ros_ok()}")

    # Tear down explicitly to demonstrate the helper. In normal use the
    # atexit handler does this for you.
    m.ros_shutdown()
    print(f"ros_ok after ros_shutdown: {m.ros_ok()}")

    # Re-initialize for any subsequent work in the same process.
    m.ros_init()
    print(f"ros_ok after re-init: {m.ros_ok()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
