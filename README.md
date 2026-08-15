# simplebgc32-ros2

ROS 2 driver for SimpleBGC32 (BaseCam / AlexMos) 32-bit brushless gimbal controllers.

Built on the byte-verified C protocol core from
[simplebgc32-control](https://github.com/magdang/simplebgc32-control), vendored as a git
submodule so the wire format has exactly one implementation shared by this driver and the
upstream standalone tools.

> [!CAUTION]
> This is experimental software that moves physical hardware. It offers no safety
> guarantees. Nothing here — travel limits, command watchdogs, arming gates, stop
> services — is a substitute for an independent motor power cutoff. See
> [Safety](#safety) before connecting a gimbal that can hit something.

Target distro: **ROS 2 Jazzy** (primary), **Lyrical Luth** (forward target).

## Status

Under construction. Implemented so far:

| Package | State |
|---|---|
| `sbgc_protocol` | ✅ ament wrapper around the vendored C core; upstream's 37 byte-level vendor assertions run under `colcon test` |
| `sbgc_interfaces` | planned |
| `sbgc_driver` | planned |
| `sbgc_description` | planned |
| `sbgc_bringup` | planned |

## Getting the source

```bash
git clone --recurse-submodules https://github.com/…/simplebgc32-ros2.git
# or, in an existing clone:
git submodule update --init --recursive
```

## Building

With ROS sourced, this is an ordinary colcon workspace:

```bash
colcon build --symlink-install && colcon test && colcon test-result --verbose
```

If you do not have ROS installed locally, the `Makefile` runs the same commands inside the
official `ros:jazzy-ros-base` image:

```bash
make test                      # build + test in a container
make test ROS_DISTRO=lyrical   # forward target
make shell                     # interactive container, workspace mounted
```

## Design

Full design notes, including why this is a lifecycle node rather than a `ros2_control`
hardware component, live in `docs/`. In brief:

- **Standard ROS interfaces.** `sensor_msgs/JointState`, `sensor_msgs/Imu`,
  `sensor_msgs/BatteryState`, `control_msgs/JointJog`,
  `control_msgs/action/FollowJointTrajectory`, `std_srvs`, `diagnostic_msgs`. Custom
  types only for board identity, driver status, control-mode selection and gyro
  calibration.
- **SI units at the boundary.** Radians and rad/s per
  [REP-103](https://www.ros.org/reps/rep-0103.html). The board's fixed-point units never
  leak into the ROS API.
- **`sbgc_protocol` has no ROS dependency**, so the protocol layer stays usable — and
  testable — outside ROS.

## Safety

The upstream project's warnings apply in full here, and one of them constrains how you
should integrate this driver:

> The watchdog can stop motion only while it can send a stop frame. If the serial
> connection fails while a continuous rate is active, the controller may keep turning at
> the last commanded rate.

For any setup where continued motion can cause harm, use an independent motor power cutoff,
and prefer bounded angle targets (`FollowJointTrajectory`) over continuous rate commands
(`JointJog`).

Please do not describe any software interlock, watchdog, travel limit or stop command in
this repository as a safety guarantee.

## Licence

MIT. Unofficial and not affiliated with BaseCam Electronics / AlexMos.
