# simplebgc32-ros2

ROS 2 driver for SimpleBGC32 (BaseCam / AlexMos) 32-bit brushless gimbal controllers.

Built on the byte-verified C protocol core from
[simplebgc32-control](https://github.com/magdang/simplebgc32-control), carried in-tree so
this repository is self-contained: clone it and build.

> [!CAUTION]
> This is experimental software that moves physical hardware. It offers no safety
> guarantees. Nothing here — travel limits, command watchdogs, arming gates, stop
> services — is a substitute for an independent motor power cutoff. Read
> [Safety](#safety) before connecting a gimbal that can hit something.

Target distro: **ROS 2 Jazzy** (primary), **Lyrical Luth** (forward target).

## Packages

| Package | What it is |
|---|---|
| `sbgc_protocol` | The vendored C protocol core as an ament library. No ROS dependency. |
| `sbgc_interfaces` | Messages, services and actions. Depends on nothing else here. |
| `sbgc_driver` | The lifecycle node, plus `GimbalCore`|
| `sbgc_description` | URDF/xacro and TF frames. |
| `sbgc_bringup` | Launch files and configuration.|

## Getting the source

```bash
git clone https://github.com/cmu-impactlab/simplebgc32_ROS2.git
```

That is all of it. The repository is self-contained: no submodules, and nothing else to
fetch. Drop it into a colcon workspace and build.

The SimpleBGC wire protocol is C code copied in under
`sbgc_protocol/vendor/`, from
[simplebgc32-control](https://github.com/magdang/simplebgc32-control) (MIT). It is copied
rather than referenced so that cloning gives you a working tree every time.
`sbgc_protocol/vendor/README.md` records which upstream commit these files came from, and
`sbgc_protocol/vendor/sync.sh` takes a newer one when you want it. Do not edit those files
in place — a protocol change belongs upstream, where its own byte-level tests cover it.

## Building

With ROS sourced this is an ordinary colcon workspace:

```bash
colcon build && colcon test && colcon test-result --verbose
```

Without a local ROS install, the `Makefile` runs the same thing in a container:

```bash
make test                      # build + test
make test ROS_DISTRO=lyrical   # forward target
make shell                     # interactive container, workspace mounted
```

## Running

```bash
ros2 launch sbgc_bringup gimbal.launch.py port:=/dev/simplebgc allow_control:=true
ros2 lifecycle set /sbgc_driver configure
ros2 lifecycle set /sbgc_driver activate
ros2 service call /sbgc_driver/arm std_srvs/srv/SetBool "{data: true}"
```

`allow_control` defaults to **false**, which brings the driver up as a monitor that will
publish telemetry and refuse to move anything. Motion needs that parameter *and* the `arm`
service — see [Safety](#safety).

With no hardware, point it at the upstream simulator instead:

```bash
python3 sbgc_protocol/vendor/test/sbgc_sim.py   # prints a pty path
ros2 launch sbgc_bringup gimbal.launch.py port:=/dev/pts/N allow_control:=true
```

For a stable device name, install `sbgc_driver/config/99-simplebgc.rules` into
`/etc/udev/rules.d/`. `/dev/ttyUSB0` renumbers depending on what was plugged in first.

## Interface

Standard ROS types wherever one exists. Custom messages only where nothing standard fits.

| Direction | Name | Type |
|---|---|---|
| pub | `/joint_states` | `sensor_msgs/JointState` |
| pub | `~/imu` | `sensor_msgs/Imu` |
| pub | `~/mount_orientation` | `geometry_msgs/QuaternionStamped` |
| pub | `~/battery` | `sensor_msgs/BatteryState` |
| pub | `~/status` | `sbgc_interfaces/GimbalStatus` |
| pub | `~/board_info` | `sbgc_interfaces/BoardInfo` (transient local) |
| pub | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| sub | `~/joint_jog` | `control_msgs/JointJog` |
| action | `~/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` |
| action | `~/calibrate_gyro` | `sbgc_interfaces/action/CalibrateGyro` |
| service | `~/arm`, `~/set_motors`, `~/set_lock_mode` | `std_srvs/SetBool` |
| service | `~/stop`, `~/home`, `~/level` | `std_srvs/Trigger` |
| service | `~/set_control_mode`, `~/get_board_info` | `sbgc_interfaces` |

Units at the boundary are SI per [REP-103](https://www.ros.org/reps/rep-0103.html):
radians and rad/s, right-handed, x forward, y left, z up. The board's fixed-point counts
and its positive-downward pitch never reach the ROS API.

### Conventions worth knowing

- **Positive pitch raises the camera.** The board's own pitch axis is positive downward;
  the driver inverts it. The URDF's pitch joint axis is `0 -1 0` for the same reason —
  with `0 1 0` the model would tilt opposite to the real gimbal.
- **Yaw is continuous.** `JointState.position` carries an unwrapped track, so a consumer
  building TF never sees the camera jump a full turn as yaw crosses 180°.
- **`JointState.effort` is empty.** The board reports a raw 0–255 drive level, which is
  not the newton-metres that field means.
- **`camera_optical_frame` is not `camera_link`.** The optical frame is z-forward,
  x-right, y-down. Confusing the two is the usual way a gimbal integration ends up
  pointing sideways.
- **`FollowJointTrajectory` is a reduced implementation.** It commands the final point and
  lets the board run its own slew, so intermediate waypoints are not replayed and
  `time_from_start` bounds the goal rather than pacing it. Goals carrying `path_tolerance`
  are *rejected* rather than silently accepted, since this server never observes the poses
  such a tolerance is defined against.

### Configuring the IMU for your mount

`orientation` comes from the board's own angles and needs no configuration.

The raw `angular_velocity` and `linear_acceleration` vectors do. The controller is a chip
in a case, and which way that case is bolted into a mount is a property of *your build*,
not of the protocol — two gimbals on identical firmware can present the same physical
motion on different axes and with different signs. So the mapping is
`imu_axis_map` and `imu_axis_sign` rather than a constant in the source.

**Measuring yours** takes about a minute and is entirely read-only: leave
`allow_control:=false` and the motors off throughout, and move the gimbal by hand. Nothing
below commands the motors — with the power off the stages turn freely.

```bash
ros2 topic echo /sbgc_driver/imu --field linear_acceleration
```

1. **Sit the gimbal level and still.** `z` should read about **+9.81**.
   `sensor_msgs/Imu` carries *proper acceleration*, so up is positive at rest. If you get
   −9.81, flip the third entry of `imu_axis_sign`. If gravity shows up on `x` or `y`
   instead, that axis is your vertical one — put its board index third in `imu_axis_map`.
2. **Tilt the camera nose-down.** `x` should go **negative**. If `y` moves instead, swap
   the first two entries of `imu_axis_map`; if `x` moves the wrong way, flip its sign.
3. **Roll the camera right-side-down.** `y` should go **negative**. Same remedies.

A worked example, for a board whose sensor is rotated so that yaw feeds x and roll feeds z,
with z inverted:

```yaml
imu_axis_map:  [2, 1, 0]     # x<-YAW, y<-PITCH, z<-ROLL
imu_axis_sign: [1.0, 1.0, -1.0]
```

`imu_axis_map` must be a permutation of `0,1,2`; anything else is refused with an error and
the identity is used, because a mapping that drops or duplicates an axis still produces a
plausible-looking vector.

The IMU's *frame* is configurable too. `sbgc_description` places `gimbal_imu_link` with the
`imu_parent` argument (`base`, `yaw`, `roll` or `pitch` — most mounts carry the board on
the pitch stage) plus `imu_xyz`/`imu_rpy`, which is where a board mounted at an angle is
described.

Covariances are all-zero, which the message defines as "unknown" — the board publishes no
uncertainty and this driver has measured none. Set `publish_imu: false` to drop the topic
entirely.

## Safety

Two gates guard every motion command, and both are required: the `allow_control`
parameter, which is read-only and set at launch, and the `arm` service, which is switched
at runtime. The check is repeated at the point of transmission, not only where a request
arrives, so a future change that routes around the front door still cannot reach the
motors.

`stop` is deliberately **not** gated. Refusing to stop would invert the purpose of a guard
that exists to prevent unwanted motion.

Other behaviour that exists for a reason:

- A command older than `command_timeout` (0.5 s) holds. A publisher that dies stops the
  camera rather than leaving it running.
- Travel limits are held, not enforced, when the reported angle is stale. A board that
  stops answering leaves the last angle frozen and perfectly plausible; nothing would ever
  read as "at the limit" and the camera would drive straight through.
- A command that arrives while a gate is shut is dropped, not queued. Arming would
  otherwise start the gimbal on an instruction given before the operator armed.
- Deactivating the lifecycle node sends a hold frame. Stopping does not depend on a
  service being reachable.
- Motors are not powered at startup unless `auto_motors_on` is set.

### Critical serial-disconnect limitation

Carried over from the upstream project, because it constrains how you should integrate
this driver:

> The watchdog can stop motion only while it can send a stop frame. If the serial
> connection fails while a continuous rate is active, the controller may keep turning at
> the last commanded rate.

For any setup where continued motion can cause harm, use an independent motor power
cutoff, and prefer bounded angle targets (`FollowJointTrajectory`) over continuous rate
commands (`JointJog`).

The travel limits in this driver are a convenience, not the real protection. Set the
authoritative limits in the [SimpleBGC GUI](https://www.basecamelectronics.com/downloads/32bit/)

## Design notes

**Why a lifecycle node and not `ros2_control`.** The board runs its own stabilisation
loop, roll is gravity-referenced rather than joint-referenced, and yaw is a continuous
unwrapped board count. A `SystemInterface` would have to misrepresent all three, and
`read()`/`write()` called synchronously from the controller manager would fight a UART
that answers on its own schedule. A `SystemInterface` remains possible later as a separate
optional package for users who want MoveIt; `sbgc_driver` must never depend on
`ros2_control`.

**Why the decisions are in `GimbalCore`.** Safety rules that can only be exercised by
standing up a node, a serial port and an executor do not get exercised. `GimbalCore` has
no ROS includes and carries 36 unit tests asserting what reaches the wire.

**Threading.** `sbgc_t` has no internal locking. Every callback the driver creates that
touches the port is in one mutually-exclusive callback group, and the shipped executable
spins single-threaded, so that is sufficient there. Do not add a port-touching callback
outside that group. The lifecycle transition callbacks are the exception and cannot join
it — they are served by the LifecycleNode's own machinery, and `on_deactivate`/`on_cleanup`
touch the link. Loading this component into a *multi-threaded* container therefore needs
the host to serialise lifecycle transitions against everything else — the timers, the
services and the action callbacks alike, since those read lifecycle state too.

## Testing

```bash
make test
```

