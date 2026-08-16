# simplebgc32-ros2

ROS 2 driver for SimpleBGC32 (BaseCam / AlexMos) 32-bit brushless gimbal controllers.

Built on the byte-verified C protocol core from
[simplebgc32-control](https://github.com/magdang/simplebgc32-control), vendored as a git
submodule so the wire format has exactly one implementation shared by this driver and the
upstream standalone tools.

> [!CAUTION]
> This is experimental software that moves physical hardware. It offers no safety
> guarantees. Nothing here — travel limits, command watchdogs, arming gates, stop
> services — is a substitute for an independent motor power cutoff. Read
> [Safety](#safety) before connecting a gimbal that can hit something.

Target distro: **ROS 2 Jazzy** (primary), **Lyrical Luth** (forward target).
Unofficial; not affiliated with BaseCam Electronics / AlexMos.

## Packages

| Package | What it is |
|---|---|
| `sbgc_protocol` | The vendored C protocol core as an ament library. No ROS dependency. |
| `sbgc_interfaces` | Messages, services and actions. Depends on nothing else here. |
| `sbgc_driver` | The lifecycle node, plus `GimbalCore` — every motion decision, ROS-free and separately tested. |
| `sbgc_description` | URDF/xacro and TF frames. |
| `sbgc_bringup` | Launch files and configuration. The only package that depends broadly. |

## Getting the source

The protocol library is a submodule, so a plain clone leaves it empty and the build stops
with an explanation.

```bash
git clone --recurse-submodules <this-repo>
# or, in an existing clone:
git submodule update --init --recursive
```

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
python3 vendor/simplebgc32-control/test/sbgc_sim.py   # prints a pty path
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

### A caveat on `sensor_msgs/Imu`

`orientation` comes from the board's own angles and is trustworthy. The raw
`angular_velocity` and `linear_acceleration` vectors carry the scaling and the sign
inversion the protocol specification documents, but **the mapping of the board's axis
order onto REP-103 body axes has not been confirmed against physical hardware**, so they
are published in the board's own IMU frame. Covariances are all-zero, which the message
defines as "unknown" — the board publishes no uncertainty and this driver has measured
none. Set `publish_imu: false` if you would rather not have the topic at all.

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
authoritative limits in the SimpleBGC GUI: this software cannot stop the board doing
something its own configuration allows.

Please do not describe any software interlock, watchdog, travel limit or stop command in
this repository as a safety guarantee.

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
no ROS includes and carries 30 unit tests asserting what reaches the wire.

**Threading.** `sbgc_t` has no internal locking. Every callback that touches the port is in
one mutually-exclusive callback group, which is what makes the node safe under a
multi-threaded executor. Do not add a port-touching callback outside it.

## Testing

```bash
make test
```

- Protocol: upstream's 37 byte-level assertions against BaseCam's published examples, run
  as a CTest so a submodule bump that changes the wire format fails this build.
- Decisions: 30 gtest cases on `GimbalCore`.
- Integration: 8 `launch_testing` cases driving the real node against the upstream board
  simulator on a pty.

## Licence

MIT.
