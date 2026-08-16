# Hardware commissioning plan

First run of this driver against a physical gimbal. Written to be followed on its own,
with no prior conversation: everything needed is below.

Two parts, in order. **Part A** exercises the upstream C tool, which is the layer the ROS
driver sits on — if the protocol or the wiring is wrong, it is far easier to see there.
**Part B** commissions the ROS package.

---

## Read this first

> [!CAUTION]
> Every motion path in the ROS driver has been tested **only against a simulator**. Nothing
> in it has moved a real gimbal. Treat every step past B7 as a first flight.

**Have an independent way to cut motor power within reach, and use it as the abort.**
Not the stop service, not disarm, not unplugging the USB cable. The software gates are
risk reduction, not protection.

**The one failure mode the software cannot cover.** The watchdog stops motion only while it
can still *send* a frame. If the serial link dies while a continuous rate is active, the
board keeps turning at the last commanded rate, indefinitely — it has no serial-loss
failsafe of its own.

This plan **deliberately does not test that**, by decision, because provoking it is the one
step that can damage a mount. It therefore remains unverified on this rig. See
[Known limitation, deliberately not tested](#known-limitation-deliberately-not-tested)
before running anything unattended.

The documented way to reduce exposure to it is to prefer **bounded angle targets**
(`FollowJointTrajectory`, step B10) over **continuous rates** (`JointJog`, step B8): a
target the board is already slewing toward ends by itself, whereas a rate persists until
something cancels it.

**Travel limits in this driver are a convenience, not the protection.** The authoritative
limits live in the SimpleBGC GUI, on the board. This software cannot stop the board doing
something its own configuration permits. Confirm the board's own limits are sane *before*
step A7.

**Calibration requires a physically still gimbal.** Calibrating a moving one writes a wrong
zero-rate bias and the camera drifts in every session afterwards. The driver waits for
stillness, but do not fight it.

### Abort immediately — cut power — if

- anything moves that you did not command
- an axis runs toward a hard stop and does not slow
- `system_error` becomes non-zero, or `i2c_error_count` climbs steadily
- `motor_power` sits at or near 255 on any axis
- the camera oscillates or buzzes rather than settling

---

## The rig

| | |
|---|---|
| Board | SimpleBGC 3.1, firmware 2.63 b0 |
| Adapter | CH340 (`1a86:7523`) |
| Device | `/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0` → `/dev/ttyUSB0` |
| Port group | `uucp` (gid 985) — the user must be in it |
| ROS repo | `~/Projects/repos/simplebgc32-ros2` (`cmu-impactlab/simplebgc32_ROS2`) |
| Upstream C repo | `~/Projects/repos/patrolbot-gimbal` (`magdang/simplebgc32-control`) |
| Container | `simplebgc32-ros2-dev:jazzy`, built by `make image` |

There is no ROS installed on the host; everything ROS runs in the container. The
container needs the device and the port's group:

```bash
DOCKER="docker run --rm -it --device /dev/ttyUSB0 --group-add 985 \
  -v $PWD:/ws -w /ws --user $(id -u):$(id -g) -e HOME=/tmp --network=host \
  simplebgc32-ros2-dev:jazzy"
```

`/dev/serial/by-id/` does **not** exist inside the container — use `/dev/ttyUSB0` there.

### Already verified, so failures here mean something regressed

Monitor-mode telemetry, lifecycle transitions, diagnostics, the arming gate, board
identification, and the IMU scale and z-sign have all been confirmed against this board.
Everything else below is new ground.

---

## Part A — the upstream C tool

Run from `~/Projects/repos/patrolbot-gimbal`. No container; this builds natively.

### A1 · Simulated suite

```bash
make clean && make test
```

**Expect** five suites, `0 failures` in each: protocol 37, modules 103, page 15, CLI 23,
daemon 67. Do not continue if any fail.

### A2 · Read-only probe

The safest thing that can touch the board. It only asks questions.

```bash
make probe
./build/sbgc_probe --port /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0
```

**Expect** `BOARD_INFO` answering with board `0x1F` (3.1) and firmware `0x0A46` (2630 =
2.63 b0), and `REALTIME_DATA_3` returning 63 bytes. `CMD_ERROR` on the `EXT2`/`EXT3`
probes is normal — this firmware does not implement them.

**Record** the `REALTIME_DATA_3` hex dump. It is the reference for everything after.

### A3 · Identify without committing

```bash
./build/gimbal_gui --probe-port /dev/ttyUSB0 ; echo "exit=$?"
```

**Expect** `exit=0`. This sends only `CMD_BOARD_INFO`.

### A4 · CLI with no hardware

```bash
./build/gimbal_ctl --simulate --defaults
```

**Expect** frames printed as hex, nothing on the wire. Quit with `q`. Confirms the tool
works before it is pointed at metal.

### A5 · Browser console, monitor only

Note there is **no** `--allow-control`, so the UI cannot arm.

```bash
./build/gimbal_gui --port /dev/ttyUSB0 --no-pad
```

Open `http://127.0.0.1:8080`.

**Expect** live angles tracking the gimbal when you move it **by hand** (motors are off, the
stages turn freely), a battery reading, and the fault lamp showing "no faults". The arm
control should refuse.

**Record** whether the displayed angles match the physical position, and which way each
axis moves. This is where an inverted axis is cheapest to discover.

### A6 · Arm, motors still off

```bash
./build/gimbal_gui --port /dev/ttyUSB0 --no-pad --allow-control --no-calib-gyro
```

Arm in the UI. **Do not turn motors on yet.** Drive the pan/tilt controls.

**Expect** the "last command" hex readout to change, and the gimbal not to move — because
the motors are unpowered. Confirms commands reach the board before any of them can act.

### A7 · Motors on — first motion

> **Hand on the power cutoff from here on.**

Confirm the board's own travel limits in the SimpleBGC GUI first.

Turn motors on in the UI. The gimbal will stiffen and hold.

**Expect** it to settle within a second or two, not oscillate. Then the smallest pan input
you can give.

**Record** which physical direction each control moves the camera, and whether it matches
the label. Stop, disarm, motors off.

### A8 · Home and level

With motors on and armed, press Home, then Level.

**Expect** a smooth slew to the neutral pose, no hard-stop collision.

### A9 · Gyro calibration

Set the gimbal down. It must be **still**.

Press Calibrate in the UI.

**Expect** the UI to report settling, then calibrating, then done — roughly 5–6 seconds
total. Afterwards, with motors on and no input, the camera should hold position rather
than creeping.

**Record** whether drift is better or worse than before.

---

## Part B — the ROS 2 package

Run from `~/Projects/repos/simplebgc32-ros2`. Everything inside the container.

### B1 · Build from a clean clone

```bash
make clean && make test
```

**Expect** 5 packages, `123 tests, 0 errors, 0 failures, 8 skipped`.

### B2 · Monitor mode

The gate parameter `allow_control` defaults to false; the node will publish telemetry and
refuse to move anything.

```bash
$DOCKER bash -lc '. /opt/ros/jazzy/setup.sh && . install/setup.sh && \
  ros2 run sbgc_driver sbgc_driver_node --ros-args -p port:=/dev/ttyUSB0'
```

In a second shell into the same container:

```bash
ros2 lifecycle set /sbgc_driver configure
ros2 lifecycle set /sbgc_driver activate
ros2 topic echo /sbgc_driver/status --once
ros2 topic hz /joint_states
```

**Expect** `link_open: true`, `board_responding: true`, `armed: false`,
`control_allowed: false`, `system_error: 0`, `timeouts: 0`, and ~25 Hz on `/joint_states`.
`arm` and `set_motors` must both be **refused** with a message.

### B3 · Settle the IMU axis mapping — *the open question*

Motors off, so the stages turn freely by hand.

```bash
ros2 topic echo /sbgc_driver/imu --field linear_acceleration
```

Three observations, moving the gimbal **by hand**:

| Position | Expected |
|---|---|
| level and still | `z ≈ +9.81` |
| camera nose-down | `x` goes **negative** |
| camera rolled right-side-down | `y` goes **negative** |

Known so far on this board: level gives `x=-1.42, z=+9.84`. The z sign is right. The
component that tracks roll currently lands on `x`, where a roll about x should move
gravity onto `y` — so **`x` and `y` are probably swapped**.

Fix by parameter, not by editing code:

```yaml
# in sbgc_driver/config/sbgc_driver.yaml
imu_axis_map:  [1, 0, 2]        # try this if nose-down moves y instead of x
imu_axis_sign: [1.0, 1.0, 1.0]  # flip an entry if an axis moves the wrong way
```

`imu_axis_map` reads *"which board axis feeds ROS x, y, z"*; board axes are 0=ROLL,
1=PITCH, 2=YAW. It must be a permutation of 0,1,2 — anything else is refused in the log.

**Record the three readings and the final values.** This is the last unresolved item in
the driver.

### B4 · TF

```bash
ros2 launch sbgc_bringup gimbal.launch.py port:=/dev/ttyUSB0 auto_start:=true
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link gimbal_camera_optical_frame
```

**Expect** the chain `base_link → gimbal_base_link → yaw → roll → pitch → camera_link →
camera_optical_frame`, plus `gimbal_imu_link`. Tilt the camera down by hand and confirm the
optical frame's **z** component goes negative — the model must tilt the same way as the
real gimbal, not the opposite way.

### B5 · Control frames with motors off

```bash
ros2 launch sbgc_bringup gimbal.launch.py port:=/dev/ttyUSB0 \
  allow_control:=true auto_start:=true
ros2 service call /sbgc_driver/arm std_srvs/srv/SetBool "{data: true}"
```

**Motors stay off.** Publish a slow jog:

```bash
ros2 topic pub -r 20 /sbgc_driver/joint_jog control_msgs/msg/JointJog \
  '{joint_names: [gimbal_yaw_joint], velocities: [0.1]}'
```

**Expect** `command_timeout: false` in `~/status` while publishing, and nothing moves.

### B6 · The watchdog

Stop the `topic pub` with Ctrl-C.

**Expect** `command_timeout` to return to `true` within ~0.5 s. This is the fail-closed
guarantee; if it does not fire, stop and investigate before powering motors.

### B7 · Stop while disarmed

```bash
ros2 service call /sbgc_driver/arm std_srvs/srv/SetBool "{data: false}"
ros2 service call /sbgc_driver/stop std_srvs/srv/Trigger
```

**Expect** `success: true`. Stop is deliberately not gated — refusing to stop would invert
the point of the gate.

### B8 · Motors on, first ROS-commanded motion

> **Hand on the power cutoff.**

```bash
ros2 service call /sbgc_driver/set_motors std_srvs/srv/SetBool "{data: true}"
ros2 service call /sbgc_driver/arm std_srvs/srv/SetBool "{data: true}"
```

Smallest useful jog, one axis, briefly:

```bash
ros2 topic pub -r 20 /sbgc_driver/joint_jog control_msgs/msg/JointJog \
  '{joint_names: [gimbal_yaw_joint], velocities: [0.1]}'
```

**Expect** slow yaw motion in the direction of positive yaw (**left**, per REP-103). Then
repeat for `gimbal_pitch_joint` — **positive pitch must raise the camera**. If it dips, the
sign convention is wrong end-to-end and B4 should have caught it; stop and fix before
continuing.

**Record** direction and smoothness for each of the three joints.

### B9 · Travel limits

Jog slowly toward a configured limit.

**Expect** motion to stop at the limit, `blocked_at_max` (or `_min`) to become true for
that axis in `~/status`, and jogging **back toward the middle to still work**. Blocking
only one direction is the point.

### B10 · Trajectory

```bash
ros2 action send_goal /sbgc_driver/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [gimbal_yaw_joint],
    points: [{positions: [0.2], time_from_start: {sec: 3}}]}}' --feedback
```

**Expect** a slew to the target and `SUCCESSFUL`. Note this is a *reduced* implementation:
it commands the final point and lets the board slew, so intermediate waypoints are not
replayed. A goal carrying `path_tolerance` is rejected on purpose.

Then send a second goal and cancel it mid-slew — **expect** motion to stop and the goal to
report cancelled, not to keep running.

### B11 · Calibration

> The riskiest step, and the one with the most fixes behind it and the least hardware
> evidence. Gimbal **still**, on a flat surface, hand on the cutoff.

```bash
ros2 action send_goal /sbgc_driver/calibrate_gyro \
  sbgc_interfaces/action/CalibrateGyro \
  '{settle_timeout: {sec: 20}, skip_settle_check: false}' --feedback
```

**Expect** feedback showing `phase: 0` (settling) then `phase: 1` (calibrating), then
`result_code: 0` after roughly 5–6 s.

**The specific thing to watch:** the driver must send **no control frames** during the
calibration window — any one of them cancels the calibration on the board while the action
still reports success. Three separate defects of this kind were fixed; none has been
confirmed on hardware. If drift is *worse* afterwards, the calibration was cancelled and
reported OK, and that is the finding.

Also try: start a calibration, then call `stop` mid-way. **Expect**
`result_code: 4` (`RESULT_INTERRUPTED`), not `0`.

---

## Known limitation, deliberately not tested

**Serial loss during a continuous rate.**

If the link drops while a `JointJog` rate is in flight, the gimbal keeps turning at that
rate until something cuts power. The driver notices within about half a second, reports
`link_open: false`, raises an ERROR on `/diagnostics`, and retries every two seconds — but
it cannot stop a board it cannot reach, and the board does not stop itself.

Testing this means starting a motion and pulling the cable. That was **excluded from this
plan on purpose**: it is the one step whose failure mode is a mount driving into a hard
stop under power. The behaviour is inferred from the protocol and from upstream's own
documentation of it, not observed on this rig.

What follows from that:

- **This driver is not cleared for unattended operation** until an independent motor power
  cutoff is physically wired in, and until someone has decided whether it is fast enough.
  No software change can substitute; the driver is the thing that has become unreachable.
- Prefer `FollowJointTrajectory` over `JointJog` for anything autonomous. A bounded angle
  target ends on its own; a rate does not.
- If you later want the evidence, run it with **no payload fitted**, at the slowest rate
  the mount will accept, with the stages clear and a hand on the cutoff — and measure how
  far it travels before the cutoff stops it. That number is the actual safety margin.

Everything else in this plan was either exercised, or is listed above as not run.

## What to bring back

For each step: pass / fail / not run, plus anything surprising. The four that matter most:

1. **B3** — the three IMU readings and the final `imu_axis_map` / `imu_axis_sign`.
2. **B8** — which way each of the three joints actually moved.
3. **B11** — whether drift improved after calibration.
4. Whether an independent power cutoff is wired in yet, since the serial-loss case above
   is the reason it matters.

Anything that moved when it should not have, with what was running at the time.
