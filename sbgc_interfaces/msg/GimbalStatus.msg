# Driver and board state that has no standard ROS equivalent.
#
# Everything expressible in a standard message is published as one instead:
# joint angles as sensor_msgs/JointState, the IMU as sensor_msgs/Imu, battery
# as sensor_msgs/BatteryState, health as diagnostic_msgs/DiagnosticArray. What
# is left is this: the state of the serial link, of the driver's own safety
# gates, and the board's fault word.

std_msgs/Header header

# ---- link ----------------------------------------------------------------

# The serial port is open. Says nothing about whether the board is answering:
# a board whose transmit path dies still holds the port open.
bool link_open

# A well-formed frame arrived recently. This is the one to trust.
bool board_responding

# No hardware is attached and nothing is being written to a wire.
bool simulated

# How long before header.stamp the most recent frame arrived. Expressed
# relative to this message's own stamp rather than as an absolute time, so it
# stays meaningful without the publisher and subscriber agreeing on a clock.
# Grows without bound while the board is silent, so it is usable as a staleness
# measure and not only as a flag.
builtin_interfaces/Duration last_frame_age

# Monotonic since the driver last entered the configured state, i.e. they reset
# on cleanup and on a re-configure, but NOT on a reconnect — a link that keeps
# dropping should show an accumulating timeout count rather than hide it. Both
# saturate at their maximum instead of wrapping, so a consumer differencing
# them never sees a negative jump.
uint32 frames_received
uint32 timeouts

# ---- gates ---------------------------------------------------------------

bool motors_on

# The driver's two-key gate. control_allowed is the process-level parameter and
# cannot change at runtime; armed is the runtime half. Motion requires both.
bool control_allowed
bool armed

# No valid command arrived within command_timeout, so the driver is holding.
# This is the fail-closed state, not an error: a publisher that dies stops the
# camera rather than leaving it running.
bool command_timeout

# ---- travel limits -------------------------------------------------------

uint8 LIMITS_BUILTIN=0    # the driver's own defaults
uint8 LIMITS_PARAM=1      # from this node's parameters
uint8 LIMITS_BOARD=2      # read from the live board, and preferred

uint8 limits_source

# Radians, in the order roll, pitch, yaw.
float32[3] limit_min
float32[3] limit_max

# Per-axis, which END of the travel is currently withholding motion. Split into
# two flags rather than one "blocked" boolean because a consumer recovering a
# camera by hand needs to know which direction is still available; a single
# flag says only that something is wrong. Both may be set at once if the
# configured range is narrower than the measurement noise.
bool[3] blocked_at_min
bool[3] blocked_at_max

# Limits are enabled but there is no fresh angle to judge them against, so
# motion is held rather than allowed through unchecked. Distinct from the
# blocked flags: "at the limit" and "cannot tell where the camera is" are
# different situations with different fixes.
bool limits_stale

# ---- board fault ---------------------------------------------------------

# SYSTEM_ERROR bitmask; 0 means no fault. This is the field a current board
# fills in.
uint16 system_error

# Short name of the lowest set bit of system_error, or empty when there is no
# fault. Faults cascade, and the first one is the one worth acting on.
string system_error_name

# Emergency-stop reason code accompanying system_error.
uint8 system_sub_error

# The board's older one-byte error field. Reported because an older firmware
# may be the only thing that sets it, but the board's own specification marks
# it deprecated in favour of system_error, and it is deliberately not named:
# that table has not been verified against hardware.
uint8 deprecated_error_code

# ---- board health --------------------------------------------------------

# The board's own control loop period, as it reports it.
builtin_interfaces/Duration cycle_time

# Counters maintained by the board, not by this driver, so they reset when the
# board does and not when the driver does. They wrap at 16 bits; a consumer
# differencing them must treat a decrease as a wrap or a board reset rather
# than as a negative rate.
uint16 i2c_error_count
uint16 serial_error_count

uint8 current_profile      # 0-based, as the board reports it
bool rc_signal_present

# Raw per-axis motor drive from the board, 0-255, in the order roll, pitch,
# yaw. Not a torque, not a current, and not convertible to either without a
# per-mount calibration this driver does not have — which is why it is passed
# through unscaled rather than published as JointState effort.
uint8[3] motor_power

# Empty when healthy; a short human-readable reason otherwise. This is the same
# string the driver puts in the DiagnosticStatus it publishes on /diagnostics,
# produced once and copied, so the two channels cannot disagree.
string fault
