// Copyright 2026 Yousef Hussein
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Every decision the driver makes about what to put on the wire, with no ROS
// in it.
//
// This split is copied from the upstream project, which keeps its own
// decisions out of its I/O for the same reason: the interesting behaviour here
// is a set of safety rules, and safety rules that can only be exercised by
// standing up a node, a serial port and an executor do not get exercised.
//
// Units and signs at this boundary are ROS ones — radians, rad/s, REP-103
// right-handed with x forward, y left, z up, so positive pitch raises the
// camera. The board's own conventions (fixed-point counts, pitch positive
// downward) exist only on the far side of tick().

#ifndef SBGC_DRIVER__GIMBAL_CORE_HPP_
#define SBGC_DRIVER__GIMBAL_CORE_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <string>

// Vendored upstream header; it lives at the include root rather than in a
// subdirectory of its own, so the usual "name the directory" rule cannot apply.
#include "sbgc_api.h"  // NOLINT(build/include_subdir)

namespace sbgc_driver
{

inline constexpr int kNumAxes = SBGC_NUM_AXES;
inline constexpr int kRoll = SBGC_ROLL;
inline constexpr int kPitch = SBGC_PITCH;
inline constexpr int kYaw = SBGC_YAW;

using AxisArray = std::array<double, kNumAxes>;
using AxisFlags = std::array<bool, kNumAxes>;

enum class LimitsSource : uint8_t { Builtin = 0, Param = 1, Board = 2 };

enum class ControlMode : uint8_t { Idle = 0, Position = 1, Velocity = 2 };

struct Config
{
  // Indexed by kRoll / kPitch / kYaw throughout.
  AxisArray max_rate_rad_s{{0.52, 0.79, 0.79}};   // ~30, 45, 45 deg/s
  AxisFlags invert{{false, false, false}};

  AxisArray limit_min_rad{{-0.79, -1.57, -2.97}};
  AxisArray limit_max_rad{{0.79, 0.70, 2.97}};
  LimitsSource limits_source = LimitsSource::Builtin;
  bool enforce_limits = true;

  // Hold roll level rather than merely still. A zero *rate* in the board's
  // speed mode means "keep whatever angle you drifted to"; a zero *angle*
  // actively returns to the gravity horizon. On a mount where roll is not an
  // operator axis the second is what is wanted.
  bool roll_locked = true;

  double command_timeout_s = 0.5;

  // How old the last angle may be and still be treated as where the camera is.
  // Matched to command_timeout_s deliberately: if we have not heard the angle
  // for as long as it takes an operator's own command to expire, we do not
  // know where the camera is pointing and must not pretend otherwise.
  double angle_fresh_s = 0.5;

  double default_slew_rad_s = 0.52;
};

struct Telemetry
{
  double stamp = 0.0;
  // Wrapped to (-pi, pi] as the board reports it, already in ROS sign
  // convention. tick() judges limits against the continuous track built from
  // these, never against the wrapped value itself.
  AxisArray angle_rad{{0.0, 0.0, 0.0}};
  bool valid = false;
};

// Exactly the arguments sbgc_control_raw takes, so a test can assert what
// reaches the wire rather than an intermediate of this file's own invention.
struct WireControl
{
  std::array<uint8_t, kNumAxes> mode{};
  std::array<int16_t, kNumAxes> speed{};
  std::array<int16_t, kNumAxes> angle{};

  // This frame can cause motion. A hold frame is still transmitted — it is how
  // the board is told to stay put — so "sent something" is not the same as
  // "commanded movement", and the difference is what the arm gate is about.
  bool moving = false;
};

struct CoreStatus
{
  bool command_timeout = true;
  bool limits_stale = false;
  AxisFlags blocked_at_min{};
  AxisFlags blocked_at_max{};
  bool angle_valid = false;
  AxisArray continuous_rad{{0.0, 0.0, 0.0}};
  ControlMode mode = ControlMode::Idle;
  bool relative_to_frame = false;
};

// How the board's three sensor axes map onto the driver's own axes.
//
// The controller is a chip in a case, and which way that case is bolted into a
// mount is a property of the build rather than of the protocol. Two gimbals
// running identical firmware can present the same physical motion on different
// axes and with different signs, so this cannot be a constant in the source.
// Ceiling on |imu_axis_sign|. Orientation needs +-1 and a unit correction
// needs a small factor; anything larger is a mistake, and a merely-finite
// multiplier can still overflow once a 32767-count sample is scaled by it.
inline constexpr double kMaxAxisSign = 1.0e6;

struct AxisMapping
{
  // For each output axis, which board axis supplies it. Board axes are
  // 0 = ROLL, 1 = PITCH, 2 = YAW. Must be a permutation of {0, 1, 2}.
  std::array<int, kNumAxes> source{{0, 1, 2}};

  // Multiplier applied after the permutation, per output axis. Normally +1 or
  // -1; any non-zero value up to kMaxAxisSign is accepted, so a mount needing a
  // unit correction can be described without patching the driver.
  AxisArray sign{{1.0, 1.0, 1.0}};

  // True when `source` is a permutation of {0,1,2} and every sign is non-zero
  // with magnitude at most kMaxAxisSign. A mapping that is not valid must not
  // be applied: silently dropping or duplicating an axis would produce a
  // plausible-looking vector that is wrong, which is worse than refusing it.
  bool valid() const;

  // Apply to a raw board triple, scaling each component by `scale`. Falls back
  // to the board's own order when the mapping is not valid, so this is safe to
  // call unconditionally.
  AxisArray apply(const int16_t raw[kNumAxes], double scale) const;
};

class GimbalCore
{
public:
  GimbalCore(Config cfg, std::function<double()> now);

  void setConfig(const Config & cfg);
  const Config & config() const {return cfg_;}

  // The two halves of the gate. control_allowed mirrors a launch-time
  // parameter; armed is switched at runtime. Both are required.
  void setControlAllowed(bool allowed);
  void setArmed(bool armed);
  void setLinkUp(bool up);
  bool motionPermitted() const;

  // Command intake. Non-finite values are refused outright and leave the core
  // holding: a NaN that reaches the fixed-point conversion becomes an
  // arbitrary int16, which is a real rate.
  void submitRate(const AxisArray & rad_s);
  void submitAngle(const AxisArray & rad, bool relative_to_frame);
  void submitHold();

  void onTelemetry(const Telemetry & t);

  // What to send this cycle.
  WireControl tick();

  // A frame that commands no motion. Also what shutdown sends.
  WireControl holdFrame() const;

  const CoreStatus & status() const {return status_;}

  // Exposed for testing and for the node's own reporting; advances a
  // continuous track by the shortest step onto a newly wrapped reading.
  static double unwrapOnto(double continuous, double wrapped);

private:
  void clearCommand();
  bool commandFresh() const;

  Config cfg_;
  std::function<double()> now_;

  bool control_allowed_ = false;
  bool armed_ = false;
  bool link_up_ = false;

  ControlMode mode_ = ControlMode::Idle;
  bool relative_to_frame_ = false;
  AxisArray target_{{0.0, 0.0, 0.0}};
  double command_stamp_ = 0.0;
  bool have_command_ = false;

  Telemetry telemetry_{};
  bool have_continuous_ = false;
  AxisArray continuous_{{0.0, 0.0, 0.0}};

  CoreStatus status_{};
};

}  // namespace sbgc_driver

#endif  // SBGC_DRIVER__GIMBAL_CORE_HPP_
