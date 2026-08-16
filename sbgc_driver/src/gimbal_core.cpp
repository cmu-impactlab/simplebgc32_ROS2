// Copyright 2026 Yousef Hussein
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "sbgc_driver/gimbal_core.hpp"

#include <algorithm>
#include <cmath>

namespace sbgc_driver
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double radToDeg(double r) {return r * 180.0 / kPi;}

bool allFinite(const AxisArray & v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// The board's pitch axis is positive downward; this driver is positive up.
// The conversion lives here, at the single point where a value crosses to the
// wire, so no other code has to remember which convention it is holding.
double boardPitchFromRos(double v) {return -v;}

}  // namespace

GimbalCore::GimbalCore(Config cfg, std::function<double()> now)
: cfg_(cfg), now_(std::move(now))
{
  status_.command_timeout = true;
}

void GimbalCore::setConfig(const Config & cfg)
{
  // Configuration is not trusted any more than a command is. A NaN rate
  // ceiling would let std::clamp pass anything straight through, and a NaN
  // slew reaches the fixed-point conversion the header warns about. Bad values
  // fall back to the defaults rather than being propagated.
  const Config defaults{};
  cfg_ = cfg;

  for (int a = 0; a < kNumAxes; ++a) {
    if (!std::isfinite(cfg_.max_rate_rad_s[a]) || cfg_.max_rate_rad_s[a] <= 0.0) {
      cfg_.max_rate_rad_s[a] = defaults.max_rate_rad_s[a];
    }
    if (!std::isfinite(cfg_.limit_min_rad[a]) || !std::isfinite(cfg_.limit_max_rad[a])) {
      cfg_.limit_min_rad[a] = defaults.limit_min_rad[a];
      cfg_.limit_max_rad[a] = defaults.limit_max_rad[a];
    }
    // A reversed range would block both directions at once and strand the
    // camera. Ordering it is more useful than refusing it, and the operator
    // sees the corrected values on the status topic.
    if (cfg_.limit_min_rad[a] > cfg_.limit_max_rad[a]) {
      std::swap(cfg_.limit_min_rad[a], cfg_.limit_max_rad[a]);
    }
  }

  if (!std::isfinite(cfg_.default_slew_rad_s) || cfg_.default_slew_rad_s <= 0.0) {
    cfg_.default_slew_rad_s = defaults.default_slew_rad_s;
  }
  if (!std::isfinite(cfg_.command_timeout_s) || cfg_.command_timeout_s <= 0.0) {
    cfg_.command_timeout_s = defaults.command_timeout_s;
  }
  if (!std::isfinite(cfg_.angle_fresh_s) || cfg_.angle_fresh_s <= 0.0) {
    cfg_.angle_fresh_s = defaults.angle_fresh_s;
  }
}

void GimbalCore::setControlAllowed(bool allowed)
{
  control_allowed_ = allowed;
  if (!allowed) {clearCommand();}
}

void GimbalCore::setArmed(bool armed)
{
  armed_ = armed;
  // Disarming drops the pending command rather than merely refusing to send
  // it. Otherwise re-arming would resume a rate the operator issued before
  // they disarmed, which is not what "arm" means to anyone.
  if (!armed) {clearCommand();}
}

void GimbalCore::setLinkUp(bool up)
{
  link_up_ = up;
  if (!up) {
    // The angle we hold is from before the link dropped. Keeping it would let
    // limits be judged against a reading that is now arbitrarily old.
    telemetry_.valid = false;
    have_continuous_ = false;
    status_.angle_valid = false;
    // The command goes too. A link that drops and recovers inside the command
    // timeout would otherwise resume whatever was being commanded before the
    // drop, without the operator having said anything since.
    clearCommand();
  }
}

bool GimbalCore::motionPermitted() const
{
  return control_allowed_ && armed_ && link_up_;
}

void GimbalCore::clearCommand()
{
  have_command_ = false;
  mode_ = ControlMode::Idle;
  target_ = {{0.0, 0.0, 0.0}};
}

bool GimbalCore::commandFresh() const
{
  if (!have_command_) {return false;}
  return (now_() - command_stamp_) <= cfg_.command_timeout_s;
}

void GimbalCore::submitRate(const AxisArray & rad_s)
{
  if (!allFinite(rad_s)) {submitHold(); return;}

  // A command that arrives while a gate is shut is dropped, not stored. Merely
  // refusing to send it would leave it queued, and arming within the command
  // timeout would then start the gimbal moving on an instruction given before
  // the operator armed.
  if (!motionPermitted()) {submitHold(); return;}

  for (int a = 0; a < kNumAxes; ++a) {
    double v = cfg_.invert[a] ? -rad_s[a] : rad_s[a];
    const double lim = std::abs(cfg_.max_rate_rad_s[a]);
    target_[a] = std::clamp(v, -lim, lim);
  }
  mode_ = ControlMode::Velocity;
  command_stamp_ = now_();
  have_command_ = true;
}

void GimbalCore::submitAngle(const AxisArray & rad, bool relative_to_frame)
{
  if (!allFinite(rad)) {submitHold(); return;}
  if (!motionPermitted()) {submitHold(); return;}

  // Stored unclamped; tick() clamps against whatever limits are in force when
  // the frame is actually built. Clamping only here would emit a now-illegal
  // target if the limits were tightened after the command arrived.
  for (int a = 0; a < kNumAxes; ++a) {
    target_[a] = cfg_.invert[a] ? -rad[a] : rad[a];
  }
  mode_ = ControlMode::Position;
  relative_to_frame_ = relative_to_frame;
  command_stamp_ = now_();
  have_command_ = true;
}

void GimbalCore::submitHold()
{
  clearCommand();
  command_stamp_ = now_();
  have_command_ = true;
}

void GimbalCore::onTelemetry(const Telemetry & t)
{
  // A sample that is not usable must not refresh the timestamp. Storing it
  // would pair the previous angle with a fresh stamp, and the staleness gate
  // would then judge limits against an old reading it believes is current --
  // failing open in exactly the case the gate exists for.
  if (!t.valid || !allFinite(t.angle_rad) || !std::isfinite(t.stamp)) {
    telemetry_.valid = false;
    have_continuous_ = false;
    status_.angle_valid = false;
    return;
  }

  telemetry_ = t;

  if (!have_continuous_) {
    continuous_ = t.angle_rad;
    have_continuous_ = true;
  } else {
    for (int a = 0; a < kNumAxes; ++a) {
      continuous_[a] = unwrapOnto(continuous_[a], t.angle_rad[a]);
    }
  }
  status_.angle_valid = true;
  status_.continuous_rad = continuous_;
}

double GimbalCore::unwrapOnto(double continuous, double wrapped)
{
  // Advance the track by the shortest step onto the new reading. Using the
  // board's raw count instead would also be continuous, but would not agree
  // with the angle the limits were set against.
  double d = wrapped - std::fmod(continuous, 2.0 * kPi);
  while (d > kPi) {d -= 2.0 * kPi;}
  while (d <= -kPi) {d += 2.0 * kPi;}
  return continuous + d;
}

WireControl GimbalCore::holdFrame() const
{
  WireControl w;
  for (int a = 0; a < kNumAxes; ++a) {
    // SPEED with rate zero, not NO_CONTROL. NO_CONTROL relinquishes the axis
    // and lets whatever was commanded before persist, which is the opposite of
    // holding still.
    w.mode[a] = SBGC_MODE_SPEED;
    w.speed[a] = 0;
    w.angle[a] = 0;
  }
  // The roll lock is deliberately NOT applied here, even when configured.
  //
  // Locking roll means commanding MODE_ANGLE 0 at the slew rate, which makes an
  // off-level gimbal turn. That is wanted while an operator is driving the
  // other axes, and wrong in a hold: this frame is what the watchdog, the
  // deactivate transition and the stop service all send, and a stop that still
  // commands an axis to move is not a stop. sbgc_stop() upstream zeroes every
  // axis for the same reason.
  w.moving = false;
  return w;
}

WireControl GimbalCore::tick()
{
  status_.blocked_at_min = {};
  status_.blocked_at_max = {};
  status_.limits_stale = false;
  status_.mode = mode_;
  status_.relative_to_frame = relative_to_frame_;

  const bool fresh = commandFresh();
  status_.command_timeout = !fresh;

  if (!motionPermitted() || !fresh || mode_ == ControlMode::Idle) {
    return holdFrame();
  }

  // Limits are judged against the continuous track. A board that has stopped
  // reporting leaves the last angle frozen and perfectly plausible, so nothing
  // ever reads as "at the limit" and the camera drives straight through.
  const bool angle_stale =
    !status_.angle_valid || (now_() - telemetry_.stamp) > cfg_.angle_fresh_s;

  if (cfg_.enforce_limits && angle_stale) {
    status_.limits_stale = true;
    return holdFrame();
  }

  WireControl w;
  AxisArray value = target_;

  if (cfg_.enforce_limits) {
    for (int a = 0; a < kNumAxes; ++a) {
      const double at = continuous_[a];
      if (at <= cfg_.limit_min_rad[a]) {
        status_.blocked_at_min[a] = true;
        if (mode_ == ControlMode::Velocity && value[a] < 0.0) {value[a] = 0.0;}
      }
      if (at >= cfg_.limit_max_rad[a]) {
        status_.blocked_at_max[a] = true;
        if (mode_ == ControlMode::Velocity && value[a] > 0.0) {value[a] = 0.0;}
      }
      // Position targets are clamped here rather than on arrival, so limits
      // tightened after a command still apply to it.
      //
      // In relative_to_frame this bounds the request but cannot guarantee the
      // result: the target is measured against the mount while the limits and
      // the telemetry behind them are measured against the horizon, and this
      // class is given no frame attitude with which to relate the two. A
      // vehicle on a slope can therefore reach an absolute angle outside these
      // limits from a target that is inside them. The board's own limits are
      // what actually protects the mount.
      if (mode_ == ControlMode::Position) {
        value[a] = std::clamp(value[a], cfg_.limit_min_rad[a], cfg_.limit_max_rad[a]);
      }
    }
  }

  if (mode_ == ControlMode::Velocity) {
    for (int a = 0; a < kNumAxes; ++a) {
      w.mode[a] = SBGC_MODE_SPEED;
      const double deg_s = radToDeg(a == kPitch ? boardPitchFromRos(value[a]) : value[a]);
      w.speed[a] = sbgc_degs_to_units(deg_s);
      w.angle[a] = 0;
    }
    if (cfg_.roll_locked) {
      w.mode[kRoll] = SBGC_MODE_ANGLE;
      w.speed[kRoll] = sbgc_degs_to_units(radToDeg(cfg_.default_slew_rad_s));
      w.angle[kRoll] = 0;
    }
  } else {
    const uint8_t m = relative_to_frame_ ? SBGC_MODE_ANGLE_REL_FRAME : SBGC_MODE_ANGLE;
    for (int a = 0; a < kNumAxes; ++a) {
      w.mode[a] = m;
      const double deg = radToDeg(a == kPitch ? boardPitchFromRos(value[a]) : value[a]);
      w.speed[a] = sbgc_degs_to_units(radToDeg(cfg_.default_slew_rad_s));
      w.angle[a] = sbgc_deg_to_units(deg);
    }
    if (cfg_.roll_locked) {
      w.mode[kRoll] = SBGC_MODE_ANGLE;
      w.angle[kRoll] = 0;
    }
  }

  // Derived from the frame that is actually going out, not from the intent
  // behind it. Computing it earlier got this wrong three ways: a roll-only
  // rate read as moving even though the roll lock discards it, a rate below
  // half a wire LSB read as moving after converting to zero, and a roll-locked
  // frame with every rate at zero read as still even though it commands roll
  // back to level.
  for (int a = 0; a < kNumAxes; ++a) {
    const uint8_t m = w.mode[a];
    if (m == SBGC_MODE_SPEED) {
      if (w.speed[a] != 0) {w.moving = true;}
    } else {
      // Any angle mode names a target. Whether the axis actually turns depends
      // on where it currently is, which makes this frame capable of motion.
      w.moving = true;
    }
  }

  return w;
}

}  // namespace sbgc_driver
