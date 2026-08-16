// Copyright 2026 Yousef Hussein
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Tests for the safety rules, not for the arithmetic.
//
// Each of these corresponds to a way the upstream project found a gimbal could
// be made to move when it should not have. They assert what reaches the wire,
// because that is the only thing the board acts on.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "sbgc_driver/gimbal_core.hpp"

using sbgc_driver::AxisArray;
using sbgc_driver::Config;
using sbgc_driver::ControlMode;
using sbgc_driver::GimbalCore;
using sbgc_driver::Telemetry;
using sbgc_driver::kPitch;
using sbgc_driver::kRoll;
using sbgc_driver::kYaw;

namespace
{

constexpr double kPi = 3.14159265358979323846;

double deg(double d) {return d * kPi / 180.0;}

class Fixture
{
public:
  double t = 100.0;
  Config cfg;
  GimbalCore core;

  Fixture()
  : cfg(makeConfig()), core(makeConfig(), [this]() {return t;})
  {
    core.setControlAllowed(true);
    core.setArmed(true);
    core.setLinkUp(true);
    feedAngle(0.0, 0.0, 0.0);
  }

  static Config makeConfig()
  {
    Config c;
    c.max_rate_rad_s = {{deg(30), deg(45), deg(45)}};
    c.limit_min_rad = {{deg(-45), deg(-90), deg(-170)}};
    c.limit_max_rad = {{deg(45), deg(40), deg(170)}};
    c.roll_locked = false;      // most tests want plain rate behaviour
    return c;
  }

  void feedAngle(double roll, double pitch, double yaw)
  {
    Telemetry tel;
    tel.stamp = t;
    tel.angle_rad = {{roll, pitch, yaw}};
    tel.valid = true;
    core.onTelemetry(tel);
  }

  void rate(double roll, double pitch, double yaw)
  {
    core.submitRate({{roll, pitch, yaw}});
  }
};

}  // namespace

// ---- the watchdog --------------------------------------------------------

TEST(Watchdog, HoldsOnceTheCommandGoesStale)
{
  Fixture f;
  f.rate(0.0, 0.0, deg(20));
  EXPECT_TRUE(f.core.tick().moving);

  // Just inside the window: still moving.
  f.t += 0.4;
  f.feedAngle(0.0, 0.0, 0.0);
  EXPECT_TRUE(f.core.tick().moving);
  EXPECT_FALSE(f.core.status().command_timeout);

  // Past it: held, and the yaw rate is actually zero on the wire rather than
  // merely flagged.
  f.t += 0.2;
  f.feedAngle(0.0, 0.0, 0.0);
  auto w = f.core.tick();
  EXPECT_FALSE(w.moving);
  EXPECT_EQ(w.speed[kYaw], 0);
  EXPECT_TRUE(f.core.status().command_timeout);
}

TEST(Watchdog, HoldFrameUsesSpeedZeroNotNoControl)
{
  // NO_CONTROL relinquishes the axis and lets the previous command persist,
  // which is the opposite of holding still.
  Fixture f;
  auto w = f.core.holdFrame();
  for (int a = 0; a < sbgc_driver::kNumAxes; ++a) {
    EXPECT_EQ(w.mode[a], SBGC_MODE_SPEED) << "axis " << a;
    EXPECT_EQ(w.speed[a], 0) << "axis " << a;
  }
}

TEST(Watchdog, HoldFrameCommandsNoMotionEvenWithRollLocked)
{
  // The roll lock commands MODE_ANGLE 0 at the slew rate, which turns an
  // off-level gimbal. That belongs in an active rate command and never in a
  // hold: this frame is what the watchdog, the deactivate transition and the
  // stop service all send, and a stop that still commands an axis to move is
  // not a stop.
  Fixture f;
  auto cfg = f.cfg;
  cfg.roll_locked = true;
  f.core.setConfig(cfg);

  auto w = f.core.holdFrame();
  for (int a = 0; a < sbgc_driver::kNumAxes; ++a) {
    EXPECT_EQ(w.speed[a], 0) << "axis " << a << " must carry no rate at all";
    EXPECT_EQ(w.angle[a], 0) << "axis " << a;
  }
  EXPECT_EQ(w.mode[kRoll], SBGC_MODE_SPEED);
  EXPECT_FALSE(w.moving);
}

// ---- the arm gate --------------------------------------------------------

TEST(ArmGate, BothKeysAreRequired)
{
  Fixture f;
  f.core.setArmed(false);
  f.rate(0.0, 0.0, deg(20));
  EXPECT_FALSE(f.core.tick().moving);

  f.core.setControlAllowed(false);
  f.core.setArmed(true);
  f.rate(0.0, 0.0, deg(20));
  EXPECT_FALSE(f.core.tick().moving);

  f.core.setControlAllowed(true);
  f.rate(0.0, 0.0, deg(20));
  EXPECT_TRUE(f.core.tick().moving);
}

TEST(ArmGate, DisarmingDropsThePendingCommand)
{
  // Re-arming must not resume a rate issued before the operator disarmed.
  Fixture f;
  f.rate(0.0, 0.0, deg(20));
  f.core.setArmed(false);
  f.core.setArmed(true);
  EXPECT_FALSE(f.core.tick().moving);
}

TEST(ArmGate, ALostLinkStopsMotionEvenWhileArmed)
{
  Fixture f;
  f.rate(0.0, 0.0, deg(20));
  f.core.setLinkUp(false);
  EXPECT_FALSE(f.core.motionPermitted());
  EXPECT_FALSE(f.core.tick().moving);
}

// ---- the staleness gate --------------------------------------------------

TEST(Staleness, LimitedMotionIsHeldWhenTheAngleIsOld)
{
  // A board that stops reporting leaves the last angle frozen and plausible.
  // Nothing ever reads as "at the limit" and the camera drives through.
  Fixture f;
  f.rate(0.0, 0.0, deg(20));
  EXPECT_TRUE(f.core.tick().moving);

  f.t += 0.6;                 // telemetry not refreshed
  f.rate(0.0, 0.0, deg(20));  // command is fresh; the angle is not
  auto w = f.core.tick();
  EXPECT_FALSE(w.moving);
  EXPECT_TRUE(f.core.status().limits_stale);
}

TEST(Staleness, TurningLimitsOffRestoresMotion)
{
  // The documented way to recover a camera by hand.
  Fixture f;
  f.t += 0.6;
  f.rate(0.0, 0.0, deg(20));
  EXPECT_FALSE(f.core.tick().moving);

  auto cfg = f.cfg;
  cfg.enforce_limits = false;
  f.core.setConfig(cfg);
  f.rate(0.0, 0.0, deg(20));
  auto w = f.core.tick();
  EXPECT_TRUE(w.moving);
  EXPECT_FALSE(f.core.status().limits_stale);
}

// ---- travel limits -------------------------------------------------------

TEST(Limits, BlockOnlyTheDirectionThatWouldLeaveTheRange)
{
  Fixture f;
  f.feedAngle(0.0, 0.0, deg(170));   // at the yaw maximum

  f.rate(0.0, 0.0, deg(20));         // further out: refused
  auto out = f.core.tick();
  EXPECT_EQ(out.speed[kYaw], 0);
  EXPECT_TRUE(f.core.status().blocked_at_max[kYaw]);
  EXPECT_FALSE(f.core.status().blocked_at_min[kYaw]);

  f.rate(0.0, 0.0, deg(-20));        // back toward the middle: allowed
  auto back = f.core.tick();
  EXPECT_LT(back.speed[kYaw], 0);
  EXPECT_TRUE(back.moving);
}

TEST(Limits, AreJudgedAcrossTheWrapNotAgainstTheFoldedAngle)
{
  // Walking yaw past +180 makes the reported angle jump to -180. A limit
  // checked against the folded number would think the axis had teleported to
  // the far end of its travel.
  Fixture f;
  auto cfg = f.cfg;
  cfg.limit_min_rad = {{deg(-45), deg(-90), deg(-400)}};
  cfg.limit_max_rad = {{deg(45), deg(40), deg(400)}};
  f.core.setConfig(cfg);

  f.feedAngle(0.0, 0.0, deg(170));
  f.feedAngle(0.0, 0.0, deg(-170));   // crossed the fold, moved +20 deg

  EXPECT_NEAR(f.core.status().continuous_rad[kYaw], deg(190), 1e-6);
  f.rate(0.0, 0.0, deg(20));
  EXPECT_TRUE(f.core.tick().moving) << "a 400 deg limit must not block at 190";
}

TEST(Limits, AngleTargetsAreClampedIntoRange)
{
  Fixture f;
  f.core.submitAngle({{0.0, 0.0, deg(300)}}, false);
  auto w = f.core.tick();
  EXPECT_NEAR(sbgc_units_to_deg(w.angle[kYaw]), 170.0, 0.05);
}

// ---- board conventions ---------------------------------------------------

TEST(Conventions, PitchIsInvertedOnTheWayToTheBoard)
{
  // The board's pitch axis is positive downward and this driver is positive
  // up. With no conversion, "up" tilts the camera down and the asymmetric
  // travel limits then apply to the wrong direction.
  Fixture f;
  f.rate(0.0, deg(20), 0.0);
  auto w = f.core.tick();
  EXPECT_LT(w.speed[kPitch], 0) << "positive ROS pitch must be negative on the wire";
  EXPECT_NEAR(sbgc_units_to_degs(w.speed[kPitch]), -20.0, 0.2);
}

TEST(Conventions, YawAndRollKeepTheirSign)
{
  Fixture f;
  f.rate(0.0, 0.0, deg(20));
  EXPECT_NEAR(sbgc_units_to_degs(f.core.tick().speed[kYaw]), 20.0, 0.2);
}

TEST(Conventions, RollLockHoldsAnAngleRatherThanARate)
{
  // A zero roll *rate* holds whatever roll the gimbal drifted to. Only a zero
  // *angle* actively returns it to the horizon.
  Fixture f;
  auto cfg = f.cfg;
  cfg.roll_locked = true;
  f.core.setConfig(cfg);

  f.rate(0.0, 0.0, deg(20));
  auto w = f.core.tick();
  EXPECT_EQ(w.mode[kRoll], SBGC_MODE_ANGLE);
  EXPECT_EQ(w.angle[kRoll], 0);
  EXPECT_EQ(w.mode[kYaw], SBGC_MODE_SPEED);
}

TEST(Conventions, RelativeFrameSelectsTheOtherAngleMode)
{
  Fixture f;
  f.core.submitAngle({{0.0, 0.0, deg(10)}}, false);
  EXPECT_EQ(f.core.tick().mode[kYaw], SBGC_MODE_ANGLE);

  f.core.submitAngle({{0.0, 0.0, deg(10)}}, true);
  EXPECT_EQ(f.core.tick().mode[kYaw], SBGC_MODE_ANGLE_REL_FRAME);
}

// ---- rate limiting and bad input ----------------------------------------

TEST(Rates, AreClampedToTheConfiguredCeiling)
{
  Fixture f;
  f.rate(0.0, 0.0, deg(500));
  EXPECT_NEAR(sbgc_units_to_degs(f.core.tick().speed[kYaw]), 45.0, 0.2);
}

TEST(Rates, InversionIsApplied)
{
  Fixture f;
  auto cfg = f.cfg;
  cfg.invert[kYaw] = true;
  f.core.setConfig(cfg);
  f.rate(0.0, 0.0, deg(20));
  EXPECT_NEAR(sbgc_units_to_degs(f.core.tick().speed[kYaw]), -20.0, 0.2);
}

TEST(Rates, NonFiniteInputIsRefusedRatherThanConverted)
{
  // NaN through the fixed-point conversion becomes an arbitrary int16, which
  // is a real rate the gimbal will happily turn at.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  Fixture f;
  f.rate(0.0, 0.0, nan);
  auto w = f.core.tick();
  EXPECT_FALSE(w.moving);
  EXPECT_EQ(w.speed[kYaw], 0);

  f.rate(0.0, inf, 0.0);
  EXPECT_FALSE(f.core.tick().moving);

  f.core.submitAngle({{0.0, 0.0, nan}}, false);
  EXPECT_FALSE(f.core.tick().moving);
}

// ---- unwrap --------------------------------------------------------------

TEST(Unwrap, AdvancesByTheShortestStep)
{
  EXPECT_NEAR(GimbalCore::unwrapOnto(deg(170), deg(-170)), deg(190), 1e-9);
  EXPECT_NEAR(GimbalCore::unwrapOnto(deg(-170), deg(170)), deg(-190), 1e-9);
  EXPECT_NEAR(GimbalCore::unwrapOnto(deg(190), deg(-170)), deg(190), 1e-9);
  EXPECT_NEAR(GimbalCore::unwrapOnto(0.0, deg(10)), deg(10), 1e-9);
}

TEST(Unwrap, SurvivesManyTurnsInOneDirection)
{
  double cont = 0.0;
  for (int i = 1; i <= 360 * 3; ++i) {
    const double wrapped = std::remainder(deg(i), 2.0 * kPi);
    cont = GimbalCore::unwrapOnto(cont, wrapped);
  }
  EXPECT_NEAR(cont, deg(1080), 1e-6);
}

// ---- gate ordering -------------------------------------------------------
//
// These cover the case where a command is accepted while a gate is shut and
// then becomes live when the gate opens, with nothing said in between.

TEST(GateOrdering, ACommandGivenWhileDisarmedDoesNotSurviveArming)
{
  Fixture f;
  f.core.setArmed(false);
  f.rate(0.0, 0.0, deg(20));     // shouted at a disarmed gimbal
  f.core.setArmed(true);         // ...and then armed, well inside the timeout
  EXPECT_FALSE(f.core.tick().moving);
}

TEST(GateOrdering, ACommandGivenWhileControlIsDisallowedDoesNotSurvive)
{
  Fixture f;
  f.core.setControlAllowed(false);
  f.rate(0.0, 0.0, deg(20));
  f.core.setControlAllowed(true);
  EXPECT_FALSE(f.core.tick().moving);
}

TEST(GateOrdering, ACommandDoesNotSurviveALinkDropAndRecovery)
{
  // A link that drops and comes back inside the command timeout must not
  // resume what was being commanded before it dropped.
  Fixture f;
  f.rate(0.0, 0.0, deg(20));
  f.core.setLinkUp(false);
  f.core.setLinkUp(true);
  f.feedAngle(0.0, 0.0, 0.0);
  EXPECT_FALSE(f.core.tick().moving);
}

// ---- telemetry validity --------------------------------------------------

TEST(Telemetry, AnInvalidSampleDoesNotRefreshTheAngle)
{
  // Pairing the previous angle with a new timestamp is worse than having no
  // angle: the staleness gate would believe the old reading is current.
  Fixture f;
  f.t += 1.0;
  Telemetry bad;
  bad.stamp = f.t;
  bad.angle_rad = {{0.0, 0.0, 0.0}};
  bad.valid = false;
  f.core.onTelemetry(bad);

  f.rate(0.0, 0.0, deg(20));
  EXPECT_FALSE(f.core.tick().moving);
  EXPECT_TRUE(f.core.status().limits_stale);
}

TEST(Telemetry, NonFiniteAnglesAndStampsAreRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  Fixture f;
  Telemetry bad;
  bad.stamp = f.t;
  bad.angle_rad = {{0.0, nan, 0.0}};
  bad.valid = true;
  f.core.onTelemetry(bad);
  f.rate(0.0, 0.0, deg(20));
  EXPECT_FALSE(f.core.tick().moving) << "a NaN angle must not be trusted";
  EXPECT_TRUE(f.core.status().limits_stale);

  Fixture g;
  Telemetry bad_stamp;
  bad_stamp.stamp = nan;
  bad_stamp.angle_rad = {{0.0, 0.0, 0.0}};
  bad_stamp.valid = true;
  g.core.onTelemetry(bad_stamp);
  g.rate(0.0, 0.0, deg(20));
  EXPECT_FALSE(g.core.tick().moving) << "a NaN stamp must not be trusted";
  EXPECT_TRUE(g.core.status().limits_stale);
}

// ---- limits applied at emission -----------------------------------------

TEST(Limits, TighteningThemAfterAnAngleCommandStillApplies)
{
  Fixture f;
  f.core.submitAngle({{0.0, 0.0, deg(160)}}, false);
  EXPECT_NEAR(sbgc_units_to_deg(f.core.tick().angle[kYaw]), 160.0, 0.05);

  auto cfg = f.cfg;
  cfg.limit_max_rad[kYaw] = deg(90);
  f.core.setConfig(cfg);
  EXPECT_NEAR(sbgc_units_to_deg(f.core.tick().angle[kYaw]), 90.0, 0.05)
    << "the target must be re-clamped when the limits change";
}

// ---- the moving flag describes the emitted frame -------------------------

TEST(MovingFlag, ARollOnlyRateIsNotMotionWhenRollIsLocked)
{
  Fixture f;
  auto cfg = f.cfg;
  cfg.roll_locked = true;
  f.core.setConfig(cfg);

  // The roll rate is discarded by the lock, but the lock itself commands roll
  // to level, so the frame genuinely can move something.
  f.rate(deg(20), 0.0, 0.0);
  auto w = f.core.tick();
  EXPECT_EQ(w.mode[kRoll], SBGC_MODE_ANGLE);
  EXPECT_TRUE(w.moving) << "the roll lock commands an angle, which can move";

  // With the lock off, a roll-only rate is ordinary motion.
  cfg.roll_locked = false;
  f.core.setConfig(cfg);
  f.rate(deg(20), 0.0, 0.0);
  EXPECT_TRUE(f.core.tick().moving);
}

TEST(MovingFlag, ARateTooSmallToEncodeIsNotReportedAsMotion)
{
  // Half a wire LSB is 0.06 deg/s. Anything under that converts to zero, and
  // a frame carrying zero is not motion however it was asked for.
  Fixture f;
  auto cfg = f.cfg;
  cfg.roll_locked = false;
  f.core.setConfig(cfg);

  f.rate(0.0, 0.0, deg(0.01));
  auto w = f.core.tick();
  EXPECT_EQ(w.speed[kYaw], 0);
  EXPECT_FALSE(w.moving);
}

// ---- configuration is not trusted ---------------------------------------

TEST(ConfigValidation, NonFiniteValuesFallBackToDefaults)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();

  Fixture f;
  auto cfg = f.cfg;
  cfg.max_rate_rad_s[kYaw] = nan;
  cfg.default_slew_rad_s = nan;
  f.core.setConfig(cfg);

  EXPECT_TRUE(std::isfinite(f.core.config().max_rate_rad_s[kYaw]));
  EXPECT_TRUE(std::isfinite(f.core.config().default_slew_rad_s));

  // And the frame that comes out is a real one, not a converted NaN.
  f.rate(0.0, 0.0, deg(20));
  auto w = f.core.tick();
  EXPECT_NE(w.speed[kYaw], 0);
}

TEST(ConfigValidation, AReversedLimitRangeIsOrderedRatherThanStrandingTheAxis)
{
  Fixture f;
  auto cfg = f.cfg;
  cfg.limit_min_rad[kYaw] = deg(90);
  cfg.limit_max_rad[kYaw] = deg(-90);
  f.core.setConfig(cfg);

  EXPECT_LT(f.core.config().limit_min_rad[kYaw], f.core.config().limit_max_rad[kYaw]);
  f.feedAngle(0.0, 0.0, 0.0);
  f.rate(0.0, 0.0, deg(20));
  EXPECT_TRUE(f.core.tick().moving) << "an axis in the middle of its range must move";
}

// ---- the IMU axis mapping ------------------------------------------------
//
// The controller is a chip in a case, and which way that case is bolted into a
// mount differs between builds. These cover the mapping being applied, and
// being refused when it would silently corrupt a vector.

TEST(AxisMapping, TheIdentityPassesTheBoardsOwnOrderThrough)
{
  const int16_t raw[3] = {10, 20, 30};
  sbgc_driver::AxisMapping m;              // defaults to identity, +1 signs
  EXPECT_TRUE(m.valid());

  const auto out = m.apply(raw, 1.0);
  EXPECT_DOUBLE_EQ(out[0], 10.0);
  EXPECT_DOUBLE_EQ(out[1], 20.0);
  EXPECT_DOUBLE_EQ(out[2], 30.0);
}

TEST(AxisMapping, PermutesAndSignsIndependently)
{
  const int16_t raw[3] = {10, 20, 30};
  sbgc_driver::AxisMapping m;
  m.source = {{2, 0, 1}};                  // x<-yaw, y<-roll, z<-pitch
  m.sign = {{1.0, -1.0, 1.0}};
  ASSERT_TRUE(m.valid());

  const auto out = m.apply(raw, 1.0);
  EXPECT_DOUBLE_EQ(out[0], 30.0);
  EXPECT_DOUBLE_EQ(out[1], -10.0);
  EXPECT_DOUBLE_EQ(out[2], 20.0);
}

TEST(AxisMapping, ScaleIsAppliedToEveryComponent)
{
  // This is how the raw counts become m/s^2 and rad/s.
  const int16_t raw[3] = {0, 0, 517};      // ~1 g on the vertical axis
  sbgc_driver::AxisMapping m;
  const auto out = m.apply(raw, (1.0 / 512.0) * 9.80665);
  EXPECT_NEAR(out[2], 9.90, 0.01);
}

TEST(AxisMapping, ADuplicatedSourceAxisIsRefused)
{
  // Two outputs fed from one board axis means a third is silently dropped, and
  // the result still looks like a plausible vector. Refusing beats guessing.
  sbgc_driver::AxisMapping m;
  m.source = {{0, 0, 2}};
  EXPECT_FALSE(m.valid());
}

TEST(AxisMapping, AnOutOfRangeSourceAxisIsRefused)
{
  sbgc_driver::AxisMapping m;
  m.source = {{0, 1, 3}};
  EXPECT_FALSE(m.valid());
  m.source = {{-1, 1, 2}};
  EXPECT_FALSE(m.valid());
}

TEST(AxisMapping, ANonFiniteOrZeroSignIsRefused)
{
  sbgc_driver::AxisMapping m;
  m.sign = {{1.0, std::numeric_limits<double>::quiet_NaN(), 1.0}};
  EXPECT_FALSE(m.valid());

  // Zero would erase an axis while still reporting a three-vector.
  m.sign = {{1.0, 0.0, 1.0}};
  EXPECT_FALSE(m.valid());
}
