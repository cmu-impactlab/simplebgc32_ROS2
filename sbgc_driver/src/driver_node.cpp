// Copyright 2026 Yousef Hussein
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "sbgc_driver/driver_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace sbgc_driver
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;

// Calibration timings, taken from the upstream daemon where they were arrived
// at against real hardware.
//
// kCalibSeconds: the 2.6x manual says gyro calibration "takes about 4 seconds";
// the extra half second stops the driver declaring success early.
// kCalibStillRad: the most any axis may move between two consecutive telemetry
// samples and still count as stationary. A couple of times the reading noise,
// not zero -- an actively stabilised gimbal is never numerically still, and
// demanding that it be would mean the calibration never runs.
// kCalibStillSeconds: a settling gimbal passes through zero movement on its way
// past, so a single quiet sample proves nothing.
constexpr double kCalibSeconds = 4.5;
constexpr double kCalibStillRad = 0.20 * kPi / 180.0;
constexpr double kCalibStillSeconds = 1.0;
constexpr double kCalibSettleTimeout = 20.0;

double degToRad(double d) {return d * kPi / 180.0;}
double radToDeg(double r) {return r * 180.0 / kPi;}

builtin_interfaces::msg::Duration toDuration(double seconds)
{
  return rclcpp::Duration::from_seconds(std::max(0.0, seconds));
}

// Wrapped board angle, in the driver's own sign convention. The board reports
// pitch positive downward; everything above this line is positive up.
AxisArray rosAnglesFrom(const sbgc_realtime_t & rt)
{
  return AxisArray{{
    degToRad(rt.imu_deg[SBGC_ROLL]),
    degToRad(-rt.imu_deg[SBGC_PITCH]),
    degToRad(rt.imu_deg[SBGC_YAW])}};
}

}  // namespace

SbgcDriverNode::SbgcDriverNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sbgc_driver", options),
  board_info_stamp_(0, 0, RCL_ROS_TIME)
{
  param_listener_ = std::make_unique<sbgc_driver::ParamListener>(
    get_node_parameters_interface());
  params_ = param_listener_->get_params();
}

double SbgcDriverNode::nowSeconds() const
{
  return static_cast<double>(now().nanoseconds()) * 1e-9;
}

Config SbgcDriverNode::buildCoreConfig() const
{
  Config c;
  for (int a = 0; a < kNumAxes; ++a) {
    c.max_rate_rad_s[a] = degToRad(params_.max_rate_deg_s[static_cast<size_t>(a)]);
    c.limit_min_rad[a] = degToRad(params_.limit_min_deg[static_cast<size_t>(a)]);
    c.limit_max_rad[a] = degToRad(params_.limit_max_deg[static_cast<size_t>(a)]);
    c.invert[a] = params_.invert[static_cast<size_t>(a)];
  }
  c.limits_source = LimitsSource::Param;
  c.enforce_limits = params_.enforce_limits;
  c.roll_locked = (params_.roll_mode == "locked");
  c.command_timeout_s = params_.command_timeout;
  c.angle_fresh_s = params_.angle_fresh_timeout;
  c.default_slew_rad_s = degToRad(params_.default_slew_deg_s);
  return c;
}

void SbgcDriverNode::applyParameters()
{
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    if (core_) {core_->setConfig(buildCoreConfig());}
  }
}

// ---------------------------------------------------------------- lifecycle

CallbackReturn SbgcDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();

  joint_names_ = {
    params_.joint_prefix + "roll_joint",
    params_.joint_prefix + "pitch_joint",
    params_.joint_prefix + "yaw_joint"};

  core_ = std::make_unique<GimbalCore>(
    buildCoreConfig(), [this]() {return nowSeconds();});
  core_->setControlAllowed(params_.allow_control);
  core_->setArmed(false);

  port_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (!tryConnect()) {
    RCLCPP_ERROR(
      get_logger(), "could not open the gimbal: %s", fault_.c_str());
    return CallbackReturn::FAILURE;
  }

  // Ask who we are talking to before anything else. A board that never answers
  // is reported here rather than after something has been commanded.
  link_.send(SBGC_CMD_BOARD_INFO);
  link_.poll(200);
  if (!link_.haveBoardInfo() && !link_.simulated()) {
    RCLCPP_WARN(
      get_logger(),
      "opened %s but the board has not identified itself yet",
      resolved_port_.c_str());
  } else if (link_.haveBoardInfo()) {
    const auto & bi = link_.boardInfo();
    board_info_stamp_ = now();
    RCLCPP_INFO(
      get_logger(), "board %u.%u firmware %u.%u b%u on %s",
      bi.board_ver_major, bi.board_ver_minor,
      bi.firmware_major, bi.firmware_minor, bi.firmware_beta,
      resolved_port_.c_str());
  }

  const auto qos = rclcpp::SensorDataQoS();
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/imu", qos);
  orientation_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
    "~/mount_orientation", qos);
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("~/battery", 10);
  status_pub_ = create_publisher<sbgc_interfaces::msg::GimbalStatus>("~/status", 10);
  board_info_pub_ = create_publisher<sbgc_interfaces::msg::BoardInfo>(
    "~/board_info", rclcpp::QoS(1).transient_local());

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = port_group_;
  jog_sub_ = create_subscription<control_msgs::msg::JointJog>(
    "~/joint_jog", 10,
    std::bind(&SbgcDriverNode::onJointJog, this, std::placeholders::_1), sub_opts);

  auto bind2 = [this](auto fn) {
      return std::bind(fn, this, std::placeholders::_1, std::placeholders::_2);
    };

  set_motors_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_motors", bind2(&SbgcDriverNode::srvSetMotors),
    rclcpp::ServicesQoS(), port_group_);
  arm_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/arm", bind2(&SbgcDriverNode::srvArm),
    rclcpp::ServicesQoS(), port_group_);
  lock_mode_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/set_lock_mode", bind2(&SbgcDriverNode::srvSetLockMode),
    rclcpp::ServicesQoS(), port_group_);
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/stop", bind2(&SbgcDriverNode::srvStop),
    rclcpp::ServicesQoS(), port_group_);
  home_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/home", bind2(&SbgcDriverNode::srvHome),
    rclcpp::ServicesQoS(), port_group_);
  level_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/level", bind2(&SbgcDriverNode::srvLevel),
    rclcpp::ServicesQoS(), port_group_);
  control_mode_srv_ = create_service<sbgc_interfaces::srv::SetControlMode>(
    "~/set_control_mode", bind2(&SbgcDriverNode::srvSetControlMode),
    rclcpp::ServicesQoS(), port_group_);
  board_info_srv_ = create_service<sbgc_interfaces::srv::GetBoardInfo>(
    "~/get_board_info", bind2(&SbgcDriverNode::srvGetBoardInfo),
    rclcpp::ServicesQoS(), port_group_);

  calib_server_ = rclcpp_action::create_server<Calibrate>(
    this, "~/calibrate_gyro",
    std::bind(&SbgcDriverNode::calibGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SbgcDriverNode::calibCancel, this, std::placeholders::_1),
    std::bind(&SbgcDriverNode::calibAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(), port_group_);

  traj_server_ = rclcpp_action::create_server<Trajectory>(
    this, "~/follow_joint_trajectory",
    std::bind(&SbgcDriverNode::trajGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SbgcDriverNode::trajCancel, this, std::placeholders::_1),
    std::bind(&SbgcDriverNode::trajAccepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(), port_group_);

  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("simplebgc32");
  diagnostics_->add("gimbal", this, &SbgcDriverNode::diagnose);

  RCLCPP_INFO(
    get_logger(), "configured on %s; control is %s",
    resolved_port_.c_str(),
    params_.allow_control ? "permitted once armed" : "DISABLED (allow_control is false)");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SbgcDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  core_->setArmed(params_.start_armed);

  if (params_.auto_motors_on) {
    if (link_.send(SBGC_CMD_MOTORS_ON)) {motors_on_ = true;}
  }

  const auto period = [](double hz) {
      return std::chrono::duration<double>(1.0 / hz);
    };

  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period(params_.control_rate_hz)),
    std::bind(&SbgcDriverNode::controlTick, this), port_group_);
  telemetry_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period(params_.telemetry_rate_hz)),
    std::bind(&SbgcDriverNode::telemetryTick, this), port_group_);
  board_info_timer_ = create_wall_timer(
    std::chrono::seconds(2),
    std::bind(&SbgcDriverNode::boardInfoTick, this), port_group_);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period(params_.status_rate_hz)),
    std::bind(&SbgcDriverNode::statusTick, this), port_group_);

  RCLCPP_INFO(get_logger(), "active; holding until a valid command arrives");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SbgcDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // Stop commanding before anything else. This transition is the node's
  // "stop moving", and it must not depend on a service being reachable.
  core_->setArmed(false);
  if (link_.isOpen()) {link_.sendControl(core_->holdFrame());}

  // Before the timers go: they are the only thing advancing the action state
  // machines, so a goal left in flight here would never be resolved and its
  // client would wait for an answer that could not arrive.
  abortActiveGoals("the driver was deactivated");

  control_timer_.reset();
  telemetry_timer_.reset();
  board_info_timer_.reset();
  status_timer_.reset();

  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(get_logger(), "deactivated; a hold frame was sent");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SbgcDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  link_.close();
  core_.reset();
  diagnostics_.reset();

  joint_pub_.reset();
  imu_pub_.reset();
  orientation_pub_.reset();
  battery_pub_.reset();
  status_pub_.reset();
  board_info_pub_.reset();
  jog_sub_.reset();
  set_motors_srv_.reset();
  arm_srv_.reset();
  lock_mode_srv_.reset();
  stop_srv_.reset();
  home_srv_.reset();
  level_srv_.reset();
  control_mode_srv_.reset();
  board_info_srv_.reset();
  abortActiveGoals("the driver was cleaned up");
  calib_server_.reset();
  traj_server_.reset();

  board_info_published_ = false;
  motors_on_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn SbgcDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  if (core_ && link_.isOpen()) {
    core_->setArmed(false);
    link_.sendControl(core_->holdFrame());
  }
  return on_cleanup(state);
}

CallbackReturn SbgcDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "error state: %s", fault_.c_str());
  return on_cleanup(state);
}

// --------------------------------------------------------------------- link

bool SbgcDriverNode::tryConnect()
{
  if (params_.simulate) {
    link_.openSimulated();
    resolved_port_ = "(simulated)";
    core_->setLinkUp(true);
    fault_.clear();
    return true;
  }

  resolved_port_ = SbgcLink::discoverPort(params_.port);
  if (resolved_port_.empty()) {
    fault_ = "no serial port configured and none discovered";
    core_->setLinkUp(false);
    return false;
  }

  std::string error;
  if (!link_.open(resolved_port_, static_cast<int>(params_.baud_rate), &error)) {
    fault_ = "cannot open " + resolved_port_ + ": " + error;
    core_->setLinkUp(false);
    return false;
  }

  core_->setLinkUp(true);
  fault_.clear();
  return true;
}

void SbgcDriverNode::noteLinkLost(const std::string & why)
{
  fault_ = why;
  ++timeouts_;
  link_.close();
  core_->setLinkUp(false);
  next_reconnect_ = nowSeconds() + params_.reconnect_delay;
  RCLCPP_WARN(get_logger(), "gimbal link lost: %s", why.c_str());
}

// ------------------------------------------------------------------- timers

void SbgcDriverNode::controlTick()
{
  applyParameters();

  if (!link_.isOpen()) {
    if (nowSeconds() >= next_reconnect_) {
      if (tryConnect()) {
        RCLCPP_INFO(get_logger(), "gimbal reconnected on %s", resolved_port_.c_str());
      } else {
        next_reconnect_ = nowSeconds() + params_.reconnect_delay;
      }
    }
    return;
  }

  const WireControl w = core_->tick();
  if (!link_.sendControl(w)) {
    noteLinkLost(link_.lastError());
    return;
  }

  if (link_.poll(0) < 0) {
    noteLinkLost(link_.lastError());
    return;
  }

  if (link_.takeRealtimeFresh()) {
    telemetry_stamp_ = nowSeconds();
    Telemetry t;
    t.stamp = telemetry_stamp_;
    t.angle_rad = rosAnglesFrom(link_.realtime());
    t.valid = true;
    core_->onTelemetry(t);
    motors_on_ = link_.realtime().motors_on != 0;
    publishTelemetry();
  }

  calibTick();
  trajTick();
}

void SbgcDriverNode::telemetryTick()
{
  if (!link_.isOpen()) {return;}
  if (!link_.send(SBGC_CMD_REALTIME_DATA_3)) {noteLinkLost(link_.lastError());}
}

void SbgcDriverNode::boardInfoTick()
{
  if (!link_.isOpen()) {return;}
  if (!link_.send(SBGC_CMD_BOARD_INFO)) {
    noteLinkLost(link_.lastError());
    return;
  }
  if (link_.haveBoardInfo()) {
    board_info_stamp_ = now();
    publishBoardInfo();
  }
}

// -------------------------------------------------------------- publishing

void SbgcDriverNode::publishTelemetry()
{
  const auto & rt = link_.realtime();
  const auto stamp = now();
  const AxisArray angles = rosAnglesFrom(rt);
  const AxisArray continuous = core_->status().continuous_rad;

  sensor_msgs::msg::JointState js;
  js.header.stamp = stamp;
  js.name = joint_names_;
  // The continuous track, not the folded angle: a consumer building TF from
  // this must not see the camera jump a full turn when yaw crosses 180.
  js.position = {continuous[kRoll], continuous[kPitch], continuous[kYaw]};
  js.velocity = {
    degToRad(rt.target_deg[SBGC_ROLL]),
    degToRad(-rt.target_deg[SBGC_PITCH]),
    degToRad(rt.target_deg[SBGC_YAW])};
  // effort is left empty rather than filled with motor_power: that is a raw
  // 0-255 drive level, not a torque, and JointState effort is newton-metres.
  joint_pub_->publish(js);

  const double cr = std::cos(angles[kRoll] * 0.5), sr = std::sin(angles[kRoll] * 0.5);
  const double cp = std::cos(angles[kPitch] * 0.5), sp = std::sin(angles[kPitch] * 0.5);
  const double cy = std::cos(angles[kYaw] * 0.5), sy = std::sin(angles[kYaw] * 0.5);

  geometry_msgs::msg::QuaternionStamped q;
  q.header.stamp = stamp;
  q.header.frame_id = params_.frame_id;
  q.quaternion.w = cr * cp * cy + sr * sp * sy;
  q.quaternion.x = sr * cp * cy - cr * sp * sy;
  q.quaternion.y = cr * sp * cy + sr * cp * sy;
  q.quaternion.z = cr * cp * sy - sr * sp * cy;
  orientation_pub_->publish(q);

  if (params_.publish_imu) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = params_.imu_frame_id;
    imu.orientation = q.quaternion;

    // The specification gives ACC_DATA as 1/512 G "expressed in END coordinate
    // system, sign is inverted", and GYRO_DATA as 0.06103701895 deg/s. The
    // scaling and the documented sign inversion are applied; the mapping of
    // the board's axis order onto REP-103 body axes has NOT been confirmed
    // against a physical board, so these vectors are published in the board's
    // own IMU frame and the README says so.
    imu.linear_acceleration.x = -rt.acc_raw[SBGC_ROLL] * SBGC_ACC_UNIT_G * kG;
    imu.linear_acceleration.y = -rt.acc_raw[SBGC_PITCH] * SBGC_ACC_UNIT_G * kG;
    imu.linear_acceleration.z = -rt.acc_raw[SBGC_YAW] * SBGC_ACC_UNIT_G * kG;

    imu.angular_velocity.x = degToRad(rt.gyro_raw[SBGC_ROLL] * SBGC_GYRO_UNIT_DEGS);
    imu.angular_velocity.y = degToRad(rt.gyro_raw[SBGC_PITCH] * SBGC_GYRO_UNIT_DEGS);
    imu.angular_velocity.z = degToRad(rt.gyro_raw[SBGC_YAW] * SBGC_GYRO_UNIT_DEGS);

    // All-zero means "covariance unknown", which is the honest answer: the
    // board publishes no uncertainty and this driver has measured none.
    imu.orientation_covariance.fill(0.0);
    imu.angular_velocity_covariance.fill(0.0);
    imu.linear_acceleration_covariance.fill(0.0);
    imu_pub_->publish(imu);
  }

  sensor_msgs::msg::BatteryState bat;
  bat.header.stamp = stamp;
  bat.voltage = static_cast<float>(rt.battery_volts);
  bat.present = rt.battery_volts > 0.5;
  bat.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  bat.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  bat.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  // NaN, not zero: the message defines NaN as "not measured", and zero would
  // read as a flat battery.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  bat.current = nan;
  bat.charge = nan;
  bat.capacity = nan;
  bat.design_capacity = nan;
  bat.percentage = nan;
  bat.temperature = nan;
  battery_pub_->publish(bat);
}

void SbgcDriverNode::publishBoardInfo()
{
  if (!link_.haveBoardInfo()) {return;}
  const auto & bi = link_.boardInfo();

  sbgc_interfaces::msg::BoardInfo msg;
  msg.header.stamp = board_info_stamp_;
  msg.board_version_major = bi.board_ver_major;
  msg.board_version_minor = bi.board_ver_minor;
  msg.firmware_version_raw = bi.firmware_ver;
  msg.firmware_version_major = bi.firmware_major;
  msg.firmware_version_minor = bi.firmware_minor;
  msg.firmware_version_beta = bi.firmware_beta;
  msg.state_flags = bi.state_flags1;
  msg.board_features = bi.board_features;
  msg.connection_flag = bi.connection_flag;
  board_info_pub_->publish(msg);
  board_info_published_ = true;
}

void SbgcDriverNode::statusTick()
{
  const auto & cs = core_->status();
  const auto & rt = link_.realtime();
  const double age = link_.haveRealtime() ? (nowSeconds() - telemetry_stamp_) : 1e6;

  sbgc_interfaces::msg::GimbalStatus s;
  s.header.stamp = now();
  s.header.frame_id = params_.frame_id;

  s.link_open = link_.isOpen();
  s.board_responding = link_.haveRealtime() && age <= params_.angle_fresh_timeout;
  s.simulated = link_.simulated();
  s.last_frame_age = toDuration(age);
  s.frames_received = link_.framesReceived();
  s.timeouts = timeouts_;

  s.motors_on = motors_on_;
  s.control_allowed = params_.allow_control;
  s.armed = core_->motionPermitted() || (params_.allow_control && link_.isOpen());
  s.command_timeout = cs.command_timeout;

  s.limits_source = sbgc_interfaces::msg::GimbalStatus::LIMITS_PARAM;
  for (int a = 0; a < kNumAxes; ++a) {
    s.limit_min[static_cast<size_t>(a)] =
      static_cast<float>(core_->config().limit_min_rad[a]);
    s.limit_max[static_cast<size_t>(a)] =
      static_cast<float>(core_->config().limit_max_rad[a]);
    s.blocked_at_min[static_cast<size_t>(a)] = cs.blocked_at_min[a];
    s.blocked_at_max[static_cast<size_t>(a)] = cs.blocked_at_max[a];
  }
  s.limits_stale = cs.limits_stale;

  if (link_.haveRealtime()) {
    s.system_error = rt.system_error;
    const char * name = sbgc_system_error_name(rt.system_error);
    s.system_error_name = name ? name : "";
    s.system_sub_error = rt.system_sub_error;
    s.deprecated_error_code = rt.error_code;
    s.cycle_time = toDuration(rt.cycle_time_us * 1e-6);
    s.i2c_error_count = rt.i2c_error_count;
    s.serial_error_count = rt.serial_err_cnt;
    s.current_profile = rt.cur_profile;
    s.rc_signal_present = rt.rc_signal_present != 0;
    for (int a = 0; a < kNumAxes; ++a) {
      s.motor_power[static_cast<size_t>(a)] = rt.motor_power[a];
    }
  }

  // One source of truth: the same string goes to /diagnostics.
  if (!link_.isOpen()) {
    fault_ = fault_.empty() ? "gimbal not connected" : fault_;
  } else if (s.system_error != 0) {
    fault_ = s.system_error_name.empty() ? "board error" : s.system_error_name;
  } else if (!s.board_responding) {
    fault_ = "board is not answering";
  } else {
    fault_.clear();
  }
  s.fault = fault_;

  status_pub_->publish(s);

  if (!board_info_published_ && link_.haveBoardInfo()) {publishBoardInfo();}
  diagnostics_->force_update();
}

void SbgcDriverNode::diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto & cs = core_->status();
  const double age = link_.haveRealtime() ? (nowSeconds() - telemetry_stamp_) : 1e6;

  if (!link_.isOpen()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, fault_);
  } else if (link_.haveRealtime() && link_.realtime().system_error != 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, fault_);
  } else if (!link_.haveRealtime() || age > params_.angle_fresh_timeout) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "board is not answering");
  } else if (cs.limits_stale) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "holding: angle is stale");
  } else if (cs.command_timeout) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "holding (no active command)");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "active");
  }

  stat.add("port", resolved_port_);
  stat.add("simulated", link_.simulated());
  stat.add("frames_received", link_.framesReceived());
  stat.add("decode_errors", link_.decodeErrors());
  stat.add("timeouts", timeouts_);
  stat.add("last_frame_age_s", age);
  stat.add("motors_on", motors_on_);
  stat.add("control_allowed", params_.allow_control);
  stat.add("motion_permitted", core_->motionPermitted());
  stat.add("command_timeout", cs.command_timeout);
  stat.add("limits_stale", cs.limits_stale);
  if (link_.haveRealtime()) {
    const auto & rt = link_.realtime();
    stat.add("system_error", rt.system_error);
    const char * name = sbgc_system_error_name(rt.system_error);
    stat.add("system_error_name", name ? name : "");
    stat.add("i2c_errors", rt.i2c_error_count);
    stat.add("cycle_time_us", rt.cycle_time_us);
    stat.add("battery_volts", rt.battery_volts);
  }
}

// ----------------------------------------------------------------- commands

void SbgcDriverNode::onJointJog(const control_msgs::msg::JointJog::SharedPtr msg)
{
  // A jog with no velocities and no displacements is a request to stop, not a
  // malformed message.
  if (msg->velocities.empty() && msg->displacements.empty()) {
    core_->submitHold();
    return;
  }

  const bool use_velocity = !msg->velocities.empty();
  const auto & values = use_velocity ? msg->velocities : msg->displacements;

  AxisArray out{{0.0, 0.0, 0.0}};
  bool matched = false;

  if (msg->joint_names.empty()) {
    // No names: accept a bare triple in roll, pitch, yaw order, which is what
    // a simple teleop publishes.
    for (size_t i = 0; i < values.size() && i < static_cast<size_t>(kNumAxes); ++i) {
      out[i] = values[i];
      matched = true;
    }
  } else {
    for (size_t i = 0; i < msg->joint_names.size() && i < values.size(); ++i) {
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), msg->joint_names[i]);
      if (it == joint_names_.end()) {continue;}
      out[static_cast<size_t>(std::distance(joint_names_.begin(), it))] = values[i];
      matched = true;
    }
  }

  if (!matched) {
    // Naming only joints this gimbal does not have is not a reason to keep
    // moving on the ones it does.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "joint_jog named no joint this gimbal has; holding");
    core_->submitHold();
    return;
  }

  if (use_velocity) {
    core_->submitRate(out);
  } else {
    core_->submitAngle(out, relative_to_frame_);
  }
}

void SbgcDriverNode::srvSetMotors(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (!link_.isOpen()) {
    res->success = false;
    res->message = "gimbal not connected";
    return;
  }
  // Powering motors ON is motion-adjacent and needs the gate; powering them
  // OFF must always be allowed, for the same reason stop is.
  if (req->data && !params_.allow_control) {
    res->success = false;
    res->message = "allow_control is false; this node will not power the motors";
    return;
  }
  const uint8_t cmd = req->data ? SBGC_CMD_MOTORS_ON : SBGC_CMD_MOTORS_OFF;
  if (!link_.send(cmd)) {
    res->success = false;
    res->message = link_.lastError();
    return;
  }
  motors_on_ = req->data;
  res->success = true;
  res->message = req->data ? "motors on" : "motors off";
}

void SbgcDriverNode::srvArm(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data && !params_.allow_control) {
    res->success = false;
    res->message = "allow_control is false; arming would have no effect";
    return;
  }
  core_->setArmed(req->data);
  res->success = true;
  res->message = req->data ? "armed" : "disarmed";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void SbgcDriverNode::srvSetLockMode(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  // true = lock to the earth horizon, which is the board's plain angle mode.
  relative_to_frame_ = !req->data;
  res->success = true;
  res->message = req->data ? "locked to the horizon" : "following the frame";
}

void SbgcDriverNode::srvStop(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  // Deliberately not gated on arm. The gate exists to prevent unwanted motion,
  // and refusing to stop would invert its purpose.
  core_->submitHold();
  if (!link_.isOpen()) {
    res->success = false;
    res->message = "gimbal not connected; nothing could be sent";
    return;
  }
  res->success = link_.sendControl(core_->holdFrame());
  res->message = res->success ? "stopped" : link_.lastError();
}

void SbgcDriverNode::srvHome(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (!core_->motionPermitted()) {
    res->success = false;
    res->message = "motion not permitted; allow_control and arm are both required";
    return;
  }
  if (!link_.isOpen()) {
    res->success = false;
    res->message = "gimbal not connected";
    return;
  }
  res->success = link_.sendHome();
  res->message = res->success ? "homing" : link_.lastError();
}

void SbgcDriverNode::srvLevel(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (!core_->motionPermitted()) {
    res->success = false;
    res->message = "motion not permitted; allow_control and arm are both required";
    return;
  }
  if (!link_.isOpen()) {
    res->success = false;
    res->message = "gimbal not connected";
    return;
  }
  res->success = link_.sendLevel();
  res->message = res->success ? "levelling" : link_.lastError();
}

void SbgcDriverNode::srvSetControlMode(
  const sbgc_interfaces::srv::SetControlMode::Request::SharedPtr req,
  sbgc_interfaces::srv::SetControlMode::Response::SharedPtr res)
{
  using Req = sbgc_interfaces::srv::SetControlMode::Request;
  switch (req->mode) {
    case Req::MODE_IDLE:
      core_->submitHold();
      break;
    case Req::MODE_POSITION:
    case Req::MODE_VELOCITY:
      // The mode takes effect with the next command; selecting it does not
      // itself move anything.
      break;
    default:
      res->success = false;
      res->message = "unknown mode";
      res->mode = static_cast<uint8_t>(core_->status().mode);
      res->relative_to_frame = relative_to_frame_;
      return;
  }
  relative_to_frame_ = req->relative_to_frame;
  res->success = true;
  res->message = "ok";
  res->mode = req->mode;
  res->relative_to_frame = relative_to_frame_;
}

void SbgcDriverNode::srvGetBoardInfo(
  const sbgc_interfaces::srv::GetBoardInfo::Request::SharedPtr,
  sbgc_interfaces::srv::GetBoardInfo::Response::SharedPtr res)
{
  if (!link_.haveBoardInfo()) {
    res->available = false;
    res->message = "the board has not identified itself since this node configured";
    return;
  }
  const auto & bi = link_.boardInfo();
  res->available = true;
  res->message = "ok";
  res->info.header.stamp = board_info_stamp_;
  res->info.board_version_major = bi.board_ver_major;
  res->info.board_version_minor = bi.board_ver_minor;
  res->info.firmware_version_raw = bi.firmware_ver;
  res->info.firmware_version_major = bi.firmware_major;
  res->info.firmware_version_minor = bi.firmware_minor;
  res->info.firmware_version_beta = bi.firmware_beta;
  res->info.state_flags = bi.state_flags1;
  res->info.board_features = bi.board_features;
  res->info.connection_flag = bi.connection_flag;
}


// ------------------------------------------------------------------ actions

void SbgcDriverNode::abortActiveGoals(const std::string & why)
{
  if (calib_handle_) {
    // Reported as a link failure rather than a cancellation: nobody cancelled
    // it, and RESULT_OK would claim a calibration that did not finish.
    calibFinish(Calibrate::Result::RESULT_LINK_UNAVAILABLE, why);
  }
  calib_phase_ = CalibPhase::Idle;
  calib_handle_.reset();

  if (traj_handle_) {
    trajFinish(Trajectory::Result::INVALID_GOAL, why);
  }
  traj_handle_.reset();
}

rclcpp_action::GoalResponse SbgcDriverNode::calibGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Calibrate::Goal>)
{
  // Refused here rather than accepted and immediately failed, so a rejection
  // is a rejection in the protocol's own terms and never produces a Result.
  if (calib_phase_ != CalibPhase::Idle) {
    RCLCPP_WARN(get_logger(), "calibration already running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!core_->motionPermitted()) {
    RCLCPP_WARN(get_logger(), "calibration refused: motion is not permitted");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SbgcDriverNode::calibCancel(
  const std::shared_ptr<CalibrateGoal>)
{
  // Accepted in both phases, but honoured differently: while settling nothing
  // has been sent and the goal ends at once; once the board is calibrating,
  // calibTick waits out the window before reporting, because no frame this
  // project has verified confirms that a calibration was aborted.
  calib_cancel_requested_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SbgcDriverNode::calibAccepted(const std::shared_ptr<CalibrateGoal> handle)
{
  calib_handle_ = handle;
  calib_cancel_requested_ = false;
  calib_started_ = nowSeconds();
  calib_phase_started_ = calib_started_;
  calib_still_since_ = 0.0;
  calib_have_last_angle_ = false;
  calib_last_motion_ = 0.0;

  const double requested = rclcpp::Duration(handle->get_goal()->settle_timeout).seconds();
  calib_settle_timeout_ = requested > 0.0 ? requested : kCalibSettleTimeout;

  if (handle->get_goal()->skip_settle_check) {
    const uint8_t menu = SBGC_MENU_CALIB_GYRO;
    link_.send(SBGC_CMD_EXECUTE_MENU, &menu, 1);
    calib_phase_ = CalibPhase::Calibrating;
    calib_phase_started_ = nowSeconds();
    RCLCPP_WARN(get_logger(), "calibrating without waiting for the gimbal to settle");
  } else {
    calib_phase_ = CalibPhase::Settling;
  }
}

void SbgcDriverNode::calibFinish(uint8_t code, const std::string & message)
{
  if (!calib_handle_) {
    calib_phase_ = CalibPhase::Idle;
    return;
  }
  auto result = std::make_shared<Calibrate::Result>();
  result->result_code = code;
  result->message = message;
  result->elapsed = toDuration(nowSeconds() - calib_started_);

  if (code == Calibrate::Result::RESULT_CANCELLED) {
    calib_handle_->canceled(result);
  } else if (code == Calibrate::Result::RESULT_OK) {
    calib_handle_->succeed(result);
  } else {
    calib_handle_->abort(result);
  }

  RCLCPP_INFO(get_logger(), "calibration finished: %s", message.c_str());
  calib_handle_.reset();
  calib_phase_ = CalibPhase::Idle;
  calib_cancel_requested_ = false;
}

void SbgcDriverNode::calibTick()
{
  if (calib_phase_ == CalibPhase::Idle || !calib_handle_) {return;}

  const double t = nowSeconds();

  if (!link_.isOpen()) {
    calibFinish(Calibrate::Result::RESULT_LINK_UNAVAILABLE, "the serial link went away");
    return;
  }

  auto feedback = std::make_shared<Calibrate::Feedback>();
  feedback->elapsed = toDuration(t - calib_started_);
  feedback->motion = static_cast<float>(calib_last_motion_);

  if (calib_phase_ == CalibPhase::Settling) {
    if (calib_cancel_requested_) {
      calibFinish(Calibrate::Result::RESULT_CANCELLED, "cancelled while waiting to settle");
      return;
    }

    feedback->phase = Calibrate::Feedback::PHASE_WAITING_TO_SETTLE;
    calib_handle_->publish_feedback(feedback);

    if (!core_->status().angle_valid) {return;}

    const AxisArray angle = core_->status().continuous_rad;
    if (calib_have_last_angle_) {
      double worst = 0.0;
      for (int a = 0; a < kNumAxes; ++a) {
        worst = std::max(worst, std::abs(angle[a] - calib_last_angle_[a]));
      }
      calib_last_motion_ = worst;
      if (worst <= kCalibStillRad) {
        if (calib_still_since_ == 0.0) {calib_still_since_ = t;}
      } else {
        calib_still_since_ = 0.0;
      }
    }
    calib_last_angle_ = angle;
    calib_have_last_angle_ = true;

    if (calib_still_since_ != 0.0 && (t - calib_still_since_) >= kCalibStillSeconds) {
      const uint8_t menu = SBGC_MENU_CALIB_GYRO;
      if (!link_.send(SBGC_CMD_EXECUTE_MENU, &menu, 1)) {
        calibFinish(Calibrate::Result::RESULT_LINK_UNAVAILABLE, link_.lastError());
        return;
      }
      calib_phase_ = CalibPhase::Calibrating;
      calib_phase_started_ = t;
      RCLCPP_INFO(get_logger(), "gimbal is still; calibrating");
      return;
    }

    if ((t - calib_started_) >= calib_settle_timeout_) {
      // A gimbal that never settles must not be calibrated: the bias written
      // would be wrong and the camera would drift afterwards.
      calibFinish(
        Calibrate::Result::RESULT_NOT_STILL,
        "the gimbal did not stop moving; nothing was calibrated");
    }
    return;
  }

  feedback->phase = Calibrate::Feedback::PHASE_CALIBRATING;
  calib_handle_->publish_feedback(feedback);

  if ((t - calib_phase_started_) >= kCalibSeconds) {
    if (calib_cancel_requested_) {
      calibFinish(
        Calibrate::Result::RESULT_CANCELLED,
        "cancelled, but the calibration had already started and was allowed to finish");
    } else {
      calibFinish(Calibrate::Result::RESULT_OK, "calibrated");
    }
  }
}

// ---- follow_joint_trajectory ---------------------------------------------

rclcpp_action::GoalResponse SbgcDriverNode::trajGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Trajectory::Goal> goal)
{
  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(get_logger(), "trajectory goal has no points");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!core_->motionPermitted()) {
    RCLCPP_WARN(get_logger(), "trajectory refused: motion is not permitted");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (calib_phase_ != CalibPhase::Idle) {
    RCLCPP_WARN(get_logger(), "trajectory refused: a calibration is running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (const auto & name : goal->trajectory.joint_names) {
    if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
      RCLCPP_WARN(get_logger(), "trajectory names unknown joint '%s'", name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SbgcDriverNode::trajCancel(
  const std::shared_ptr<TrajectoryGoal>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SbgcDriverNode::trajAccepted(const std::shared_ptr<TrajectoryGoal> handle)
{
  traj_handle_ = handle;
  const auto & goal = *handle->get_goal();

  // Only the final point is driven to. The board runs its own trajectory
  // between here and there at its configured slew rate, so replaying the
  // intermediate points would fight it rather than help.
  traj_target_ = core_->status().continuous_rad;
  const auto & last = goal.trajectory.points.back();

  for (size_t i = 0; i < goal.trajectory.joint_names.size() && i < last.positions.size(); ++i) {
    const auto it = std::find(
      joint_names_.begin(), joint_names_.end(), goal.trajectory.joint_names[i]);
    const size_t a = static_cast<size_t>(std::distance(joint_names_.begin(), it));
    traj_target_[a] = last.positions[i];
  }

  // A tolerance of zero means "unspecified" in this action, not "exact".
  traj_tolerance_ = {{degToRad(1.0), degToRad(1.0), degToRad(1.0)}};
  for (const auto & tol : goal.goal_tolerance) {
    const auto it = std::find(joint_names_.begin(), joint_names_.end(), tol.name);
    if (it == joint_names_.end() || tol.position <= 0.0) {continue;}
    traj_tolerance_[static_cast<size_t>(std::distance(joint_names_.begin(), it))] = tol.position;
  }

  const double travel = rclcpp::Duration(last.time_from_start).seconds();
  const double slack = rclcpp::Duration(goal.goal_time_tolerance).seconds();
  // A goal with no timing at all still needs a deadline, or a gimbal that
  // cannot reach the target would leave the goal active forever.
  traj_deadline_ = nowSeconds() + (travel > 0.0 ? travel : 10.0) + (slack > 0.0 ? slack : 2.0);

  core_->submitAngle(traj_target_, relative_to_frame_);
}

void SbgcDriverNode::trajFinish(int32_t code, const std::string & message)
{
  if (!traj_handle_) {return;}
  auto result = std::make_shared<Trajectory::Result>();
  result->error_code = code;
  result->error_string = message;

  if (code == Trajectory::Result::SUCCESSFUL) {
    traj_handle_->succeed(result);
  } else if (traj_handle_->is_canceling()) {
    traj_handle_->canceled(result);
  } else {
    traj_handle_->abort(result);
  }
  traj_handle_.reset();
}

void SbgcDriverNode::trajTick()
{
  if (!traj_handle_) {return;}

  if (traj_handle_->is_canceling()) {
    core_->submitHold();
    trajFinish(Trajectory::Result::SUCCESSFUL, "cancelled");
    return;
  }
  if (!link_.isOpen()) {
    trajFinish(Trajectory::Result::INVALID_GOAL, "the serial link went away");
    return;
  }

  // Re-submitted every cycle. The command watchdog is deliberately short, and
  // a goal that stopped feeding it would be held mid-slew by the very
  // mechanism that protects against a dead publisher.
  core_->submitAngle(traj_target_, relative_to_frame_);

  const AxisArray at = core_->status().continuous_rad;
  auto feedback = std::make_shared<Trajectory::Feedback>();
  feedback->header.stamp = now();
  feedback->joint_names = joint_names_;
  feedback->desired.positions = {traj_target_[0], traj_target_[1], traj_target_[2]};
  feedback->actual.positions = {at[0], at[1], at[2]};
  feedback->error.positions = {
    traj_target_[0] - at[0], traj_target_[1] - at[1], traj_target_[2] - at[2]};
  traj_handle_->publish_feedback(feedback);

  if (core_->status().angle_valid) {
    bool arrived = true;
    for (int a = 0; a < kNumAxes; ++a) {
      if (std::abs(traj_target_[a] - at[a]) > traj_tolerance_[static_cast<size_t>(a)]) {
        arrived = false;
      }
    }
    if (arrived) {
      trajFinish(Trajectory::Result::SUCCESSFUL, "reached the goal");
      return;
    }
  }

  if (nowSeconds() > traj_deadline_) {
    core_->submitHold();
    trajFinish(
      Trajectory::Result::GOAL_TOLERANCE_VIOLATED,
      "the gimbal did not reach the goal in time");
  }
}

}  // namespace sbgc_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sbgc_driver::SbgcDriverNode)
