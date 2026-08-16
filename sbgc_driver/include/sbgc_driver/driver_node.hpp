// Copyright 2026 Yousef Hussein
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// The ROS half of the driver: parameters, timers, topics, services and
// reconnection. Deliberately thin — every decision about what reaches the wire
// is in GimbalCore, and every byte is in the vendored protocol library.
//
// Lifecycle, not a plain node, because the transitions map onto the states
// this hardware actually has. Nothing can move before on_configure has opened
// a port and identified the board, and on_deactivate is a first-class "stop
// moving" that does not depend on a bespoke service being reachable.
//
// THREADING: the link is not internally locked. Every callback that touches it
// is placed in one mutually-exclusive callback group, so the node is safe
// under a multi-threaded executor as well as a single-threaded one. Do not add
// a port-touching callback outside that group.

#ifndef SBGC_DRIVER__DRIVER_NODE_HPP_
#define SBGC_DRIVER__DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sbgc_driver/gimbal_core.hpp"
#include "sbgc_driver/sbgc_link.hpp"
#include "sbgc_interfaces/msg/board_info.hpp"
#include "sbgc_interfaces/msg/gimbal_status.hpp"
#include "sbgc_interfaces/srv/get_board_info.hpp"
#include "sbgc_interfaces/srv/set_control_mode.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "control_msgs/msg/joint_jog.hpp"

#include "sbgc_driver_parameters.hpp"

namespace sbgc_driver
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SbgcDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SbgcDriverNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  // ---- wiring ----
  Config buildCoreConfig() const;
  void applyParameters();
  double nowSeconds() const;

  // ---- timers ----
  void controlTick();
  void telemetryTick();
  void boardInfoTick();
  void statusTick();

  // ---- callbacks ----
  void onJointJog(const control_msgs::msg::JointJog::SharedPtr msg);

  void srvSetMotors(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  void srvArm(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  void srvSetLockMode(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  void srvStop(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void srvHome(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void srvLevel(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void srvSetControlMode(
    const sbgc_interfaces::srv::SetControlMode::Request::SharedPtr req,
    sbgc_interfaces::srv::SetControlMode::Response::SharedPtr res);
  void srvGetBoardInfo(
    const sbgc_interfaces::srv::GetBoardInfo::Request::SharedPtr req,
    sbgc_interfaces::srv::GetBoardInfo::Response::SharedPtr res);

  // ---- publishing ----
  void publishTelemetry();
  void publishBoardInfo();
  void diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // ---- link ----
  bool tryConnect();
  void noteLinkLost(const std::string & why);

  std::unique_ptr<sbgc_driver::ParamListener> param_listener_;
  sbgc_driver::Params params_;

  std::unique_ptr<GimbalCore> core_;
  SbgcLink link_;

  rclcpp::CallbackGroup::SharedPtr port_group_;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
    orientation_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbgc_interfaces::msg::GimbalStatus>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sbgc_interfaces::msg::BoardInfo>::SharedPtr board_info_pub_;

  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr jog_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_motors_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr lock_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr level_srv_;
  rclcpp::Service<sbgc_interfaces::srv::SetControlMode>::SharedPtr control_mode_srv_;
  rclcpp::Service<sbgc_interfaces::srv::GetBoardInfo>::SharedPtr board_info_srv_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr board_info_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;

  std::vector<std::string> joint_names_;
  std::string resolved_port_;

  bool motors_on_ = false;
  bool relative_to_frame_ = false;
  bool board_info_published_ = false;

  double telemetry_stamp_ = 0.0;
  rclcpp::Time board_info_stamp_;
  double next_reconnect_ = 0.0;
  uint32_t timeouts_ = 0;
  std::string fault_;
};

}  // namespace sbgc_driver

#endif  // SBGC_DRIVER__DRIVER_NODE_HPP_
