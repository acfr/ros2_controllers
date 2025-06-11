/*
* Copyright (c) 2020, Exobotic
* Copyright (c) 2017, Irstea
* Copyright (c) 2013, PAL Robotics, S.L.
* Copyright (c) 2023, Gabriel Urbain
* Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
* Copyright (c) 2023, Patrick Ven der Velde
* Copyright (c) 2023, Australian Centre For Robotics
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Author: Jerome Justin
*/

#ifndef SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_
#define SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_

#define _USE_MATH_DEFINES
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2/transform_datatypes.h"

#include "angles/angles.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_controllers_interfaces/msg/swerve_controller_status.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "swerve_controller/odometry.hpp"
#include "swerve_controller/speed_limiter.hpp"
#include "swerve_controller/visibility_control.h"
#include "swerve_controller/swerve_controller_parameters.hpp"

namespace swerve_controller
{
static constexpr size_t STATE_DRIVE_FL_WHEEL = 0;
static constexpr size_t STATE_DRIVE_FR_WHEEL = 1;
static constexpr size_t STATE_DRIVE_RL_WHEEL = 2;
static constexpr size_t STATE_DRIVE_RR_WHEEL = 3;
static constexpr size_t STATE_STEER_FL_WHEEL = 4;
static constexpr size_t STATE_STEER_FR_WHEEL = 5;
static constexpr size_t STATE_STEER_RL_WHEEL = 6;
static constexpr size_t STATE_STEER_RR_WHEEL = 7;

static constexpr size_t CMD_DRIVE_FL_WHEEL = 0;
static constexpr size_t CMD_DRIVE_FR_WHEEL = 1;
static constexpr size_t CMD_DRIVE_RL_WHEEL = 2;
static constexpr size_t CMD_DRIVE_RR_WHEEL = 3;
static constexpr size_t CMD_STEER_FL_WHEEL = 4;
static constexpr size_t CMD_STEER_FR_WHEEL = 5;
static constexpr size_t CMD_STEER_RL_WHEEL = 6;
static constexpr size_t CMD_STEER_RR_WHEEL = 7;

static constexpr size_t NR_STATE_ITFS = 8;
static constexpr size_t NR_CMD_ITFS = 8;
static constexpr size_t NR_REF_ITFS = 3;

class SwerveController : public controller_interface::ChainableControllerInterface
{
public:
  SWERVE_CONTROLLER_PUBLIC
  SwerveController();

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn configure_odometry();

  SWERVE_CONTROLLER_PUBLIC
  bool update_odometry(const rclcpp::Duration & period);

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;

  SWERVE_CONTROLLER_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerTwistReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
  using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;
  using SwerveControllerState = ros2_controllers_interfaces::msg::SwerveControllerStatus;
  using IcrState = visualization_msgs::msg::Marker;

protected:
  controller_interface::CallbackReturn set_interface_numbers(
    size_t nr_state_itfs, size_t nr_cmd_itfs, size_t nr_ref_itfs);

  struct DriveModuleDesiredValues
  {
    double steering_angle;  // rad/s
    double drive_velocity;  // m/s
  };

  struct Point
  {
    double x;
    double y;
    double z;
  };

  struct Line
  {
    Point p1;
    Point p2;
  };

  std::vector<std::string> steer_joints_names_;
  std::vector<std::string> drive_joints_names_;
  
  std::vector<std::string> drive_joints_state_names_;
  std::vector<std::string> steer_joints_state_names_;

  // Parameters from ROS for diff_drive_controller
  std::shared_ptr<swerve_controller::ParamListener> param_listener_;
  swerve_controller::Params params_;

  struct WheelParams
  {
    double wheelbase = 0.0;  // w.r.t. the midpoint of the wheel width
    double wheel_track =
      0.0;  // distance between left and right side wheels w.r.t. the midpoint of the wheel
    double radius = 0.0;  // Assumed to be the same for all wheels
    double drive_to_steer_offset = 0.0f;
    double max_steering_angle = 0.0f;
    double min_steering_angle = 0.0f;
  } wheel_params_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool position_feedback = false;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerTwistReferenceMsg>::SharedPtr ref_subscriber_twist_ = nullptr;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ref_subscriber_unstamped_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerTwistReferenceMsg>> input_ref_;
  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.01);  // 0ms

  using ControllerStatePublisherOdom = realtime_tools::RealtimePublisher<ControllerStateMsgOdom>;
  using ControllerStatePublisherTf = realtime_tools::RealtimePublisher<ControllerStateMsgTf>;

  rclcpp::Publisher<ControllerStateMsgOdom>::SharedPtr odom_s_publisher_;
  rclcpp::Publisher<ControllerStateMsgTf>::SharedPtr tf_odom_s_publisher_;

  std::unique_ptr<ControllerStatePublisherOdom> rt_odom_state_publisher_;
  std::unique_ptr<ControllerStatePublisherTf> rt_tf_odom_state_publisher_;

  using IcrPublisher = realtime_tools::RealtimePublisher<IcrState>;
  rclcpp::Publisher<IcrState>::SharedPtr icr_s_publisher_;
  std::unique_ptr<IcrPublisher> icr_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  Odometry odometry_;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<SwerveControllerState>;
  rclcpp::Publisher<SwerveControllerState>::SharedPtr controller_s_publisher_;
  std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;

  // name constants for state interfaces
  size_t nr_state_itfs_;
  // name constants for command interfaces
  size_t nr_cmd_itfs_;
  // name constants for reference interfaces
  size_t nr_ref_itfs_;

  // store last velocity
  double linear_x_command_ = 0.0;
  double linear_y_command_ = 0.0;
  double angular_command_ = 0.0;

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  std::vector<double> drive_joints_values_;
  std::vector<double> steer_joints_values_;

  /// Speed limiters:
  SpeedLimiter limiter_linear_x_;
  SpeedLimiter limiter_linear_y_;
  SpeedLimiter limiter_angular_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<ControllerTwistReferenceMsg>> limited_velocity_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<ControllerTwistReferenceMsg>>
    realtime_limited_velocity_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<ControllerTwistReferenceMsg>::SharedPtr velocity_command_subscriber_ =
    nullptr;

  std::queue<ControllerTwistReferenceMsg> previous_commands_;  // last two commands

  float min_steering_angle_ = -M_PI;
  float max_steering_angle_ = M_PI;

  SWERVE_CONTROLLER_LOCAL bool reset();

  SWERVE_CONTROLLER_LOCAL void brake();

  SWERVE_CONTROLLER_LOCAL void update_command_interfaces(
    std::vector<double> & drive_values, std::vector<double> & steer_values);

  SWERVE_CONTROLLER_LOCAL bool check_joint_states_are_valid();

  SWERVE_CONTROLLER_LOCAL void check_steering_limits(
    std::vector<DriveModuleDesiredValues> & result);

  SWERVE_CONTROLLER_LOCAL bool is_close(float a, float b, float abs_tol, float rel_tol);

  SWERVE_CONTROLLER_LOCAL double normalise_angle(float angle);
  SWERVE_CONTROLLER_LOCAL double difference_between_angles(float a, float b);

  SWERVE_CONTROLLER_LOCAL void publish_icrs(std::vector<std::vector<double>> icr_list);
  SWERVE_CONTROLLER_LOCAL std::vector<Point> find_wheel_centre_coords();
  SWERVE_CONTROLLER_LOCAL void find_icrs(std::vector<double> angles);

  // callback for topic interface
  SWERVE_CONTROLLER_LOCAL void reference_callback(
    const std::shared_ptr<ControllerTwistReferenceMsg> msg);

  void reference_callback_unstamped(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
};
}  // namespace swerve_controller

#endif  // SWERVE_CONTROLLER__SWERVE_CONTROLLER_HPP_
