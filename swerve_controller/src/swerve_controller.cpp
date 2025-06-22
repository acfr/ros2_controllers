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

#include "swerve_controller/swerve_controller.hpp"

namespace
{  // utility
using ControllerTwistReferenceMsg =
  swerve_controller::SwerveController::ControllerTwistReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerTwistReferenceMsg> & msg,
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg->header.stamp = node->now();
  msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

namespace swerve_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

SwerveController::SwerveController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn SwerveController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<swerve_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::set_interface_numbers(
  size_t nr_state_itfs = 4, size_t nr_cmd_itfs = 4, size_t nr_ref_itfs = 3)
{
  nr_state_itfs_ = nr_state_itfs;
  nr_cmd_itfs_ = nr_cmd_itfs;
  nr_ref_itfs_ = nr_ref_itfs;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  configure_odometry();

  if (!params_.steer_joints_names.empty())
  {
    steer_joints_names_ = params_.steer_joints_names;
  }
  else
  {
    RCLCPP_INFO(logger, "Steer Joints are not specified");
  }

  if (!params_.drive_joints_names.empty())
  {
    drive_joints_names_ = params_.drive_joints_names;
  }
  else
  {
    RCLCPP_INFO(logger, "Drive Joints are not specified");
  }

  if (!params_.drive_joints_state_names.empty())
  {
    drive_joints_state_names_ = params_.drive_joints_state_names;
  }
  else
  {
    RCLCPP_INFO(logger, "Drive Joints are not specified");
  }

  if (!params_.steer_joints_state_names.empty())
  {
    steer_joints_state_names_ = params_.steer_joints_state_names;
  }
  else
  {
    RCLCPP_INFO(logger, "Steer Joints are not specified");
  }

  cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};
  publish_limited_velocity_ = params_.publish_limited_velocity;

  limiter_linear_x_ = SpeedLimiter(
    params_.linear.x.has_velocity_limits, params_.linear.x.has_acceleration_limits,
    params_.linear.x.has_jerk_limits, params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.min_acceleration, params_.linear.x.max_acceleration, params_.linear.x.min_jerk,
    params_.linear.x.max_jerk);

  limiter_linear_y_ = SpeedLimiter(
    params_.linear.y.has_velocity_limits, params_.linear.y.has_acceleration_limits,
    params_.linear.y.has_jerk_limits, params_.linear.y.min_velocity, params_.linear.y.max_velocity,
    params_.linear.y.min_acceleration, params_.linear.y.max_acceleration, params_.linear.y.min_jerk,
    params_.linear.y.max_jerk);

  limiter_angular_ = SpeedLimiter(
    params_.angular.z.has_velocity_limits, params_.angular.z.has_acceleration_limits,
    params_.angular.z.has_jerk_limits, params_.angular.z.min_velocity,
    params_.angular.z.max_velocity, params_.angular.z.min_acceleration,
    params_.angular.z.max_acceleration, params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ = get_node()->create_publisher<ControllerTwistReferenceMsg>(
      "~/cmd_vel_limitted", rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<ControllerTwistReferenceMsg>>(
        limited_velocity_publisher_);
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);

  const ControllerTwistReferenceMsg empty_twist;
  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  if (params_.use_stamped_vel)
  {
    ref_subscriber_twist_ = get_node()->create_subscription<ControllerTwistReferenceMsg>(
      "~/reference", subscribers_qos,
      std::bind(&SwerveController::reference_callback, this, std::placeholders::_1));
  }
  else
  {
    ref_subscriber_unstamped_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "~/reference_unstamped", subscribers_qos,
      std::bind(&SwerveController::reference_callback_unstamped, this, std::placeholders::_1));
  }

  std::shared_ptr<ControllerTwistReferenceMsg> msg =
    std::make_shared<ControllerTwistReferenceMsg>();
  reset_controller_reference_msg(msg, get_node());
  input_ref_.writeFromNonRT(msg);

  try
  {
    // Odom state publisher
    odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgOdom>(
      "~/odometry", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<ControllerStatePublisherOdom>(odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
  rt_odom_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  rt_odom_state_publisher_->msg_.child_frame_id = params_.base_frame_id;
  rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;

  auto & covariance = rt_odom_state_publisher_->msg_.twist.covariance;
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    covariance[diagonal_index] = params_.pose_covariance_diagonal[index];
    covariance[diagonal_index] = params_.twist_covariance_diagonal[index];
  }
  rt_odom_state_publisher_->unlock();

  try
  {
    // Tf State publisher
    tf_odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgTf>(
      "~/tf_odometry", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ =
      std::make_unique<ControllerStatePublisherTf>(tf_odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = params_.odom_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = params_.base_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  rt_tf_odom_state_publisher_->unlock();

  try
  {
    // State publisher
    icr_s_publisher_ =
      get_node()->create_publisher<IcrState>("~/icrs", rclcpp::SystemDefaultsQoS());
    icr_publisher_ = std::make_unique<IcrPublisher>(icr_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  icr_publisher_->lock();
  icr_publisher_->msg_.header.stamp = get_node()->now();
  icr_publisher_->msg_.header.frame_id = "base_footprint";
  icr_publisher_->msg_.type = 8;    // list of points
  icr_publisher_->msg_.action = 0;  // add/modify
  icr_publisher_->unlock();

  try
  {
    // State publisher
    controller_s_publisher_ = get_node()->create_publisher<SwerveControllerState>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    controller_state_publisher_ =
      std::make_unique<ControllerStatePublisher>(controller_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  controller_state_publisher_->lock();
  controller_state_publisher_->msg_.header.stamp = get_node()->now();
  controller_state_publisher_->msg_.header.frame_id = params_.odom_frame_id;
  controller_state_publisher_->unlock();

  RCLCPP_INFO(logger, "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::configure_odometry()
{
  wheel_params_.radius = params_.wheel_radius;
  wheel_params_.wheelbase = params_.wheelbase;
  wheel_params_.wheel_track = params_.wheel_track;
  wheel_params_.drive_to_steer_offset = params_.drive_to_steer_offset;
  wheel_params_.max_steering_angle = params_.max_steering_limit;
  wheel_params_.min_steering_angle = params_.min_steering_limit;

  odometry_.set_wheel_params(
    wheel_params_.radius, wheel_params_.wheelbase, wheel_params_.wheel_track,
    wheel_params_.drive_to_steer_offset);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "Swerve Drive odometry configuration successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool SwerveController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(
      linear_x_command_, linear_y_command_, angular_command_, period.seconds());
  }
  else
  {
    if (check_joint_states_are_valid())
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          drive_joints_values_, steer_joints_values_, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(
          drive_joints_values_, steer_joints_values_, period.seconds());
      }
    }
  }
  return true;
}

void SwerveController::reference_callback(const std::shared_ptr<ControllerTwistReferenceMsg> msg)
{
  // if no timestamp provided use current time for command timestamp
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
    msg->header.stamp = get_node()->now();
  }
  const auto age_of_last_command = get_node()->now() - msg->header.stamp;

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received message has timestamp %.10f older for %.10f which is more then allowed timeout "
      "(%.4f).",
      rclcpp::Time(msg->header.stamp).seconds(), age_of_last_command.seconds(),
      ref_timeout_.seconds());
  }
}

void SwerveController::reference_callback_unstamped(
  const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  RCLCPP_WARN(
    get_node()->get_logger(),
    "Use of Twist message without stamped is deprecated and it will be removed in ROS 2 J-Turtle "
    "version. Use '~/reference' topic with 'geometry_msgs::msg::TwistStamped' message type in the "
    "future.");
  auto twist_stamped = *(input_ref_.readFromNonRT());
  twist_stamped->header.stamp = get_node()->now();
  // if no timestamp provided use current time for command timestamp
  if (twist_stamped->header.stamp.sec == 0 && twist_stamped->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
    twist_stamped->header.stamp = get_node()->now();
  }

  const auto age_of_last_command = get_node()->now() - twist_stamped->header.stamp;

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
  {
    twist_stamped->twist = *msg;
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Received message has timestamp %.10f older for %.10f which is more then allowed timeout "
      "(%.4f).",
      rclcpp::Time(twist_stamped->header.stamp).seconds(), age_of_last_command.seconds(),
      ref_timeout_.seconds());
  }
}

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(nr_cmd_itfs_);

  // set all drive joints to VELOCITY interface
  for (size_t i = 0; i < drive_joints_names_.size(); i++)
  {
    command_interfaces_config.names.push_back(
      drive_joints_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  // set all steer joints to POSITION interface
  for (size_t i = 0; i < steer_joints_names_.size(); i++)
  {
    command_interfaces_config.names.push_back(
      steer_joints_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names.reserve(nr_state_itfs_);

  const auto drive_joints_feedback = params_.position_feedback ? hardware_interface::HW_IF_POSITION
                                                               : hardware_interface::HW_IF_VELOCITY;

  for (size_t i = 0; i < drive_joints_names_.size(); i++)
  {
    state_interfaces_config.names.push_back(
      drive_joints_state_names_[i] + "/" + drive_joints_feedback);
  }

  for (size_t i = 0; i < steer_joints_names_.size(); i++)
  {
    state_interfaces_config.names.push_back(
      steer_joints_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> SwerveController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(nr_ref_itfs_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(nr_ref_itfs_);

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), std::string("linear_x/") + hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[0]));

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), std::string("linear_y/") + hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[1]));

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), std::string("angular/") + hardware_interface::HW_IF_POSITION,
      &reference_interfaces_[2]));

  return reference_interfaces;
}

bool SwerveController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < nr_cmd_itfs_; ++i)
  {
    if(!command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN()))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to set command interface %s to NaN on deactivation.",
        command_interfaces_[i].get_name().c_str());
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type SwerveController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto current_ref = *(input_ref_.readFromRT());

  if (!is_in_chained_mode())
  {
    const auto age_of_last_command = time - (current_ref)->header.stamp;
    // send message only if there is no timeout
    if (age_of_last_command <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0))
    {
      if (
        !std::isnan(current_ref->twist.linear.x) && !std::isnan(current_ref->twist.linear.y) &&
        !std::isnan(current_ref->twist.angular.z))
      {
        reference_interfaces_[0] = current_ref->twist.linear.x;
        reference_interfaces_[1] = current_ref->twist.linear.y;
        reference_interfaces_[2] = current_ref->twist.angular.z;
      }
    }
  }
  else
  {
    if (
      !std::isnan(current_ref->twist.linear.x) && !std::isnan(current_ref->twist.linear.y) &&
      !std::isnan(current_ref->twist.angular.z))
    {
      RCLCPP_INFO(get_node()->get_logger(), "REF TIMED OUT");

      reference_interfaces_[0] = 0.0;
      reference_interfaces_[1] = 0.0;
      reference_interfaces_[2] = 0.0;
      current_ref->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
      current_ref->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
      current_ref->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
    }
  }

  update_odometry(period);

  if (
    (std::abs(reference_interfaces_[0]) > 0.001) || (std::abs(reference_interfaces_[1]) > 0.001) ||
    (std::abs(reference_interfaces_[2]) > 0.0349))
  {
    std::vector<double> drive_commands;
    std::vector<double> steer_commands;
    const double steering_track =
      wheel_params_.wheel_track - 2 * wheel_params_.drive_to_steer_offset;

    // store and set commands

    linear_x_command_ = reference_interfaces_[0];
    linear_y_command_ = reference_interfaces_[1];
    angular_command_ = reference_interfaces_[2];

    auto & last_command = previous_commands_.back().twist;
    auto & second_to_last_command = previous_commands_.front().twist;

    // Limit velocities and accelerations:
    limiter_linear_x_.limit(
      linear_x_command_, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
    limiter_linear_y_.limit(
      linear_y_command_, last_command.linear.y, second_to_last_command.linear.y, period.seconds());
    limiter_angular_.limit(
      angular_command_, last_command.angular.z, second_to_last_command.angular.z, period.seconds());

    previous_commands_.pop();
    previous_commands_.emplace(*current_ref);

    // Publish limited velocity
    if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
    {
      auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
      limited_velocity_command.header.stamp = time;
      limited_velocity_command.twist = (current_ref)->twist;
      realtime_limited_velocity_publisher_->unlockAndPublish();
    }

    // Calculate the raw speeds w/o offset
    double fl_speed_x = linear_x_command_ - angular_command_ * steering_track / 2.0;
    double fl_speed_y = linear_y_command_ + angular_command_ * wheel_params_.wheelbase / 2.0;
    double fr_speed_x = linear_x_command_ + angular_command_ * steering_track / 2.0;
    double fr_speed_y = linear_y_command_ + angular_command_ * wheel_params_.wheelbase / 2.0;
    double rl_speed_x = linear_x_command_ - angular_command_ * steering_track / 2.0;
    double rl_speed_y = linear_y_command_ - angular_command_ * wheel_params_.wheelbase / 2.0;
    double rr_speed_x = linear_x_command_ + angular_command_ * steering_track / 2.0;
    double rr_speed_y = linear_y_command_ - angular_command_ * wheel_params_.wheelbase / 2.0;

    std::vector<double> drive_linear_x = {fl_speed_x, fr_speed_x, rl_speed_x, rr_speed_x};
    std::vector<double> drive_linear_y = {fl_speed_y, fr_speed_y, rl_speed_y, rr_speed_y};

    // Create velocities and position variables
    double fl_speed = 0, fr_speed = 0, rr_speed = 0, rl_speed = 0;

    fl_speed = sqrt(pow(fl_speed_x, 2) + pow(fl_speed_y, 2));
    fr_speed = sqrt(pow(fr_speed_x, 2) + pow(fr_speed_y, 2));
    rl_speed = sqrt(pow(rl_speed_x, 2) + pow(rl_speed_y, 2));
    rr_speed = sqrt(pow(rr_speed_x, 2) + pow(rr_speed_y, 2));

    std::vector<double> drive_speeds = {fl_speed, fr_speed, rl_speed, rr_speed};
    std::vector<double> scales;

    float max_drive_speed = 1.0;

    // This part of the code implements some logic from zinger_swerve_controller
    // Shout out to @pvandervelde
    for (std::size_t i = 0; i < drive_speeds.size(); i++)
    {
      float a = drive_speeds[i];
      float b = 0.0;
      float abs_tol = 1e-15;
      float rel_tol = 1e-15;
      float scale = 1.0f;

      if (!is_close(a, b, abs_tol, rel_tol))
      {
        scale = max_drive_speed / a;
        if (scale >= 1.0) scale = 1.0;
      }

      scales.push_back(scale);
    }

    float normalising_factor = *max_element(scales.begin(), scales.end());

    std::vector<DriveModuleDesiredValues> forward_states;
    std::vector<DriveModuleDesiredValues> reverse_states;

    for (std::size_t i = 0; i < drive_speeds.size(); i++)
    {
      auto [forward_angle, reverse_angle] =
        calculate_steering_angles(drive_linear_x[i], drive_linear_y[i], drive_speeds[i]);

      DriveModuleDesiredValues forward_state{forward_angle, drive_speeds[i] * normalising_factor};
      DriveModuleDesiredValues reverse_state{
        reverse_angle, -1 * drive_speeds[i] * normalising_factor};

      forward_states.push_back(forward_state);
      reverse_states.push_back(reverse_state);
    }

    std::vector<DriveModuleDesiredValues> result;
    for (std::size_t i = 0; i < forward_states.size(); i++)
    {
      double current_velocity =
        state_interfaces_[i].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
      double current_steering =
        state_interfaces_[i + forward_states.size()].get_optional().value_or(
          std::numeric_limits<double>::quiet_NaN());
      result.push_back(select_best_state(
        forward_states[i], reverse_states[i], current_velocity, current_steering));
    }

    check_steering_limits(result);

    for (std::size_t i = 0; i < result.size(); i++)
    {
      // converting from m/s to rotational velocity rads/sec
      drive_commands.push_back(result[i].drive_velocity / wheel_params_.radius);
      steer_commands.push_back(result[i].steering_angle);
    }

    find_icrs(steer_commands);

    update_command_interfaces(drive_commands, steer_commands);
  }
  else
  {
    brake();
  }

  // Publish odometry message
  // Compute and store orientation info
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.get_heading());

  // Populate odom message and publish
  if (rt_odom_state_publisher_->trylock())
  {
    rt_odom_state_publisher_->msg_.header.stamp = time;
    rt_odom_state_publisher_->msg_.pose.pose.position.x = odometry_.get_x();
    rt_odom_state_publisher_->msg_.pose.pose.position.y = odometry_.get_y();
    rt_odom_state_publisher_->msg_.pose.pose.orientation = tf2::toMsg(orientation);
    rt_odom_state_publisher_->msg_.twist.twist.linear.x = odometry_.get_linear_x();
    rt_odom_state_publisher_->msg_.twist.twist.linear.y = odometry_.get_linear_y();
    rt_odom_state_publisher_->msg_.twist.twist.angular.z = odometry_.get_angular();
    rt_odom_state_publisher_->unlockAndPublish();
  }

  // Publish tf /odom frame
  if (params_.enable_odom_tf && rt_tf_odom_state_publisher_->trylock())
  {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp = time;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x =
      odometry_.get_x();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y =
      odometry_.get_y();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation =
      tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }

  if (controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.header.stamp = time;
    controller_state_publisher_->msg_.wheel_drive_position.clear();
    controller_state_publisher_->msg_.wheel_drive_velocity.clear();
    controller_state_publisher_->msg_.wheel_steer_position.clear();
    controller_state_publisher_->msg_.wheel_drive_velocity_cmd.clear();
    controller_state_publisher_->msg_.wheel_steering_angle_cmd.clear();

    auto number_of_drive_joints = params_.drive_joints_names.size();

    for (size_t i = 0; i < number_of_drive_joints; ++i)
    {
      if (params_.position_feedback)
      {
        controller_state_publisher_->msg_.wheel_drive_position.push_back(
          state_interfaces_[i].get_optional().value_or(std::numeric_limits<double>::quiet_NaN()));
      }
      else
      {
        controller_state_publisher_->msg_.wheel_drive_velocity.push_back(
          state_interfaces_[i].get_optional().value_or(std::numeric_limits<double>::quiet_NaN()));
      }
      controller_state_publisher_->msg_.wheel_steer_position.push_back(
        state_interfaces_[number_of_drive_joints + i].get_optional().value_or(
          std::numeric_limits<double>::quiet_NaN()));

      controller_state_publisher_->msg_.wheel_drive_velocity_cmd.push_back(
        command_interfaces_[i].get_optional().value_or(std::numeric_limits<double>::quiet_NaN()));

      controller_state_publisher_->msg_.wheel_steering_angle_cmd.push_back(
        command_interfaces_[number_of_drive_joints + i].get_optional().value_or(
          std::numeric_limits<double>::quiet_NaN()));
    }

    controller_state_publisher_->unlockAndPublish();
  }

  reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[2] = std::numeric_limits<double>::quiet_NaN();

  return controller_interface::return_type::OK;
}

void SwerveController::check_steering_limits(std::vector<DriveModuleDesiredValues> & result)
{
  for (std::size_t i = 0; i < result.size(); i++)
  {
    float angle = result[i].steering_angle;

    if (angle > max_steering_angle_)
    {
      // RCLCPP_INFO(get_node()->get_logger(), "More than MAX");
      if (angle - M_PI > min_steering_angle_)
      {
        result[i].steering_angle = angle - M_PI;
        result[i].drive_velocity = -1 * result[i].drive_velocity;
      }
      else  // unreachable so set to max
      {
        result[i].steering_angle = max_steering_angle_;
      }
    }
    else if (angle < min_steering_angle_)
    {
      // RCLCPP_INFO(get_node()->get_logger(), "Less than MIN");
      if (angle + M_PI < max_steering_angle_)
      {
        result[i].steering_angle = angle + M_PI;
        result[i].drive_velocity = -1 * result[i].drive_velocity;
      }
      else  // unreachable so set to min
      {
        result[i].steering_angle = min_steering_angle_;
      }
    }
  }
}

std::pair<double, double> SwerveController::calculate_steering_angles(
  double vx, double vy, double speed)
{
  if (is_close(speed, 0.0, 1e-9, 1e-9))
  {
    return {std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  }
  double cos_angle = std::acos(vx / speed);
  double sin_angle = std::asin(vy / speed);
  double forward_angle;
  if (cos_angle <= 0.5 * M_PI)
  {
    forward_angle = sin_angle;
  }
  else
  {
    forward_angle = (sin_angle < 0) ? difference_between_angles(cos_angle, M_PI) + M_PI : cos_angle;
    forward_angle = normalise_angle(forward_angle);
  }
  double reverse_angle = !std::isinf(forward_angle) ? normalise_angle(forward_angle + M_PI)
                                                    : -std::numeric_limits<double>::infinity();
  return {forward_angle, reverse_angle};
}

const SwerveController::DriveModuleDesiredValues & SwerveController::select_best_state(
  const DriveModuleDesiredValues & forward, const DriveModuleDesiredValues & reverse,
  double current_velocity, double current_steering, double tolerance)
{
  double forward_rot_diff = difference_between_angles(current_steering, forward.steering_angle);
  double reverse_rot_diff = difference_between_angles(current_steering, reverse.steering_angle);
  double forward_vel_diff = forward.drive_velocity - current_velocity;
  double reverse_vel_diff = reverse.drive_velocity - current_velocity;

  if (std::abs(forward_rot_diff) <= std::abs(reverse_rot_diff))
  {
    if (std::abs(forward_vel_diff) <= std::abs(reverse_vel_diff))
    {
      return forward;
    }
    else if (is_close(std::abs(forward_rot_diff), std::abs(reverse_rot_diff), tolerance, tolerance))
    {
      return reverse;
    }
    else
    {
      return forward;
    }
  }
  else
  {
    if (std::abs(reverse_vel_diff) <= std::abs(forward_vel_diff))
    {
      return reverse;
    }
    else if (is_close(std::abs(forward_rot_diff), std::abs(reverse_rot_diff), tolerance, tolerance))
    {
      return forward;
    }
    else
    {
      return reverse;
    }
  }
}

bool SwerveController::reset()
{
  // reset all variables
  odometry_.reset_odometry();

  // release the old queue
  std::queue<ControllerTwistReferenceMsg> empty;
  std::swap(previous_commands_, empty);

  subscriber_is_active_ = false;
  is_halted = false;

  return true;
}

void SwerveController::brake()
{
  // Drive joins come to complete stop
  for (size_t i = 0; i < params_.drive_joints_names.size(); i++)
  {
    if(!command_interfaces_[i].set_value(0.0))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to set drive joint '%s' to 0.0 velocity, check if the joint is configured correctly.",
        params_.drive_joints_names[i].c_str());
    }
  }

  // The steer joints hold the current position
  int no_of_steer_joints = params_.steer_joints_names.size();

  for (size_t i = 0; i < no_of_steer_joints; i++)
  {
    double current_position = state_interfaces_[i + no_of_steer_joints].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
    if(!command_interfaces_[i + no_of_steer_joints].set_value(current_position))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), 
        "Failed to set steer joint '%s' to current position %.2f, check if the joint is configured ",
        params_.steer_joints_names[i].c_str(), current_position);
    }
  }
}

void SwerveController::update_command_interfaces(
  std::vector<double> & drive_values, std::vector<double> & steer_values)
{
  for (std::size_t i = 0; i < drive_joints_names_.size(); i++)
  {
    // TODO check join limits before writing to them here - SANITY CHECK

    // then write
    if(!command_interfaces_[i].set_value(drive_values[i]))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to set drive joint '%s' to %.2f velocity, check if the joint is configured correctly.",
        drive_joints_names_[i].c_str(), drive_values[i]);
    }

    if(!command_interfaces_[i + steer_joints_names_.size()].set_value(steer_values[i]))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to set steer joint '%s' to %.2f position, check if the joint is configured correctly.",
        steer_joints_names_[i].c_str(), steer_values[i]);
    }
  }
  return;
}

bool SwerveController::check_joint_states_are_valid()
{
  bool drive_joints_valid = false;
  bool steer_joints_valid = false;

  const double fl_drive_joint_value =
    state_interfaces_[STATE_DRIVE_FL_WHEEL].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  const double fr_drive_joint_value =
    state_interfaces_[STATE_DRIVE_FR_WHEEL].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  const double rl_drive_joint_value =
    state_interfaces_[STATE_DRIVE_RL_WHEEL].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  const double rr_drive_joint_value =
    state_interfaces_[STATE_DRIVE_RR_WHEEL].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());

  const double fl_steer_joint_value =
    state_interfaces_[STATE_STEER_FL_WHEEL].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  const double fr_steer_joint_value =
    state_interfaces_[STATE_STEER_FR_WHEEL].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  const double rl_steer_joint_value =
    state_interfaces_[STATE_STEER_RL_WHEEL].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  const double rr_steer_joint_value =
    state_interfaces_[STATE_STEER_RR_WHEEL].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());

  if (
    !std::isnan(fr_drive_joint_value) && !std::isnan(fl_drive_joint_value) &&
    !std::isnan(rr_drive_joint_value) && !std::isnan(rl_drive_joint_value))
  {
    drive_joints_valid = true;
  }

  if (
    !std::isnan(fr_steer_joint_value) && !std::isnan(fl_steer_joint_value) &&
    !std::isnan(rr_steer_joint_value) && !std::isnan(rl_steer_joint_value))
  {
    steer_joints_valid = true;
  }

  if (drive_joints_valid && steer_joints_valid)
  {
    drive_joints_values_ = std::vector<double>{
      fl_drive_joint_value, fr_drive_joint_value, rl_drive_joint_value, rr_drive_joint_value};
    steer_joints_values_ = std::vector<double>{
      fl_steer_joint_value, fr_steer_joint_value, rl_steer_joint_value, rr_steer_joint_value};
    return true;
  }

  RCLCPP_WARN(get_node()->get_logger(), "JOINT STATES NOT VALID");

  return false;
}

std::vector<SwerveController::Point> SwerveController::find_wheel_centre_coords()
{
  Point p1 = {1, 1, 0};
  Point p2 = {1, -1, 0};
  Point p3 = {-1, 1, 0};
  Point p4 = {-1, -1, 0};

  std::vector<Point> centres = {p1, p2, p3, p4};

  for (std::size_t i = 0; i < centres.size(); i++)
  {
    // x is vertical
    double wheel_centre_x = centres[i].x * wheel_params_.wheelbase / 2;

    // y is horizontal
    const double steering_track =
      wheel_params_.wheel_track - 2 * wheel_params_.drive_to_steer_offset;

    double wheel_centre_y = centres[i].y * steering_track / 2;

    centres[i] = {wheel_centre_x, wheel_centre_y, wheel_params_.radius};
  }

  return centres;
}

void SwerveController::find_icrs(std::vector<double> angles)
{
  std::vector<Point> wheel_centres = find_wheel_centre_coords();

  Line icr_line;
  std::vector<Line> icr_lines;

  for (std::size_t i = 0; i < angles.size(); i++)
  {
    icr_line.p1 = {wheel_centres[i].x, wheel_centres[i].y, 0.0};
    icr_line.p2 = {-sin(angles[i]) + wheel_centres[i].x, cos(angles[i]) + wheel_centres[i].y, 0.0};

    icr_lines.push_back(icr_line);
  }

  std::vector<std::vector<double>> icrs;

  for (std::size_t i = 0; i < icr_lines.size(); i++)
  {
    Line segment_one = icr_lines[i];

    for (std::size_t j = 0; j < icr_lines.size(); j++)
    {
      if (j != i)
      {
        Line segment_two = icr_lines[j];

        // Find the intersection point of the two lines
        Eigen::Vector3d seg_one_p1{segment_one.p1.x, segment_one.p1.y, 1.0};
        Eigen::Vector3d seg_one_p2{segment_one.p2.x, segment_one.p2.y, 1.0};
        Eigen::Vector3d seg_two_p1{segment_two.p1.x, segment_two.p1.y, 1.0};
        Eigen::Vector3d seg_two_p2{segment_two.p2.x, segment_two.p2.y, 1.0};

        Eigen::Vector3d l1 = seg_one_p1.cross(seg_one_p2);
        Eigen::Vector3d l2 = seg_two_p1.cross(seg_two_p2);

        Eigen::Vector3d result = l1.cross(l2);

        float abs_tol = 1e-15;
        float rel_tol = 1e-15;
        if (!is_close(result.z(), 0.0, abs_tol, rel_tol))
        {
          icrs.push_back({result.x() / result.z(), result.y() / result.z()});
        }
      }
    }
  }

  publish_icrs(icrs);
}

void SwerveController::publish_icrs(std::vector<std::vector<double>> icr_list)
{
  geometry_msgs::msg::Vector3 scale_;
  std_msgs::msg::ColorRGBA color_;

  icr_publisher_->lock();
  icr_publisher_->msg_.header.stamp = get_node()->now();
  icr_publisher_->msg_.header.frame_id = "base_footprint";
  icr_publisher_->msg_.type = 8;    // list of points
  icr_publisher_->msg_.action = 0;  // add/modify

  scale_.x = 0.1;
  scale_.y = 0.1;
  scale_.z = 0.1;
  icr_publisher_->msg_.scale = scale_;

  color_.r = 1.0;
  color_.g = 1.0;
  color_.b = 0.0;
  color_.a = 1.0;

  icr_publisher_->msg_.points.clear();
  icr_publisher_->msg_.colors.clear();

  for (std::size_t i = 0; i < icr_list.size(); i++)
  {
    float x_icr = icr_list[i][0];
    float y_icr = icr_list[i][1];

    geometry_msgs::msg::Point point;
    point.x = x_icr;
    point.y = y_icr;
    point.z = 0.0;
    icr_publisher_->msg_.points.push_back(point);
    icr_publisher_->msg_.colors.push_back(color_);
  }

  float x_body_icr = -linear_y_command_ / angular_command_;
  float y_body_icr = linear_x_command_ / angular_command_;

  geometry_msgs::msg::Point point;
  point.x = x_body_icr;
  point.y = y_body_icr;
  point.z = 0.0;

  color_.r = 1.0;
  color_.g = 0.38;
  color_.b = 0.278;
  color_.a = 1.0;
  icr_publisher_->msg_.points.push_back(point);
  icr_publisher_->msg_.colors.push_back(color_);

  icr_publisher_->unlockAndPublish();
}

bool SwerveController::is_close(float a, float b, float abs_tol, float rel_tol)
{
  if ((std::abs(a - b) <= (abs_tol + rel_tol * std::abs(b))))
    return true;
  else
    return false;
}

double SwerveController::normalise_angle(float angle)
{
  // reduce the angle to [-2pi, 2pi]
  angle = fmod(angle, (2 * M_PI));

  // Force the angle to the between 0 and 2pi
  angle = fmod((angle + 2 * M_PI), (2 * M_PI));

  if (angle > M_PI) angle -= 2 * M_PI;

  return angle;

}

double SwerveController::difference_between_angles(float a, float b)
{
  float normalized_start = normalise_angle(a);
  float normalized_end = normalise_angle(b);

  float delta = normalized_end - normalized_start;
  // make sure we get the smallest angle
  if (delta > M_PI)
  {
    delta -= 2 * M_PI;
  }
  else
  {
    if (delta < -M_PI)
    {
      delta += 2 * M_PI;
    }
  }
  return delta;
}
}  // namespace swerve_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  swerve_controller::SwerveController, controller_interface::ChainableControllerInterface)
