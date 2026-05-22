// Copyright 2020 PAL Robotics SL.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "swerve_controller/swerve_controller.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using lifecycle_msgs::msg::State;
using testing::SizeIs;

using ControllerStateMsg = swerve_controller::SwerveController::SwerveControllerState;
using ControllerReferenceMsg = swerve_controller::SwerveController::ControllerTwistReferenceMsg;

namespace
{
const std::vector<std::string> drive_joints_names = {
  "fl_drive_joint", "fr_drive_joint", "rl_drive_joint", "rr_drive_joint"};
const std::vector<std::string> steer_joints_names = {
  "fl_steer_joint", "fr_steer_joint", "rl_steer_joint", "rr_steer_joint"};
}  // namespace

class TestableSwerveController : public swerve_controller::SwerveController
{
public:
  using SwerveController::SwerveController;
  /**
   * @brief wait_for_twist block until a new twist is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function
   */
  void wait_for_twist(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(500))
  {
    auto until = get_node()->get_clock()->now() + timeout;
    while (get_node()->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }
};

class TestSwerveController : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  void SetUp() override
  {
    controller_ = std::make_unique<TestableSwerveController>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      controller_name + "/reference", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  /// Publish velocity msgs
  /**
   *  linear_x - magnitude of the linear command in the geometry_msgs::twist message
   *  linear_y - magnitude of the linear command in the geometry_msgs::twist message
   *  angular - the magnitude of the angular command in geometry_msgs::twist message
   */
  void publish(double linear_x, double linear_y, double angular)
  {
    int wait_count = 0;
    auto topic = command_publisher_->get_topic_name();
    while (command_publisher_node_->count_subscribers(topic) == 0)
    {
      if (wait_count >= 5)
      {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    geometry_msgs::msg::TwistStamped velocity_message;
    velocity_message.header.stamp = command_publisher_node_->get_clock()->now();
    velocity_message.twist.linear.x = linear_x;
    velocity_message.twist.linear.y = linear_y;
    velocity_message.twist.angular.z = angular;
    command_publisher_->publish(velocity_message);
  }

  /// \brief wait for the subscriber and publisher to completely setup
  void waitForSetup(rclcpp::Executor & executor)
  {
    constexpr std::chrono::seconds TIMEOUT{2};
    auto clock = command_publisher_node_->get_clock();
    auto start = clock->now();
    while (command_publisher_->get_subscription_count() <= 0)
    {
      if ((clock->now() - start) > TIMEOUT)
      {
        FAIL() << "Timeout waiting for subscriber to connect";
        return;  // FAIL() does not exit the function; return to prevent infinite loop
      }
      executor.spin_some();
      rclcpp::spin_some(command_publisher_node_);
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  void assignResources()
  {
    std::vector<LoanedStateInterface> state_ifs;
    state_ifs.emplace_back(fl_drive_state_);
    state_ifs.emplace_back(fr_drive_state_);
    state_ifs.emplace_back(rl_drive_state_);
    state_ifs.emplace_back(rr_drive_state_);
    state_ifs.emplace_back(fl_steer_state_);
    state_ifs.emplace_back(fr_steer_state_);
    state_ifs.emplace_back(rl_steer_state_);
    state_ifs.emplace_back(rr_steer_state_);

    std::vector<LoanedCommandInterface> command_ifs;
    command_ifs.emplace_back(fl_drive_cmd_);
    command_ifs.emplace_back(fr_drive_cmd_);
    command_ifs.emplace_back(rl_drive_cmd_);
    command_ifs.emplace_back(rr_drive_cmd_);
    command_ifs.emplace_back(fl_steer_cmd_);
    command_ifs.emplace_back(fr_steer_cmd_);
    command_ifs.emplace_back(rl_steer_cmd_);
    command_ifs.emplace_back(rr_steer_cmd_);

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  controller_interface::return_type InitController(
    const std::vector<std::string> drive_joints = drive_joints_names,
    const std::vector<std::string> steer_joints = steer_joints_names,
    const std::vector<rclcpp::Parameter> & parameters = {}, const std::string ns = "")
  {
    auto node_options = rclcpp::NodeOptions();
    std::vector<rclcpp::Parameter> parameter_overrides;

    parameter_overrides.push_back(
      rclcpp::Parameter("drive_joints_names", rclcpp::ParameterValue(drive_joints)));
    parameter_overrides.push_back(
      rclcpp::Parameter("steer_joints_names", rclcpp::ParameterValue(steer_joints)));

    parameter_overrides.push_back(
      rclcpp::Parameter("drive_joints_state_names", rclcpp::ParameterValue(drive_joints)));
    parameter_overrides.push_back(
      rclcpp::Parameter("steer_joints_state_names", rclcpp::ParameterValue(steer_joints)));

    // default parameters
    parameter_overrides.push_back(rclcpp::Parameter("wheelbase", rclcpp::ParameterValue(1.8)));
    parameter_overrides.push_back(rclcpp::Parameter("wheel_radius", rclcpp::ParameterValue(0.2)));
    parameter_overrides.push_back(rclcpp::Parameter("wheel_track", rclcpp::ParameterValue(2.3)));
    parameter_overrides.push_back(
      rclcpp::Parameter("drive_to_steer_offset", rclcpp::ParameterValue(0.0)));

    parameter_overrides.insert(parameter_overrides.end(), parameters.begin(), parameters.end());
    node_options.parameter_overrides(parameter_overrides);

    const auto update_rate = 0;
    const auto controller_name = "test_swerve_controller";
    return controller_->init(controller_name, urdf_, update_rate, ns, node_options);
  }

  const std::string controller_name = "test_swerve_controller";
  std::unique_ptr<TestableSwerveController> controller_;

  std::vector<double> position_values_ = {0.0, 0.0, 0.0, 0.0};
  std::vector<double> velocity_values_ = {0.0, 0.0, 0.0, 0.0};

  hardware_interface::StateInterface fl_drive_state_{
    drive_joints_names[0], HW_IF_VELOCITY, &velocity_values_[0]};

  hardware_interface::StateInterface fr_drive_state_{
    drive_joints_names[1], HW_IF_VELOCITY, &velocity_values_[1]};

  hardware_interface::StateInterface rl_drive_state_{
    drive_joints_names[2], HW_IF_VELOCITY, &velocity_values_[2]};

  hardware_interface::StateInterface rr_drive_state_{
    drive_joints_names[3], HW_IF_VELOCITY, &velocity_values_[3]};

  hardware_interface::StateInterface fl_steer_state_{
    steer_joints_names[0], HW_IF_POSITION, &position_values_[0]};

  hardware_interface::StateInterface fr_steer_state_{
    steer_joints_names[1], HW_IF_POSITION, &position_values_[1]};

  hardware_interface::StateInterface rl_steer_state_{
    steer_joints_names[2], HW_IF_POSITION, &position_values_[2]};

  hardware_interface::StateInterface rr_steer_state_{
    steer_joints_names[3], HW_IF_POSITION, &position_values_[3]};

  hardware_interface::CommandInterface fl_drive_cmd_{
    drive_joints_names[0], HW_IF_VELOCITY, &velocity_values_[0]};

  hardware_interface::CommandInterface fr_drive_cmd_{
    drive_joints_names[1], HW_IF_VELOCITY, &velocity_values_[1]};

  hardware_interface::CommandInterface rl_drive_cmd_{
    drive_joints_names[2], HW_IF_VELOCITY, &velocity_values_[2]};

  hardware_interface::CommandInterface rr_drive_cmd_{
    drive_joints_names[3], HW_IF_VELOCITY, &velocity_values_[3]};

  hardware_interface::CommandInterface fl_steer_cmd_{
    steer_joints_names[0], HW_IF_POSITION, &position_values_[0]};

  hardware_interface::CommandInterface fr_steer_cmd_{
    steer_joints_names[1], HW_IF_POSITION, &position_values_[1]};

  hardware_interface::CommandInterface rl_steer_cmd_{
    steer_joints_names[2], HW_IF_POSITION, &position_values_[2]};

  hardware_interface::CommandInterface rr_steer_cmd_{
    steer_joints_names[3], HW_IF_POSITION, &position_values_[3]};

  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;

  const std::string urdf_ = "";
};

TEST_F(TestSwerveController, init_fails_without_parameters)
{
  const auto ret = controller_->init(
    "test_swerve_controller", "", 0, "", controller_->define_custom_node_options());
  ASSERT_EQ(ret, controller_interface::return_type::ERROR);
}

TEST_F(TestSwerveController, configure_succeeds_when_wheels_are_specified)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto state_if_conf = controller_->state_interface_configuration();
  ASSERT_THAT(state_if_conf.names, SizeIs(drive_joints_names.size() + steer_joints_names.size()));
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto cmd_if_conf = controller_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, SizeIs(drive_joints_names.size() + steer_joints_names.size()));
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(TestSwerveController, activate_succeeds_with_resources_assigned)
{
  ASSERT_EQ(
    InitController(drive_joints_names, steer_joints_names), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  assignResources();
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(TestSwerveController, correct_initialization_using_parameters)
{
  ASSERT_EQ(
    InitController(
      drive_joints_names, steer_joints_names,
      {rclcpp::Parameter("wheel_radius", 0.2), rclcpp::Parameter("wheelbase", 1.8),
       rclcpp::Parameter("wheel_track", 2.3), rclcpp::Parameter("drive_to_steer_offset", 0.0),
       rclcpp::Parameter("use_stamped_vel", true)}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  assignResources();
  // Export reference interfaces so that reference_interfaces_ vector is populated
  // (normally done by the controller manager, but required in unit tests)
  auto ref_interfaces = controller_->export_reference_interfaces();
  ASSERT_EQ(ref_interfaces.size(), 3u);

  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(0.00, fl_drive_cmd_.get_optional().value());
  EXPECT_EQ(0.00, fr_drive_cmd_.get_optional().value());
  EXPECT_EQ(0.00, rl_drive_cmd_.get_optional().value());
  EXPECT_EQ(0.00, rr_drive_cmd_.get_optional().value());
  EXPECT_EQ(0.00, fl_steer_cmd_.get_optional().value());
  EXPECT_EQ(0.00, fr_steer_cmd_.get_optional().value());
  EXPECT_EQ(0.00, rl_steer_cmd_.get_optional().value());
  EXPECT_EQ(0.00, rr_steer_cmd_.get_optional().value());

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

  waitForSetup(executor);

  // send msg
  const double linear_x = 1.0;
  const double linear_y = 1.0;
  const double angular = 0.0;
  publish(linear_x, linear_y, angular);
  // wait for msg is be published to the system
  controller_->wait_for_twist(executor);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // The controller normalizes wheel speeds to max_drive_speed (1.0 m/s).
  // With linear_x=1.0, linear_y=1.0, the raw wheel speed is sqrt(2) ≈ 1.414 m/s > 1.0 m/s,
  // so all speeds are scaled down proportionally: effective_speed = 1.0 m/s.
  const double expected_wheel_vel = 1.0 / 0.2;  // max_drive_speed / wheel_radius
  EXPECT_NEAR(expected_wheel_vel, fl_drive_cmd_.get_optional().value(), 0.01);
  EXPECT_NEAR(expected_wheel_vel, fr_drive_cmd_.get_optional().value(), 0.01);
  EXPECT_NEAR(expected_wheel_vel, rl_drive_cmd_.get_optional().value(), 0.01);
  EXPECT_NEAR(expected_wheel_vel, rr_drive_cmd_.get_optional().value(), 0.01);

  const double expected_wheel_angle = std::atan2(linear_y, linear_x);
  EXPECT_NEAR(expected_wheel_angle, fl_steer_cmd_.get_optional().value(), 0.01);
  EXPECT_NEAR(expected_wheel_angle, fr_steer_cmd_.get_optional().value(), 0.01);
  EXPECT_NEAR(expected_wheel_angle, rl_steer_cmd_.get_optional().value(), 0.01);
  EXPECT_NEAR(expected_wheel_angle, rr_steer_cmd_.get_optional().value(), 0.01);

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(0.0, fl_drive_cmd_.get_optional().value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, fr_drive_cmd_.get_optional().value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, rl_drive_cmd_.get_optional().value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, rr_drive_cmd_.get_optional().value()) << "Wheels are halted on deactivate()";

  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(0.0, fl_drive_cmd_.get_optional().value());
  EXPECT_EQ(0.0, fr_drive_cmd_.get_optional().value());
  EXPECT_EQ(0.0, rl_drive_cmd_.get_optional().value());
  EXPECT_EQ(0.0, rr_drive_cmd_.get_optional().value());

  EXPECT_EQ(0.0, fl_steer_cmd_.get_optional().value());
  EXPECT_EQ(0.0, fr_steer_cmd_.get_optional().value());
  EXPECT_EQ(0.0, rl_steer_cmd_.get_optional().value());
  EXPECT_EQ(0.0, rr_steer_cmd_.get_optional().value());

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}
