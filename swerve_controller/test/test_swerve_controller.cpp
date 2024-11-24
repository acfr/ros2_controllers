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
  void waitForSetup()
  {
    constexpr std::chrono::seconds TIMEOUT{2};
    auto clock = command_publisher_node_->get_clock();
    auto start = clock->now();
    while (command_publisher_->get_subscription_count() <= 0)
    {
      if ((clock->now() - start) > TIMEOUT)
      {
        FAIL();
      }
      rclcpp::spin_some(command_publisher_node_);
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

    return controller_->init(controller_name, ns, node_options);
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
};

TEST_F(TestSwerveController, init_fails_without_parameters)
{
  const auto ret = controller_->init(controller_name);
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

TEST_F(TestSwerveController, activate_fails_without_resources_assigned)
{
  ASSERT_EQ(InitController(), controller_interface::return_type::OK);

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
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
       rclcpp::Parameter("wheel_track", 2.3), rclcpp::Parameter("drive_to_steer_offset", 0.0)}),
    controller_interface::return_type::OK);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());

  auto state = controller_->get_node()->configure();
  assignResources();

  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  EXPECT_EQ(0.00, fl_drive_cmd_.get_value());
  EXPECT_EQ(0.00, fr_drive_cmd_.get_value());
  EXPECT_EQ(0.00, rl_drive_cmd_.get_value());
  EXPECT_EQ(0.00, rr_drive_cmd_.get_value());

  state = controller_->get_node()->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, state.id());

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
  EXPECT_EQ(1.0, fl_drive_cmd_.get_value());
  EXPECT_EQ(1.0, fr_drive_cmd_.get_value());
  EXPECT_EQ(1.0, rl_drive_cmd_.get_value());
  EXPECT_EQ(1.0, rr_drive_cmd_.get_value());

  // deactivated
  // wait so controller process the second point when deactivated
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  state = controller_->get_node()->deactivate();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(
    controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(0.0, fl_drive_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, fr_drive_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, rl_drive_cmd_.get_value()) << "Wheels are halted on deactivate()";
  EXPECT_EQ(0.0, rr_drive_cmd_.get_value()) << "Wheels are halted on deactivate()";
  // cleanup
  state = controller_->get_node()->cleanup();
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, state.id());
  EXPECT_EQ(0.0, fl_drive_cmd_.get_value());
  EXPECT_EQ(0.0, fr_drive_cmd_.get_value());
  EXPECT_EQ(0.0, rl_drive_cmd_.get_value());
  EXPECT_EQ(0.0, rr_drive_cmd_.get_value());

  state = controller_->get_node()->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, state.id());
  executor.cancel();
}
