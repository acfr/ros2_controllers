// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef TEST_SWERVE_CONTROLLER_HPP_
#define TEST_SWERVE_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "gmock/gmock.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "swerve_controller/swerve_controller.hpp"

using ControllerStateMsg = swerve_controller::SwerveController::SwerveControllerState;
using ControllerReferenceMsg = swerve_controller::SwerveController::ControllerTwistReferenceMsg;

// NOTE: Testing swerve_controller for ackermann vehicle configuration only

// name constants for state interfaces
static constexpr size_t STATE_DRIVE_FL_WHEEL = 0;
static constexpr size_t STATE_DRIVE_FR_WHEEL = 1;
static constexpr size_t STATE_DRIVE_RL_WHEEL = 2;
static constexpr size_t STATE_DRIVE_RR_WHEEL = 3;
static constexpr size_t STATE_STEER_FL_WHEEL = 4;
static constexpr size_t STATE_STEER_FR_WHEEL = 5;
static constexpr size_t STATE_STEER_RL_WHEEL = 6;
static constexpr size_t STATE_STEER_RR_WHEEL = 7;

// name constants for command interfaces
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

static constexpr double WHEELBASE_ = 1.260;
static constexpr double WHEEL_TRACK_ = 1.260;
static constexpr double WHEEL_RADIUS_ = 0.328;
static constexpr double DRIVE_TO_STEER_OFFSET = 0.0415;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace
// namespace

// subclassing and friending so we can access member variables
class TestableSwerveController : public swerve_controller::SwerveController
{
  FRIEND_TEST(SwerveControllerTest, check_exported_interfaces);
  FRIEND_TEST(SwerveControllerTest, test_both_update_methods_for_ref_timeout);

public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    return swerve_controller::SwerveController::on_configure(previous_state);
  }

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override
  {
    auto ref_itfs = on_export_reference_interfaces();
    return swerve_controller::SwerveController::on_activate(previous_state);
  }

  /**
   * @brief wait_for_command blocks until a new ControllerReferenceMsg is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function.
   */
  void wait_for_command(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    auto until = get_node()->get_clock()->now() + timeout;
    while (get_node()->get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  void wait_for_commands(
    rclcpp::Executor & executor,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds{500})
  {
    wait_for_command(executor, timeout);
  }

  // implementing methods which are declared virtual in the swerve_controller.hpp
  void initialize_implementation_parameter_listener()
  {
    param_listener_ = std::make_shared<swerve_controller::ParamListener>(get_node());
  }

  controller_interface::CallbackReturn configure_odometry()
  {
    set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);
    odometry_.set_wheel_params(WHEEL_RADIUS_, WHEELBASE_, WHEEL_TRACK_, DRIVE_TO_STEER_OFFSET);

    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool update_odometry(const rclcpp::Duration & /*period*/) { return true; }
};

// We are using template class here for easier reuse of Fixture in specializations of controllers
template <typename CtrlType>
class SwerveControllerFixture : public ::testing::Test
{
public:
  static void SetUpTestCase() {}

  void SetUp()
  {
    // initialize controller
    controller_ = std::make_unique<CtrlType>();

    command_publisher_node_ = std::make_shared<rclcpp::Node>("command_publisher");
    command_publisher_ = command_publisher_node_->create_publisher<ControllerReferenceMsg>(
      "/test_swerve_controller/reference", rclcpp::SystemDefaultsQoS());
  }

  static void TearDownTestCase() {}

  void TearDown() { controller_.reset(nullptr); }

protected:
  void SetUpController(const std::string controller_name = "test_swerve_controller")
  {
    ASSERT_EQ(controller_->init(controller_name), controller_interface::return_type::OK);

    std::vector<hardware_interface::LoanedCommandInterface> command_ifs;
    command_itfs_.reserve(joint_command_values_.size());
    command_ifs.reserve(joint_command_values_.size());

    for (std::size_t i = 0; i < joint_command_values_.size(); i++)
    {
      if (i < 4)
      {
        hw_interface_name_ = "velocity";
      }
      else
      {
        hw_interface_name_ = "position";
      }

      command_itfs_.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hw_interface_name_, &joint_command_values_[i]));
      command_ifs.emplace_back(command_itfs_.back());
    }

    std::vector<hardware_interface::LoanedStateInterface> state_ifs;
    state_itfs_.reserve(joint_state_values_.size());
    state_ifs.reserve(joint_state_values_.size());

    for (std::size_t i = 0; i < joint_command_values_.size(); i++)
    {
      if (i < 4)
      {
        hw_interface_name_ = "velocity";
      }
      else
      {
        hw_interface_name_ = "position";
      }

      state_itfs_.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hw_interface_name_, &joint_state_values_[i]));
      state_ifs.emplace_back(state_itfs_.back());
    }

    controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
  }

  void subscribe_and_get_messages(ControllerStateMsg & msg)
  {
    // create a new subscriber
    ControllerStateMsg::SharedPtr received_msg;
    rclcpp::Node test_subscription_node("test_subscription_node");
    auto subs_callback = [&](const ControllerStateMsg::SharedPtr cb_msg) { received_msg = cb_msg; };
    auto subscription = test_subscription_node.create_subscription<ControllerStateMsg>(
      "/test_swerve_controller/controller_state", 10, subs_callback);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_subscription_node.get_node_base_interface());

    // call update to publish the test value
    ASSERT_EQ(
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01)),
      controller_interface::return_type::OK);
    // call update to publish the test value
    // since update doesn't guarantee a published message, republish until received
    int max_sub_check_loop_count = 5;  // max number of tries for pub/sub loop
    while (max_sub_check_loop_count--)
    {
      controller_->update(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.01));
      const auto timeout = std::chrono::milliseconds{1};
      const auto until = test_subscription_node.get_clock()->now() + timeout;
      while (!received_msg && test_subscription_node.get_clock()->now() < until)
      {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
      // check if message has been received
      if (received_msg.get())
      {
        break;
      }
    }
    ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
                                              "controller/broadcaster update loop";
    ASSERT_TRUE(received_msg);

    // take message from subscription
    msg = *received_msg;
  }

  void publish_commands(const double linear = 0.1, const double angular = 0.2)
  {
    auto wait_for_topic = [&](const auto topic_name)
    {
      size_t wait_count = 0;
      while (command_publisher_node_->count_subscribers(topic_name) == 0)
      {
        if (wait_count >= 5)
        {
          auto error_msg =
            std::string("publishing to ") + topic_name + " but no node subscribes to it";
          throw std::runtime_error(error_msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++wait_count;
      }
    };

    wait_for_topic(command_publisher_->get_topic_name());

    ControllerReferenceMsg msg;
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;

    command_publisher_->publish(msg);
  }

protected:
  // Controller-related parameters
  double reference_timeout_ = 2.0;
  bool front_steering_ = true;
  bool open_loop_ = false;
  unsigned int velocity_rolling_window_size_ = 10;
  bool position_feedback_ = false;
  bool use_stamped_vel_ = true;

  std::vector<std::string> joint_names_ = {
    "fl_drive_joint",
    "fr_drive_joint",
    "rl_drive_joint",
    "rr_drive_joint",
    "fl_steer_joint",
    "fr_steer_joint",
    "rl_steer_joint",
    "rr_steer_joint",
  };

  std::vector<std::string> preceding_joint_names_ = {
    "drive_pid_controller/fl_drive_joint",
    "drive_pid_controller/fr_drive_joint",
    "drive_pid_controller/rl_drive_joint",
    "drive_pid_controller/rr_drive_joint",
    "steer_pid_controller/fl_steer_joint",
    "steer_pid_controller/fr_steer_joint",
    "steer_pid_controller/rl_steer_joint",
    "steer_pid_controller/rr_steer_joint",
  };

  double wheelbase_ = 1.260;
  double wheel_track_ = 1.260;
  double wheels_radius_ = 0.328;
  double drive_to_steer_offset_ = 0.0415;

  std::array<double, 8> joint_state_values_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 8> joint_command_values_ = {1.1, 3.3, 2.2, 4.4, 1.1, 3.3, 2.2, 4.4};

  std::array<std::string, 2> joint_reference_interfaces_ = {"linear/velocity", "angular/velocity"};
  std::string steering_interface_name_ = "position";
  // defined in setup
  std::string hw_interface_name_ = "";
  std::string preceeding_prefix_ = "pid_controller";

  std::vector<hardware_interface::StateInterface> state_itfs_;
  std::vector<hardware_interface::CommandInterface> command_itfs_;

  // Test related parameters
  std::unique_ptr<CtrlType> controller_;
  rclcpp::Node::SharedPtr command_publisher_node_;
  rclcpp::Publisher<ControllerReferenceMsg>::SharedPtr command_publisher_;
};

#endif  // TEST_SWERVE_CONTROLLER_HPP_
