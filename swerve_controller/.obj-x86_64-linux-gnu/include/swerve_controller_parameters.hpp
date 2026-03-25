#pragma message("#include \"swerve_controller_parameters.hpp\" is deprecated. Use #include <swerve_controller/swerve_controller_parameters.hpp> instead.")
// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <parameter_traits/parameter_traits.hpp>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace swerve_controller {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

// temporarily needed for backwards compatibility for custom validators
using namespace parameter_traits;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        double reference_timeout = 1.0;
        std::vector<std::string> steer_joints_names;
        std::vector<std::string> drive_joints_names;
        std::vector<std::string> steer_joints_state_names;
        std::vector<std::string> drive_joints_state_names;
        bool use_stamped_vel = false;
        bool open_loop = false;
        int64_t velocity_rolling_window_size = 10;
        std::string base_frame_id = "base_link";
        std::string odom_frame_id = "odom";
        bool enable_odom_tf = true;
        std::vector<double> twist_covariance_diagonal = {0.0, 7.0, 14.0, 21.0, 28.0, 35.0};
        std::vector<double> pose_covariance_diagonal = {0.0, 7.0, 14.0, 21.0, 28.0, 35.0};
        bool position_feedback = false;
        double max_steering_limit = 0.87267;
        double min_steering_limit = -0.87267;
        double wheel_track = 0.0;
        double wheelbase = 0.0;
        double wheel_radius = 0.0;
        double drive_to_steer_offset = 0.0;
        bool publish_limited_velocity = false;
        double cmd_vel_timeout = 0.5;
        struct Linear {
            struct X {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } x;
            struct Y {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } y;
        } linear;
        struct Angular {
            struct Z {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } z;
        } angular;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        double reference_timeout = 1.0;
        bool use_stamped_vel = false;
        bool open_loop = false;
        int64_t velocity_rolling_window_size = 10;
        bool enable_odom_tf = true;
        bool position_feedback = false;
        double max_steering_limit = 0.87267;
        double min_steering_limit = -0.87267;
        double wheel_track = 0.0;
        double wheelbase = 0.0;
        double wheel_radius = 0.0;
        double drive_to_steer_offset = 0.0;
        bool publish_limited_velocity = false;
        double cmd_vel_timeout = 0.5;
        struct Linear {
            struct X {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } x;
            struct Y {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } y;
        } linear;
        struct Angular {
            struct Z {
                bool has_velocity_limits = false;
                bool has_acceleration_limits = false;
                bool has_jerk_limits = false;
                double max_velocity = std::numeric_limits<double>::quiet_NaN();
                double min_velocity = std::numeric_limits<double>::quiet_NaN();
                double max_acceleration = std::numeric_limits<double>::quiet_NaN();
                double min_acceleration = std::numeric_limits<double>::quiet_NaN();
                double max_jerk = std::numeric_limits<double>::quiet_NaN();
                double min_jerk = std::numeric_limits<double>::quiet_NaN();
            } z;
        } angular;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    template <typename NodeT>
    ParamListener(NodeT node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("swerve_controller"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "")
    : prefix_{prefix},
      logger_{std::move(logger)} {
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    /**
     * @brief Tries to update the parsed Params object
     * @param params_in The Params object to update
     * @return true if the Params object was updated, false if it was already up to date or the mutex could not be locked
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_update_params(Params & params_in) const {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
          return true;
        }
      }
      return false;
    }

    /**
     * @brief Tries to get the current Params object
     * @param params_in The Params object to fill with the current parameters
     * @return true if mutex can be locked, false if mutex could not be locked
     * @note The parameters are only filled, when the mutex can be locked and the params timestamp is different
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.reference_timeout = params.reference_timeout;
      output.use_stamped_vel = params.use_stamped_vel;
      output.open_loop = params.open_loop;
      output.velocity_rolling_window_size = params.velocity_rolling_window_size;
      output.enable_odom_tf = params.enable_odom_tf;
      output.position_feedback = params.position_feedback;
      output.max_steering_limit = params.max_steering_limit;
      output.min_steering_limit = params.min_steering_limit;
      output.wheel_track = params.wheel_track;
      output.wheelbase = params.wheelbase;
      output.wheel_radius = params.wheel_radius;
      output.drive_to_steer_offset = params.drive_to_steer_offset;
      output.publish_limited_velocity = params.publish_limited_velocity;
      output.linear.x.has_velocity_limits = params.linear.x.has_velocity_limits;
      output.linear.x.has_acceleration_limits = params.linear.x.has_acceleration_limits;
      output.linear.x.has_jerk_limits = params.linear.x.has_jerk_limits;
      output.linear.x.max_velocity = params.linear.x.max_velocity;
      output.linear.x.min_velocity = params.linear.x.min_velocity;
      output.linear.x.max_acceleration = params.linear.x.max_acceleration;
      output.linear.x.min_acceleration = params.linear.x.min_acceleration;
      output.linear.x.max_jerk = params.linear.x.max_jerk;
      output.linear.x.min_jerk = params.linear.x.min_jerk;
      output.linear.y.has_velocity_limits = params.linear.y.has_velocity_limits;
      output.linear.y.has_acceleration_limits = params.linear.y.has_acceleration_limits;
      output.linear.y.has_jerk_limits = params.linear.y.has_jerk_limits;
      output.linear.y.max_velocity = params.linear.y.max_velocity;
      output.linear.y.min_velocity = params.linear.y.min_velocity;
      output.linear.y.max_acceleration = params.linear.y.max_acceleration;
      output.linear.y.min_acceleration = params.linear.y.min_acceleration;
      output.linear.y.max_jerk = params.linear.y.max_jerk;
      output.linear.y.min_jerk = params.linear.y.min_jerk;
      output.angular.z.has_velocity_limits = params.angular.z.has_velocity_limits;
      output.angular.z.has_acceleration_limits = params.angular.z.has_acceleration_limits;
      output.angular.z.has_jerk_limits = params.angular.z.has_jerk_limits;
      output.angular.z.max_velocity = params.angular.z.max_velocity;
      output.angular.z.min_velocity = params.angular.z.min_velocity;
      output.angular.z.max_acceleration = params.angular.z.max_acceleration;
      output.angular.z.min_acceleration = params.angular.z.min_acceleration;
      output.angular.z.max_jerk = params.angular.z.max_jerk;
      output.angular.z.min_jerk = params.angular.z.min_jerk;
      output.cmd_vel_timeout = params.cmd_vel_timeout;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "reference_timeout")) {
            updated_params.reference_timeout = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steer_joints_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.steer_joints_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "drive_joints_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.drive_joints_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steer_joints_state_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.steer_joints_state_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "drive_joints_state_names")) {
            if(auto validation_result = size_lt<std::string>(param, 5);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = unique<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = not_empty<std::string>(param);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.drive_joints_state_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "use_stamped_vel")) {
            updated_params.use_stamped_vel = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "open_loop")) {
            updated_params.open_loop = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "velocity_rolling_window_size")) {
            updated_params.velocity_rolling_window_size = param.as_int();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "base_frame_id")) {
            updated_params.base_frame_id = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "odom_frame_id")) {
            updated_params.odom_frame_id = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "enable_odom_tf")) {
            updated_params.enable_odom_tf = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "twist_covariance_diagonal")) {
            updated_params.twist_covariance_diagonal = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "pose_covariance_diagonal")) {
            updated_params.pose_covariance_diagonal = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "position_feedback")) {
            updated_params.position_feedback = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "max_steering_limit")) {
            updated_params.max_steering_limit = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "min_steering_limit")) {
            updated_params.min_steering_limit = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_track")) {
            updated_params.wheel_track = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheelbase")) {
            updated_params.wheelbase = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_radius")) {
            updated_params.wheel_radius = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "drive_to_steer_offset")) {
            updated_params.drive_to_steer_offset = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "publish_limited_velocity")) {
            updated_params.publish_limited_velocity = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.has_velocity_limits")) {
            updated_params.linear.x.has_velocity_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.has_acceleration_limits")) {
            updated_params.linear.x.has_acceleration_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.has_jerk_limits")) {
            updated_params.linear.x.has_jerk_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.max_velocity")) {
            updated_params.linear.x.max_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.min_velocity")) {
            updated_params.linear.x.min_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.max_acceleration")) {
            updated_params.linear.x.max_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.min_acceleration")) {
            updated_params.linear.x.min_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.max_jerk")) {
            updated_params.linear.x.max_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.x.min_jerk")) {
            updated_params.linear.x.min_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.has_velocity_limits")) {
            updated_params.linear.y.has_velocity_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.has_acceleration_limits")) {
            updated_params.linear.y.has_acceleration_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.has_jerk_limits")) {
            updated_params.linear.y.has_jerk_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.max_velocity")) {
            updated_params.linear.y.max_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.min_velocity")) {
            updated_params.linear.y.min_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.max_acceleration")) {
            updated_params.linear.y.max_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.min_acceleration")) {
            updated_params.linear.y.min_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.max_jerk")) {
            updated_params.linear.y.max_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "linear.y.min_jerk")) {
            updated_params.linear.y.min_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.has_velocity_limits")) {
            updated_params.angular.z.has_velocity_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.has_acceleration_limits")) {
            updated_params.angular.z.has_acceleration_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.has_jerk_limits")) {
            updated_params.angular.z.has_jerk_limits = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.max_velocity")) {
            updated_params.angular.z.max_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.min_velocity")) {
            updated_params.angular.z.min_velocity = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.max_acceleration")) {
            updated_params.angular.z.max_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.min_acceleration")) {
            updated_params.angular.z.min_acceleration = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.max_jerk")) {
            updated_params.angular.z.max_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "angular.z.min_jerk")) {
            updated_params.angular.z.min_jerk = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cmd_vel_timeout")) {
            updated_params.cmd_vel_timeout = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      if (user_callback_) {
         user_callback_(updated_params);
      }
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "reference_timeout")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Timeout for controller references after which they will be reset. This is especially useful for controllers that can cause unwanted and dangerous behaviour if reference is not reset, e.g., velocity controllers. If value is 0 the reference is reset after each run.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.reference_timeout);
          parameters_interface_->declare_parameter(prefix_ + "reference_timeout", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steer_joints_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Names of wheel steer joints.";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "steer_joints_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "drive_joints_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Names of wheel drive joints.";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "drive_joints_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steer_joints_state_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Names of states of wheel steer joints.";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "steer_joints_state_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "drive_joints_state_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Names of states of wheel drive joints.";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "drive_joints_state_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "use_stamped_vel")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "bool parameter decides cmd_vel is stamped or not.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.use_stamped_vel);
          parameters_interface_->declare_parameter(prefix_ + "use_stamped_vel", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "open_loop")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "bool parameter decides if open oop or not (feedback).";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.open_loop);
          parameters_interface_->declare_parameter(prefix_ + "open_loop", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "velocity_rolling_window_size")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "The number of velocity samples to average together to compute the odometry twist.linear.x and twist.angular.z velocities.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.velocity_rolling_window_size);
          parameters_interface_->declare_parameter(prefix_ + "velocity_rolling_window_size", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "base_frame_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Base frame_id set to value of base_frame_id.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.base_frame_id);
          parameters_interface_->declare_parameter(prefix_ + "base_frame_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "odom_frame_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Odometry frame_id set to value of odom_frame_id.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.odom_frame_id);
          parameters_interface_->declare_parameter(prefix_ + "odom_frame_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "enable_odom_tf")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Publishing to tf is enabled or disabled ?.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.enable_odom_tf);
          parameters_interface_->declare_parameter(prefix_ + "enable_odom_tf", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "twist_covariance_diagonal")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "diagonal values of twist covariance matrix.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.twist_covariance_diagonal);
          parameters_interface_->declare_parameter(prefix_ + "twist_covariance_diagonal", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "pose_covariance_diagonal")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "diagonal values of pose covariance matrix.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.pose_covariance_diagonal);
          parameters_interface_->declare_parameter(prefix_ + "pose_covariance_diagonal", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "position_feedback")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Choice of feedback type, if position_feedback is false then HW_IF_VELOCITY is taken as interface type, if position_feedback is true then HW_IF_POSITION is taken as interface type";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.position_feedback);
          parameters_interface_->declare_parameter(prefix_ + "position_feedback", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "max_steering_limit")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Max joint angle for the steering joint";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.max_steering_limit);
          parameters_interface_->declare_parameter(prefix_ + "max_steering_limit", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "min_steering_limit")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Min joint angle for the steering joint";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.min_steering_limit);
          parameters_interface_->declare_parameter(prefix_ + "min_steering_limit", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_track")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Wheel track length. For details see: https://en.wikipedia.org/wiki/Wheelbase";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.wheel_track);
          parameters_interface_->declare_parameter(prefix_ + "wheel_track", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheelbase")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Distance between front and rear wheels. For details see: https://en.wikipedia.org/wiki/Wheelbase";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.wheelbase);
          parameters_interface_->declare_parameter(prefix_ + "wheelbase", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_radius")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Wheels radius.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.wheel_radius);
          parameters_interface_->declare_parameter(prefix_ + "wheel_radius", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "drive_to_steer_offset")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Distance from steer joint to drive joint.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.drive_to_steer_offset);
          parameters_interface_->declare_parameter(prefix_ + "drive_to_steer_offset", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "publish_limited_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Choice of whether to publish limited velocities as a message";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.publish_limited_velocity);
          parameters_interface_->declare_parameter(prefix_ + "publish_limited_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.has_velocity_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.has_velocity_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.has_velocity_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.has_acceleration_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.has_acceleration_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.has_acceleration_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.has_jerk_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.has_jerk_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.has_jerk_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.max_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.max_velocity);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.max_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.min_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.min_velocity);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.min_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.max_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.max_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.max_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.min_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.min_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.min_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.max_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.max_jerk);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.max_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.x.min_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.x.min_jerk);
          parameters_interface_->declare_parameter(prefix_ + "linear.x.min_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.has_velocity_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.has_velocity_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.has_velocity_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.has_acceleration_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.has_acceleration_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.has_acceleration_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.has_jerk_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.has_jerk_limits);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.has_jerk_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.max_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.max_velocity);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.max_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.min_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.min_velocity);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.min_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.max_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.max_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.max_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.min_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.min_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.min_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.max_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.max_jerk);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.max_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "linear.y.min_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.linear.y.min_jerk);
          parameters_interface_->declare_parameter(prefix_ + "linear.y.min_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.has_velocity_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.has_velocity_limits);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.has_velocity_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.has_acceleration_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.has_acceleration_limits);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.has_acceleration_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.has_jerk_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.has_jerk_limits);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.has_jerk_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.max_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.max_velocity);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.max_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.min_velocity")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.min_velocity);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.min_velocity", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.max_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.max_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.max_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.min_acceleration")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.min_acceleration);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.min_acceleration", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.max_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.max_jerk);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.max_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "angular.z.min_jerk")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.angular.z.min_jerk);
          parameters_interface_->declare_parameter(prefix_ + "angular.z.min_jerk", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cmd_vel_timeout")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Timeout after which input command on cmd_vel topic is considered staled.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.cmd_vel_timeout);
          parameters_interface_->declare_parameter(prefix_ + "cmd_vel_timeout", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "reference_timeout");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "reference_timeout") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.reference_timeout = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "steer_joints_names");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "steer_joints_names") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steer_joints_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steer_joints_names': {}", validation_result.error()));
      }
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steer_joints_names': {}", validation_result.error()));
      }
      updated_params.steer_joints_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "drive_joints_names");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "drive_joints_names") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'drive_joints_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'drive_joints_names': {}", validation_result.error()));
      }
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'drive_joints_names': {}", validation_result.error()));
      }
      updated_params.drive_joints_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "steer_joints_state_names");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "steer_joints_state_names") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steer_joints_state_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steer_joints_state_names': {}", validation_result.error()));
      }
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steer_joints_state_names': {}", validation_result.error()));
      }
      updated_params.steer_joints_state_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "drive_joints_state_names");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "drive_joints_state_names") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = size_lt<std::string>(param, 5);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'drive_joints_state_names': {}", validation_result.error()));
      }
      if(auto validation_result = unique<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'drive_joints_state_names': {}", validation_result.error()));
      }
      if(auto validation_result = not_empty<std::string>(param);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'drive_joints_state_names': {}", validation_result.error()));
      }
      updated_params.drive_joints_state_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "use_stamped_vel");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "use_stamped_vel") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.use_stamped_vel = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "open_loop");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "open_loop") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.open_loop = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "velocity_rolling_window_size");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "velocity_rolling_window_size") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.velocity_rolling_window_size = param.as_int();
      param = parameters_interface_->get_parameter(prefix_ + "base_frame_id");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "base_frame_id") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.base_frame_id = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "odom_frame_id");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "odom_frame_id") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.odom_frame_id = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "enable_odom_tf");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "enable_odom_tf") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.enable_odom_tf = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "twist_covariance_diagonal");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "twist_covariance_diagonal") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.twist_covariance_diagonal = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "pose_covariance_diagonal");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "pose_covariance_diagonal") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.pose_covariance_diagonal = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "position_feedback");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "position_feedback") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.position_feedback = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "max_steering_limit");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "max_steering_limit") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.max_steering_limit = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "min_steering_limit");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "min_steering_limit") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.min_steering_limit = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_track");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "wheel_track") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheel_track = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "wheelbase");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "wheelbase") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheelbase = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_radius");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "wheel_radius") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheel_radius = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "drive_to_steer_offset");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "drive_to_steer_offset") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.drive_to_steer_offset = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "publish_limited_velocity");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "publish_limited_velocity") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.publish_limited_velocity = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.has_velocity_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.has_velocity_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.has_velocity_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.has_acceleration_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.has_acceleration_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.has_acceleration_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.has_jerk_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.has_jerk_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.has_jerk_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.max_velocity");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.max_velocity") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.max_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.min_velocity");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.min_velocity") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.min_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.max_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.max_acceleration") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.max_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.min_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.min_acceleration") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.min_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.max_jerk");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.max_jerk") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.max_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.x.min_jerk");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.x.min_jerk") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.x.min_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.has_velocity_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.has_velocity_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.has_velocity_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.has_acceleration_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.has_acceleration_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.has_acceleration_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.has_jerk_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.has_jerk_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.has_jerk_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.max_velocity");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.max_velocity") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.max_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.min_velocity");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.min_velocity") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.min_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.max_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.max_acceleration") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.max_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.min_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.min_acceleration") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.min_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.max_jerk");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.max_jerk") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.max_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "linear.y.min_jerk");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "linear.y.min_jerk") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.linear.y.min_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.has_velocity_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.has_velocity_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.has_velocity_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.has_acceleration_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.has_acceleration_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.has_acceleration_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.has_jerk_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.has_jerk_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.has_jerk_limits = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.max_velocity");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.max_velocity") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.max_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.min_velocity");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.min_velocity") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.min_velocity = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.max_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.max_acceleration") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.max_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.min_acceleration");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.min_acceleration") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.min_acceleration = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.max_jerk");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.max_jerk") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.max_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "angular.z.min_jerk");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "angular.z.min_jerk") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.angular.z.min_jerk = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "cmd_vel_timeout");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "cmd_vel_timeout") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.cmd_vel_timeout = param.as_double();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    using userParameterUpdateCB = std::function<void(const Params&)>;
    void setUserCallback(const userParameterUpdateCB& callback){
      user_callback_ = callback;
    }

    void clearUserCallback(){
      user_callback_ = {};
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
      userParameterUpdateCB user_callback_;

      rclcpp::Logger logger_;
      std::mutex mutable mutex_;
  };

} // namespace swerve_controller
