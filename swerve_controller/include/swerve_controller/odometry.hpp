// Copyright 2023 Australian Centre For Robotics
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

/*
 * Author: Jerome Justin
 */

#ifndef SWERVE_CONTROLLER__ODOMETRY_HPP_
#define SWERVE_CONTROLLER__ODOMETRY_HPP_

#include <tuple>
#include <vector>
#include <cmath>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace swerve_controller
{
  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:
    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     *
     */
    explicit Odometry();

    /**
     * \brief Updates the odometry class with latest wheels position
     * \param drive_joints_values  Drive joint positions vector contains values in [rad]
     * \param steer_joints_values Steer joint position vector contains values in [rad]
     * \param dt      time difference to last call
     * \return true if the odometry is actually updated
     */
    bool update_from_position(const std::vector<double> drive_joints_values,
                              const std::vector<double> steer_joints_values, const double dt);

    bool update_from_velocity(const std::vector<double> drive_joints_values,
                              const std::vector<double> steer_joints_values, const double dt);

    /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param time    Current time
     */
    void update_open_loop(const double linear_x, const double linear_y, const double angular, const double dt);

    /**
     * \brief Set odometry type
     * \param type odometry type
     */
    void set_odometry_type(const unsigned int type);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double get_heading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double get_x() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double get_y() const
    {
      return y_;
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double get_linear_x() const
    {
      return linear_x_;
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double get_linear_y() const
    {
      return linear_y_;
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity as a tuple in x and y directions [m/s]
     */
    std::tuple<double, double> get_linear() const
    {
      return std::make_tuple(linear_x_, linear_y_);
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double get_angular() const
    {
      return angular_;
    }

    /**
     * \brief Sets the wheel parameters: radius, separation and drive to steer joint offset
     */
    void set_wheel_params(double wheel_radius, double wheel_base, double wheel_track, double offset);

    /**
     *  \brief Reset poses, heading, and accumulators
     */
    void reset_odometry();

  private:
    /**
     * \brief Uses precomputed linear and angular velocities to compute dometry and update
     * accumulators
     */
    bool update_odometry(const double &fl_speed, const double &fr_speed, const double &rr_speed, const double &rl_speed,
                         const double &fl_steering, const double &fr_steering, const double &rr_steering,
                         const double &rl_steering, const double dt);

    void integrateXY(double linear_x, double linear_y, double angular);

    /// Current pose:
    double x_;         //   [m]
    double y_;         //   [m]
    double steer_pos_; // [rad]
    double heading_;   // [rad]

    /// Current velocity:
    double linear_x_; //   [m/s]
    double linear_y_; //   [m/s]
    double angular_;  // [rad/s]

    /// Kinematic parameters
    double wheel_track_; // [m]
    double wheelbase_;   // [m]
    double wheel_radius_;
    double drive_to_steer_offset_; // [m]
    double steering_track_;        // [m]
  };
} // namespace swerve_controller
#endif // SWERVE_CONTROLLER__ODOMETRY_HPP_
