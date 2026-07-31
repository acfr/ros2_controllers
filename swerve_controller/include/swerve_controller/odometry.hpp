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

#ifndef SWERVE_CONTROLLER__ODOMETRY_HPP_
#define SWERVE_CONTROLLER__ODOMETRY_HPP_

#include <tuple>
#include <vector>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

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
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param time    Current time
     */
    void update_open_loop(const double linear_x, const double linear_y, const double angular, const double dt);

    /**
     * \brief Updates the odometry class with the measured per-wheel velocity vectors
     * (closed loop). Each wheel's velocity vector is fit against the wheel's known
     * position relative to the robot origin to recover the robot's body-frame
     * linear (x, y) and angular velocity via least squares, which is then integrated.
     * \param drive_speed_vector Measured velocity vector [vx, vy] of each wheel, in the base frame [m/s]
     * \param wheel_centres      Position of each wheel centre relative to the robot origin [m]
     * \param dt                 Time since the last update [s]
     * \return true if the update succeeded (sizes matched and were non-empty), false otherwise
     */
    bool update_odometry(const std::vector<Eigen::Vector2d> & drive_speed_vector,
                          const std::vector<Eigen::Vector2d> & wheel_centres,
                          const double dt);

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