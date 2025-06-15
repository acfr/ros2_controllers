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

#include "swerve_controller/odometry.hpp"

namespace swerve_controller
{
  Odometry::Odometry()
      : x_(0.0), y_(0.0),
        heading_(0.0),
        linear_x_(0.0),
        linear_y_(0.0),
        angular_(0.0),
        wheel_track_(0.0),
        wheelbase_(0.0),
        wheel_radius_(0.0),
        steering_track_(0.0)
  {
  }

  bool Odometry::update_odometry(const double &fl_speed, const double &fr_speed, const double &rr_speed,
                                 const double &rl_speed, const double &fl_steering, const double &fr_steering,
                                 const double &rr_steering, const double &rl_steering, const double dt)
  {
    // Compute velocity vectors in X-Y for each wheel
    const double fl_speed_x = sin(fl_steering) * fl_speed * wheel_radius_;
    const double fl_speed_y = cos(fl_steering) * fl_speed * wheel_radius_;
    const double fr_speed_x = sin(fr_steering) * fr_speed * wheel_radius_;
    const double fr_speed_y = cos(fr_steering) * fr_speed * wheel_radius_;
    const double rr_speed_x = sin(rr_steering) * rr_speed * wheel_radius_;
    const double rr_speed_y = cos(rr_steering) * rr_speed * wheel_radius_;
    const double rl_speed_x = sin(rl_steering) * rl_speed * wheel_radius_;
    const double rl_speed_y = cos(rl_steering) * rl_speed * wheel_radius_;

    // Compute robot velocities components
    const double a = (rl_speed_x + rr_speed_x) / 2.0;
    const double b = (fr_speed_x + fl_speed_x) / 2.0;
    const double c = (fr_speed_y + rl_speed_y) / 2.0;
    const double d = (fl_speed_y + rr_speed_y) / 2.0;

    // Average angular speed
    const double angular_1 = (b - a) / wheelbase_;
    const double angular_2 = (c - d) / steering_track_;
    angular_ = (angular_1 + angular_2) / 2.0;

    // Average linear speed
    const double linear_x_1 = angular_ * (wheelbase_ / 2.0) + c;
    const double linear_x_2 = -angular_ * (wheelbase_ / 2.0) + d;
    const double linear_y_1 = angular_ * (steering_track_ / 2.0) + a;
    const double linear_y_2 = -angular_ * (steering_track_ / 2.0) + b;

    linear_x_ = (linear_x_1 + linear_x_2) / 2.0;
    linear_y_ = (linear_y_1 + linear_y_2) / 2.0;

    integrateXY(linear_x_ * dt, linear_y_ * dt, angular_ * dt);

    return true;
  }

  bool Odometry::update_from_position(const std::vector<double> drive_joints_values,
                                      const std::vector<double> steer_joints_values, const double dt)
  {
    // Get current position and velocities
    const double fl_speed = drive_joints_values[0] / dt;
    const double fr_speed = drive_joints_values[1] / dt;
    const double rl_speed = drive_joints_values[2] / dt;
    const double rr_speed = drive_joints_values[3] / dt;

    if (std::isnan(fl_speed) || std::isnan(fr_speed) || std::isnan(rl_speed) || std::isnan(rr_speed))
      return false;

    const double fl_steering = steer_joints_values[0];
    const double fr_steering = steer_joints_values[1];
    const double rl_steering = steer_joints_values[2];
    const double rr_steering = steer_joints_values[3];

    if (std::isnan(fl_steering) || std::isnan(fr_steering) || std::isnan(rl_steering) || std::isnan(rr_steering))
      return false;

    // Estimate linear and angular velocity using joint information
    return update_odometry(fl_speed, fr_speed, rl_speed, rr_speed, fl_steering, fr_steering, rl_steering, rr_steering, dt);
  }

  bool Odometry::update_from_velocity(const std::vector<double> drive_joints_values,
                                      const std::vector<double> steer_joints_values, const double dt)
  {
    // Get current position and velocities
    const double fl_speed = drive_joints_values[0];
    const double fr_speed = drive_joints_values[1];
    const double rl_speed = drive_joints_values[2];
    const double rr_speed = drive_joints_values[3];

    const double fl_steering = steer_joints_values[0];
    const double fr_steering = steer_joints_values[1];
    const double rl_steering = steer_joints_values[2];
    const double rr_steering = steer_joints_values[3];

    // Estimate linear and angular velocity using joint information
    return update_odometry(fl_speed, fr_speed, rl_speed, rr_speed, fl_steering, fr_steering, rl_steering, rr_steering, dt);
  }

  void Odometry::update_open_loop(double linear_x, double linear_y, const double angular, const double dt)
  {
    /// Save last linear and angular velocity:
    linear_x_ = linear_x;
    linear_y_ = linear_y;
    angular_ = angular;

    // Integrate odometry
    integrateXY(linear_x_ * dt, linear_y_ * dt, angular_ * dt);
  }

  void Odometry::set_wheel_params(double wheel_radius, double wheel_base, double wheel_track, double offset)
  {
    wheel_radius_ = wheel_radius;
    wheelbase_ = wheel_base;
    wheel_track_ = wheel_track;
    drive_to_steer_offset_ = offset;
    steering_track_ = wheel_track - 2 * offset;
  }

  void Odometry::reset_odometry()
  {
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
  }

  void Odometry::integrateXY(double linear_x, double linear_y, double angular)
  {
    const double delta_x = linear_x * cos(heading_) - linear_y * sin(heading_);
    const double delta_y = linear_x * sin(heading_) + linear_y * cos(heading_);

    x_ += delta_x;
    y_ += delta_y;
    heading_ += angular;
  }
} // namespace swerve_controller
