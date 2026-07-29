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

  bool Odometry::update_odometry(const std::vector<Eigen::Vector2d> & drive_speed_vector,
                                  const std::vector<Eigen::Vector2d> & wheel_centres,
                                  const double dt)
  {
    const size_t num_wheels = drive_speed_vector.size();

    if (num_wheels == 0 || num_wheels != wheel_centres.size())
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("swerve_controller_odometry"),
          "update_odometry: drive_speed_vector (%zu) and wheel_centres (%zu) must be the "
          "same non-zero size",
          num_wheels, wheel_centres.size());
      return false;
    }

    // Each wheel's measured velocity vector v_i (expressed in the base frame) is related
    // to the robot's body-frame velocity [vx, vy] and yaw rate omega by the rigid-body
    // constraint:
    //
    //   v_i = [vx, vy] + omega * [-y_i, x_i]
    //
    // where (x_i, y_i) is that wheel's centre position relative to the robot origin.
    // Stacking every wheel gives an overdetermined linear system A * [vx, vy, omega]^T = b,
    // which is solved in a least-squares sense so that noisy/redundant wheel measurements
    // are fused into a single best-fit body velocity.
    Eigen::MatrixXd A(2 * num_wheels, 3);
    Eigen::VectorXd b(2 * num_wheels);

    for (size_t i = 0; i < num_wheels; ++i)
    {
      const double x_i = wheel_centres[i].x();
      const double y_i = wheel_centres[i].y();

      A(2 * i, 0) = 1.0;
      A(2 * i, 1) = 0.0;
      A(2 * i, 2) = -y_i;
      b(2 * i) = drive_speed_vector[i].x();

      A(2 * i + 1, 0) = 0.0;
      A(2 * i + 1, 1) = 1.0;
      A(2 * i + 1, 2) = x_i;
      b(2 * i + 1) = drive_speed_vector[i].y();
    }

    // JacobiSVD handles rank-deficient cases gracefully (e.g. a single wheel, or wheel
    // centres that are collinear and therefore can't fully observe rotation).
    const Eigen::VectorXd solution = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    linear_x_ = solution(0);
    linear_y_ = solution(1);
    angular_ = solution(2);

    integrateXY(linear_x_ * dt, linear_y_ * dt, angular_ * dt);

    return true;
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