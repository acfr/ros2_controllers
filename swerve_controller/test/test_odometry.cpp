#include <gmock/gmock.h>

#include <vector>

#include "swerve_controller/odometry.hpp"

namespace
{
const Eigen::Vector3d kFloor(0.001, 0.001, 0.01);
constexpr double kMaxVariance = 1.0;
constexpr double kTolerance = 1e-9;

// Wheel centres for a 0.505 m wheelbase, 0.62 m track robot
std::vector<Eigen::Vector2d> wheel_centres()
{
  return {{0.2525, 0.31}, {0.2525, -0.31}, {-0.2525, 0.31}, {-0.2525, -0.31}};
}

swerve_controller::Odometry make_odometry()
{
  swerve_controller::Odometry odometry;
  odometry.set_covariance_params(kFloor, 1.0, kMaxVariance);
  return odometry;
}
}  // namespace

TEST(TestOdometry, consistent_wheels_give_floor_covariance)
{
  auto odometry = make_odometry();
  const std::vector<Eigen::Vector2d> speeds(4, Eigen::Vector2d(0.5, 0.0));

  ASSERT_TRUE(odometry.update_odometry(speeds, wheel_centres(), 0.01));

  const Eigen::Matrix3d & covariance = odometry.get_twist_covariance();
  EXPECT_TRUE(covariance.isApprox(Eigen::Matrix3d(kFloor.asDiagonal()), kTolerance));
}

TEST(TestOdometry, consistent_rotation_gives_floor_covariance)
{
  auto odometry = make_odometry();
  const double omega = 0.5;
  std::vector<Eigen::Vector2d> speeds;
  for (const auto & centre : wheel_centres())
  {
    speeds.emplace_back(-omega * centre.y(), omega * centre.x());
  }

  ASSERT_TRUE(odometry.update_odometry(speeds, wheel_centres(), 0.01));

  EXPECT_NEAR(odometry.get_angular(), omega, kTolerance);
  const Eigen::Matrix3d & covariance = odometry.get_twist_covariance();
  EXPECT_TRUE(covariance.isApprox(Eigen::Matrix3d(kFloor.asDiagonal()), kTolerance));
}

TEST(TestOdometry, slipping_wheel_raises_covariance)
{
  auto odometry = make_odometry();
  std::vector<Eigen::Vector2d> speeds(4, Eigen::Vector2d(0.5, 0.0));
  speeds[0] = Eigen::Vector2d(0.25, 0.0);

  ASSERT_TRUE(odometry.update_odometry(speeds, wheel_centres(), 0.01));

  const Eigen::Matrix3d & covariance = odometry.get_twist_covariance();
  EXPECT_GT(covariance(0, 0), kFloor(0) + 1e-4);
  EXPECT_GT(covariance(2, 2), kFloor(2) + 1e-4);
  EXPECT_LE(covariance.diagonal().maxCoeff(), kMaxVariance);
}

TEST(TestOdometry, open_loop_gives_floor_covariance)
{
  auto odometry = make_odometry();
  odometry.update_open_loop(0.5, 0.0, 0.0, 0.01);

  const Eigen::Matrix3d & covariance = odometry.get_twist_covariance();
  EXPECT_TRUE(covariance.isApprox(Eigen::Matrix3d(kFloor.asDiagonal()), kTolerance));
}
