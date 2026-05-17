#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <vector>

#include "plan_manage/ground_safety_utils.h"

namespace {

TEST(GroundSafetyUtils, ProjectsOccupiedGoalUsingRuntime2dOccupancy)
{
  const Eigen::Vector3d requested(0.0, 0.0, 0.7123);
  Eigen::Vector3d projected = Eigen::Vector3d::Zero();

  const auto occupancy = [](const Eigen::Vector2d &pt) {
    return pt.norm() < 0.45 ? 1 : 0;
  };

  ASSERT_TRUE(ego_planner::projectToNearestFree2d(requested, occupancy, &projected, 1.0, 0.15));
  EXPECT_GE(projected.head<2>().norm(), 0.45);
  EXPECT_DOUBLE_EQ(projected.z(), requested.z());
}

TEST(GroundSafetyUtils, SkipsBoundaryFreeCellsWithoutClearance)
{
  const Eigen::Vector3d requested(0.0, 0.0, 0.7123);
  Eigen::Vector3d projected = Eigen::Vector3d::Zero();

  const auto occupancy = [](const Eigen::Vector2d &pt) {
    return pt.norm() < 0.45 ? 1 : 0;
  };

  ASSERT_TRUE(ego_planner::projectToNearestFree2d(requested, occupancy, &projected, 1.2, 0.15, 0.30, 0.15));
  EXPECT_GE(projected.head<2>().norm(), 0.75);
  EXPECT_DOUBLE_EQ(projected.z(), requested.z());
}

TEST(GroundSafetyUtils, RejectsUnknownCellsWhenProjectingGoal)
{
  const Eigen::Vector3d requested(0.0, 0.0, 0.7123);
  Eigen::Vector3d projected = Eigen::Vector3d::Zero();

  const auto occupancy = [](const Eigen::Vector2d &) {
    return -1;
  };

  EXPECT_FALSE(ego_planner::projectToNearestFree2d(requested, occupancy, &projected, 0.6, 0.15));
}

TEST(GroundSafetyUtils, DetectsBlockedPointsInExecutionWindow)
{
  const std::vector<std::pair<double, Eigen::Vector3d>> points = {
      {0.2, Eigen::Vector3d(0.0, 0.0, 0.7)},
      {0.8, Eigen::Vector3d(1.0, 0.0, 0.7)},
      {1.6, Eigen::Vector3d(2.0, 0.0, 0.7)},
  };

  const auto occupancy = [](const Eigen::Vector2d &pt) {
    return pt.x() > 0.9 && pt.x() < 1.1 ? 1 : 0;
  };

  EXPECT_FALSE(ego_planner::isCollisionFreeWindow2d(points, occupancy, 0.5, 1.2));
  EXPECT_TRUE(ego_planner::isCollisionFreeWindow2d(points, occupancy, 1.0, 2.0));
}

TEST(GroundSafetyUtils, RejectsExecutionWindowWithoutSamples)
{
  const std::vector<std::pair<double, Eigen::Vector3d>> points = {
      {0.2, Eigen::Vector3d(0.0, 0.0, 0.7)},
      {0.8, Eigen::Vector3d(1.0, 0.0, 0.7)},
  };

  const auto occupancy = [](const Eigen::Vector2d &) {
    return 0;
  };

  EXPECT_FALSE(ego_planner::isCollisionFreeWindow2d(points, occupancy, 1.0, 2.0));
}

TEST(GroundSafetyUtils, AllowsContinuingWhenBrakingSafetyWindowIsFree)
{
  const std::vector<std::pair<double, Eigen::Vector3d>> points = {
      {0.2, Eigen::Vector3d(0.2, 0.0, 0.7)},
      {0.8, Eigen::Vector3d(0.8, 0.0, 0.7)},
      {1.5, Eigen::Vector3d(1.5, 0.0, 0.7)},
      {2.4, Eigen::Vector3d(2.4, 0.0, 0.7)},
  };

  const auto occupancy = [](const Eigen::Vector2d &pt) {
    return pt.x() > 2.8 && pt.x() < 3.2 ? 1 : 0;
  };

  EXPECT_TRUE(ego_planner::isSafeToContinueDuringReplan2d(points, occupancy, 0.0, 1.8));
}

TEST(GroundSafetyUtils, RejectsContinuingWhenBrakingSafetyWindowIsBlocked)
{
  const std::vector<std::pair<double, Eigen::Vector3d>> points = {
      {0.2, Eigen::Vector3d(0.2, 0.0, 0.7)},
      {0.8, Eigen::Vector3d(0.8, 0.0, 0.7)},
      {1.5, Eigen::Vector3d(1.5, 0.0, 0.7)},
      {2.4, Eigen::Vector3d(2.4, 0.0, 0.7)},
  };

  const auto occupancy = [](const Eigen::Vector2d &pt) {
    return pt.x() > 1.3 && pt.x() < 1.7 ? 1 : 0;
  };

  EXPECT_FALSE(ego_planner::isSafeToContinueDuringReplan2d(points, occupancy, 0.0, 1.8));
}

TEST(GroundSafetyUtils, ComputesControlledStopTargetWithFixedZ)
{
  const Eigen::Vector3d pos(1.0, 2.0, 0.7);
  const Eigen::Vector3d vel(2.0, 0.0, 0.5);
  Eigen::Vector3d target = Eigen::Vector3d::Zero();

  ASSERT_TRUE(ego_planner::computeControlledStopTarget2d(pos, vel, 1.0, &target));

  EXPECT_NEAR(target.x(), 3.0, 1e-9);
  EXPECT_NEAR(target.y(), 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(target.z(), 0.7);
}

TEST(GroundSafetyUtils, RejectsBlockedControlledStopPath)
{
  const Eigen::Vector3d pos(0.0, 0.0, 0.7);
  const Eigen::Vector3d vel(2.0, 0.0, 0.0);
  const auto occupancy = [](const Eigen::Vector2d &pt) {
    return pt.x() > 0.8 && pt.x() < 1.2 ? 1 : 0;
  };

  EXPECT_FALSE(ego_planner::isControlledStopPathFree2d(pos, vel, 1.0, occupancy, 0.2));
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
