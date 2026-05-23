#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

#include "plan_manage/fsm_heading_utils.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(FsmHeadingUtils, SkipsDuplicateStartAndFindsFirstSegmentYaw)
{
  const Eigen::Vector3d odom(-2.0, 1.0, 0.85);
  const std::vector<Eigen::Vector3d> wps = {
      Eigen::Vector3d(-2.02, 1.01, 0.85),
      Eigen::Vector3d(-6.0, 3.0, 0.85),
      Eigen::Vector3d(-8.0, 8.0, 0.85),
  };

  double yaw = 0.0;
  ASSERT_TRUE(ego_planner::firstDistinctWaypointYaw2d(wps, odom, 0.30, &yaw));

  EXPECT_NEAR(yaw, std::atan2(2.0, -4.0), 1e-9);
}

TEST(FsmHeadingUtils, DetectsNeedToAlignBeforeForwardOnlySegment)
{
  const double current_yaw = -0.5 * kPi;
  const double segment_yaw = std::atan2(2.0, -4.0);

  EXPECT_TRUE(ego_planner::needsYawAlignment(segment_yaw, current_yaw, 20.0 / 180.0 * kPi));
  EXPECT_FALSE(ego_planner::needsYawAlignment(segment_yaw, segment_yaw + 5.0 / 180.0 * kPi,
                                             20.0 / 180.0 * kPi));
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
