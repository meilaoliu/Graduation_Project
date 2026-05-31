#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <type_traits>
#include <vector>

#include "traj_utils/planning_visualization.h"

namespace {

TEST(PlanningVisualization, ExposesGlobalWaypointListDisplayApi)
{
  using DisplayFn = void (ego_planner::PlanningVisualization::*)(
      const std::vector<Eigen::Vector3d> &, const double, int);

  DisplayFn fn = &ego_planner::PlanningVisualization::displayGlobalWaypointList;
  EXPECT_TRUE(std::is_member_function_pointer<DisplayFn>::value);
  EXPECT_NE(fn, nullptr);
}

TEST(PlanningVisualization, GeneratesPointDisplayArrayWithoutConnectingLine)
{
  ego_planner::PlanningVisualization visualization;
  visualization_msgs::MarkerArray array;
  std::vector<Eigen::Vector3d> points{
      Eigen::Vector3d(1.0, 2.0, 0.1),
      Eigen::Vector3d(3.0, 4.0, 0.1),
      Eigen::Vector3d(5.0, 6.0, 0.1)};

  visualization.generatePointDisplayArray(
      array, points, 0.25, Eigen::Vector4d(1.0, 0.55, 0.0, 1.0), 7,
      "global_waypoint_list");

  ASSERT_EQ(array.markers.size(), 1u);
  EXPECT_EQ(array.markers[0].type, visualization_msgs::Marker::SPHERE_LIST);
  EXPECT_NE(array.markers[0].type, visualization_msgs::Marker::LINE_STRIP);
  EXPECT_EQ(array.markers[0].ns, "global_waypoint_list");
  EXPECT_EQ(array.markers[0].id, 7);
  EXPECT_EQ(array.markers[0].points.size(), points.size());
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
