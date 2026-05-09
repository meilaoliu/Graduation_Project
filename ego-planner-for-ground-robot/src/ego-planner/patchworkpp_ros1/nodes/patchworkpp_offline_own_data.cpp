#define PCL_NO_PRECOMPILE

#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "patchwork/patchworkpp.h"

using PointType = pcl::PointXYZ;

namespace {

ros::Publisher cloud_pub;
ros::Publisher ground_pub;
ros::Publisher nonground_pub;

std::mutex odom_mutex;
double robot_x = 0.0;
double robot_y = 0.0;
double robot_z = 0.0;
Eigen::Quaterniond robot_q = Eigen::Quaterniond::Identity();
bool has_odom = false;

std::unique_ptr<patchwork::PatchWorkpp> patchworkpp;
double self_filter_radius = 0.0;
double sensor_height = 0.75;
double min_obstacle_height = 0.18;

void odomCallback(const nav_msgs::OdometryConstPtr &odom) {
  std::lock_guard<std::mutex> lock(odom_mutex);
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_z = odom->pose.pose.position.z;
  robot_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                               odom->pose.pose.orientation.x,
                               odom->pose.pose.orientation.y,
                               odom->pose.pose.orientation.z);
  has_odom = true;
}

template <typename T>
T getParam(const ros::NodeHandle &nh, const std::string &name, const T &fallback) {
  T value;
  nh.param(name, value, fallback);
  return value;
}

std::vector<int> getIntVectorParam(const ros::NodeHandle &nh,
                                   const std::string &name,
                                   const std::vector<int> &fallback) {
  std::vector<int> value;
  if (!nh.getParam(name, value)) {
    return fallback;
  }
  return value;
}

std::vector<double> getDoubleVectorParam(const ros::NodeHandle &nh,
                                         const std::string &name,
                                         const std::vector<double> &fallback) {
  std::vector<double> value;
  if (!nh.getParam(name, value)) {
    return fallback;
  }
  return value;
}

patchwork::Params loadParams(const ros::NodeHandle &nh) {
  patchwork::Params params;
  params.sensor_height = getParam(nh, "patchworkpp/sensor_height", params.sensor_height);
  params.verbose = getParam(nh, "patchworkpp/verbose", params.verbose);

  params.enable_RNR = getParam(nh, "patchworkpp/enable_RNR", params.enable_RNR);
  params.enable_RVPF = getParam(nh, "patchworkpp/enable_RVPF", params.enable_RVPF);
  params.enable_TGR = getParam(nh, "patchworkpp/enable_TGR", params.enable_TGR);

  params.num_iter = getParam(nh, "patchworkpp/num_iter", params.num_iter);
  params.num_lpr = getParam(nh, "patchworkpp/num_lpr", params.num_lpr);
  params.num_min_pts = getParam(nh, "patchworkpp/num_min_pts", params.num_min_pts);
  params.num_rings_of_interest =
      getParam(nh, "patchworkpp/num_rings_of_interest", params.num_rings_of_interest);

  params.th_seeds = getParam(nh, "patchworkpp/th_seeds", params.th_seeds);
  params.th_dist = getParam(nh, "patchworkpp/th_dist", params.th_dist);
  params.th_seeds_v = getParam(nh, "patchworkpp/th_seeds_v", params.th_seeds_v);
  params.th_dist_v = getParam(nh, "patchworkpp/th_dist_v", params.th_dist_v);

  params.max_range = getParam(nh, "patchworkpp/max_range", params.max_range);
  params.min_range = getParam(nh, "patchworkpp/min_range", params.min_range);
  params.uprightness_thr =
      getParam(nh, "patchworkpp/uprightness_thr", params.uprightness_thr);
  params.adaptive_seed_selection_margin = getParam(
      nh, "patchworkpp/adaptive_seed_selection_margin", params.adaptive_seed_selection_margin);

  params.RNR_ver_angle_thr =
      getParam(nh, "patchworkpp/RNR_ver_angle_thr", params.RNR_ver_angle_thr);
  params.RNR_intensity_thr =
      getParam(nh, "patchworkpp/RNR_intensity_thr", params.RNR_intensity_thr);

  params.num_sectors_each_zone =
      getIntVectorParam(nh, "patchworkpp/num_sectors_each_zone", params.num_sectors_each_zone);
  params.num_rings_each_zone =
      getIntVectorParam(nh, "patchworkpp/num_rings_each_zone", params.num_rings_each_zone);
  params.elevation_thr =
      getDoubleVectorParam(nh, "patchworkpp/elevation_thr", params.elevation_thr);
  params.flatness_thr =
      getDoubleVectorParam(nh, "patchworkpp/flatness_thr", params.flatness_thr);

  return params;
}

Eigen::MatrixX3f filterNongroundByHeight(const Eigen::MatrixX3f &nonground,
                                         Eigen::MatrixX3f *low_points) {
  const float obstacle_z_threshold = static_cast<float>(-sensor_height + min_obstacle_height);
  std::vector<int> obstacle_indices;
  std::vector<int> low_indices;
  obstacle_indices.reserve(nonground.rows());
  low_indices.reserve(nonground.rows());

  for (int i = 0; i < nonground.rows(); ++i) {
    if (nonground(i, 2) >= obstacle_z_threshold) {
      obstacle_indices.push_back(i);
    } else {
      low_indices.push_back(i);
    }
  }

  Eigen::MatrixX3f obstacles(obstacle_indices.size(), 3);
  for (size_t i = 0; i < obstacle_indices.size(); ++i) {
    obstacles.row(static_cast<int>(i)) = nonground.row(obstacle_indices[i]);
  }

  low_points->resize(low_indices.size(), 3);
  for (size_t i = 0; i < low_indices.size(); ++i) {
    low_points->row(static_cast<int>(i)) = nonground.row(low_indices[i]);
  }

  return obstacles;
}

sensor_msgs::PointCloud2 eigenToCloudMsg(const Eigen::MatrixX3f &points,
                                         const std_msgs::Header &header) {
  pcl::PointCloud<PointType> cloud;
  cloud.reserve(points.rows());
  for (int i = 0; i < points.rows(); ++i) {
    cloud.emplace_back(points(i, 0), points(i, 1), points(i, 2));
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;
  return msg;
}

void callbackNode(const sensor_msgs::PointCloud2ConstPtr &msg) {
  static int frame_count = 0;
  double ox;
  double oy;
  double oz;
  Eigen::Quaterniond oq;
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    if (!has_odom) {
      return;
    }
    ox = robot_x;
    oy = robot_y;
    oz = robot_z;
    oq = robot_q;
  }

  pcl::PointCloud<PointType> pcl_cloud;
  pcl::fromROSMsg(*msg, pcl_cloud);

  std::vector<Eigen::Vector3f> local_points;
  local_points.reserve(pcl_cloud.size());
  const Eigen::Vector3d origin(ox, oy, oz);
  for (const auto &pt : pcl_cloud.points) {
    const Eigen::Vector3d map_point(pt.x, pt.y, pt.z);
    const Eigen::Vector3d local_point = oq.inverse() * (map_point - origin);
    if (std::hypot(local_point.x(), local_point.y()) < self_filter_radius) {
      continue;
    }
    local_points.emplace_back(local_point.cast<float>());
  }

  Eigen::MatrixXf cloud(local_points.size(), 3);
  for (size_t i = 0; i < local_points.size(); ++i) {
    cloud.row(static_cast<int>(i)) = local_points[i];
  }

  patchworkpp->estimateGround(cloud);

  Eigen::MatrixX3f centered_ground = patchworkpp->getGround();
  Eigen::MatrixX3f raw_centered_nonground = patchworkpp->getNonground();
  Eigen::MatrixX3f low_nonground;
  Eigen::MatrixX3f centered_nonground =
      filterNongroundByHeight(raw_centered_nonground, &low_nonground);
  const int raw_nonground_count = raw_centered_nonground.rows();
  const int low_filtered_count = low_nonground.rows();
  const int original_ground_count = centered_ground.rows();
  centered_ground.conservativeResize(original_ground_count + low_filtered_count, 3);
  if (low_filtered_count > 0) {
    centered_ground.bottomRows(low_filtered_count) = low_nonground;
  }
  Eigen::MatrixX3f centered_cloud = cloud;

  if (frame_count++ % 30 == 0) {
    ROS_INFO_STREAM("[PatchworkPP] input=" << pcl_cloud.size()
                    << " filtered=" << (pcl_cloud.size() - local_points.size())
                    << " used=" << local_points.size()
                    << " ground=" << centered_ground.rows()
                    << " nonground=" << centered_nonground.rows()
                    << " raw_nonground=" << raw_nonground_count
                    << " low_filtered=" << low_filtered_count
                    << " time=" << patchworkpp->getTimeTaken());
  }

  for (int i = 0; i < centered_cloud.rows(); ++i) {
    const Eigen::Vector3d map_point = oq * centered_cloud.row(i).cast<double>().transpose() + origin;
    centered_cloud.row(i) = map_point.cast<float>().transpose();
  }
  for (int i = 0; i < centered_ground.rows(); ++i) {
    const Eigen::Vector3d map_point = oq * centered_ground.row(i).cast<double>().transpose() + origin;
    centered_ground.row(i) = map_point.cast<float>().transpose();
  }
  for (int i = 0; i < centered_nonground.rows(); ++i) {
    const Eigen::Vector3d map_point =
        oq * centered_nonground.row(i).cast<double>().transpose() + origin;
    centered_nonground.row(i) = map_point.cast<float>().transpose();
  }

  cloud_pub.publish(eigenToCloudMsg(centered_cloud, msg->header));
  ground_pub.publish(eigenToCloudMsg(centered_ground, msg->header));
  nonground_pub.publish(eigenToCloudMsg(centered_nonground, msg->header));
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "patchworkpp_offline_own_data");
  ros::NodeHandle nh;

  nh.param("patchworkpp/self_filter_radius", self_filter_radius, self_filter_radius);
  nh.param("patchworkpp/sensor_height", sensor_height, sensor_height);
  nh.param("patchworkpp/min_obstacle_height", min_obstacle_height, min_obstacle_height);
  patchworkpp = std::make_unique<patchwork::PatchWorkpp>(loadParams(nh));

  ros::Subscriber cloud_sub =
      nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud_scan", 5000, callbackNode);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 100, odomCallback);

  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100);
  ground_pub = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/P", 100);
  nonground_pub = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/N", 100);

  ROS_INFO("Patchwork++ ROS1 wrapper started.");
  ros::spin();
  return 0;
}
