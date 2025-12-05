#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cmath>

#include "raspicat_tvvf_navigation/waypoint_manager.hpp"

using raspicat_tvvf_navigation::WaypointManager;
using raspicat_tvvf_navigation::Waypoint;

namespace {

geometry_msgs::msg::Quaternion createQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  const double half_yaw = yaw * 0.5;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  return q;
}

geometry_msgs::msg::Pose createPose(double x, double y, double yaw) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = createQuaternion(yaw);
  return pose;
}

std::string writeTestCsv() {
  const std::string header = "id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command";
  // Waypoints:
// 0: default (empty -> loose)
// 1: strict
// 2: wait:3 (loose判定のまま)
  Waypoint wp0;
  wp0.id = 0;
  wp0.pose = createPose(0.0, 0.0, 0.0);
  wp0.command = "";

  Waypoint wp1;
  wp1.id = 1;
  wp1.pose = createPose(0.0, 0.0, 0.0);
  wp1.command = "strict";

  Waypoint wp2;
  wp2.id = 2;
  wp2.pose = createPose(0.0, 0.0, 0.0);
  wp2.command = "wait:3";

  auto to_line = [](const Waypoint& wp) {
    std::ostringstream ss;
    ss << wp.id << ","
       << wp.pose.position.x << ","
       << wp.pose.position.y << ","
       << wp.pose.position.z << ","
       << wp.pose.orientation.x << ","
       << wp.pose.orientation.y << ","
       << wp.pose.orientation.z << ","
       << wp.pose.orientation.w << ","
       << wp.command;
    return ss.str();
  };

  std::filesystem::path path = std::filesystem::temp_directory_path() / "waypoint_tolerance_test.csv";
  std::ofstream ofs(path);
  ofs << header << "\n";
  ofs << to_line(wp0) << "\n";
  ofs << to_line(wp1) << "\n";
  ofs << to_line(wp2) << "\n";
  ofs.close();
  return path.string();
}

}  // namespace

TEST(WaypointToleranceTest, LooseToleranceAppliesByDefault) {
  const std::string csv_path = writeTestCsv();
  WaypointManager mgr(/*position_tolerance_strict=*/0.3,
                      /*orientation_tolerance_strict=*/0.3,
                      /*position_tolerance_loose=*/0.5,
                      /*orientation_tolerance_loose=*/1.0);
  ASSERT_TRUE(mgr.loadWaypoints(csv_path));

  auto pose_within_loose = createPose(0.45, 0.0, 0.5);  // within loose, outside strict
  EXPECT_TRUE(mgr.isWaypointReached(pose_within_loose));

  auto pose_outside_loose = createPose(0.6, 0.0, 0.0);
  EXPECT_FALSE(mgr.isWaypointReached(pose_outside_loose));

  auto pose_outside_loose_ori = createPose(0.0, 0.0, 1.5);  // orientation diff > loose
  EXPECT_FALSE(mgr.isWaypointReached(pose_outside_loose_ori));
}

TEST(WaypointToleranceTest, StrictToleranceAppliedWhenCommandIsStrict) {
  const std::string csv_path = writeTestCsv();
  WaypointManager mgr(0.3, 0.3, 0.5, 1.0);
  ASSERT_TRUE(mgr.loadWaypoints(csv_path));

  // Advance to waypoint with command "strict"
  mgr.markCurrentReached();

  auto pose_strict_ok = createPose(0.25, 0.0, 0.0);  // within strict
  EXPECT_TRUE(mgr.isWaypointReached(pose_strict_ok));

  auto pose_strict_fail = createPose(0.35, 0.0, 0.4);  // outside strict
  EXPECT_FALSE(mgr.isWaypointReached(pose_strict_fail));
}

TEST(WaypointToleranceTest, NonStrictCommandKeepsLooseTolerance) {
  const std::string csv_path = writeTestCsv();
  WaypointManager mgr(0.3, 0.3, 0.5, 1.0);
  ASSERT_TRUE(mgr.loadWaypoints(csv_path));

  // Move to third waypoint (command wait:3)
  mgr.markCurrentReached();
  mgr.markCurrentReached();

  auto pose_loose_ok = createPose(0.45, 0.0, 0.0);
  EXPECT_TRUE(mgr.isWaypointReached(pose_loose_ok));
}
