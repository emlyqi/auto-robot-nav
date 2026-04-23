#include "control_core.hpp"

#include <cmath>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void ControlCore::updatePath(const nav_msgs::msg::Path& msg) {
  path_ = msg;
  has_path_ = true;
}

void ControlCore::updateOdometry(const nav_msgs::msg::Odometry& msg) {
  robot_x_ = msg.pose.pose.position.x;
  robot_y_ = msg.pose.pose.position.y;

  // extract yaw from quarternion
  const auto& q = msg.pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Twist ControlCore::computeCommand() {
  geometry_msgs::msg::Twist cmd; // default: zero velocity

  if (!has_path_ || path_.poses.empty()) return cmd;

  // stop if close to end of path
  const auto& last = path_.poses.back();
  double dx_end = last.pose.position.x - robot_x_;
  double dy_end = last.pose.position.y - robot_y_;
  double dist_to_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
  if (dist_to_end < goal_tolerance_) return cmd;

  // find lookahead point
  auto target = findLookaheadPoint();
  if (!target.has_value()) return cmd;

  // angle from robot's position to lookahead point (world frame)
  double dx = target->pose.position.x - robot_x_;
  double dy = target->pose.position.y - robot_y_;
  double angle_to_target = std::atan2(dy, dx);

  // how much do we need to rotate? diff btwn target angle and our heading
  double heading_error = angle_to_target - robot_yaw_; // angle in rad

  // wrap to [-pi, pi] so we always turn the short way
  while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  // cap steering angle
  if (std::abs(heading_error) > max_steering_angle_) {
    cmd.linear.x = 0; // stop and rotate in place
  } else {
    cmd.linear.x = linear_speed_;
  }
  heading_error = std::max(-max_steering_angle_, std::min(heading_error, max_steering_angle_));
  cmd.angular.z = 2.0 * heading_error;

  return cmd;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() const {
  if (!has_path_ || path_.poses.empty()) return std::nullopt;

  for (const auto& pose : path_.poses) {
    double dx = pose.pose.position.x - robot_x_;
    double dy = pose.pose.position.y - robot_y_;
    double d = std::sqrt(dx * dx + dy * dy);

    // forward-only filter
    double angle_to_pose = std::atan2(dy, dx);
    double angle_diff = angle_to_pose - robot_yaw_;
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
    if (std::abs(angle_diff) > M_PI_2) continue; // skip pts behind
    if (d >= lookahead_dist_) return pose;
  }

  // all waypoints are within lookahead -> we're near the end; aim at last one
  return path_.poses.back();
}

bool ControlCore::hasPath() const {
  return has_path_ && !path_.poses.empty();
}

}  
