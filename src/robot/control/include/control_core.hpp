#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
    void updatePath(const nav_msgs::msg::Path& msg);
    void updateOdometry(const nav_msgs::msg::Odometry& msg);
    geometry_msgs::msg::Twist computeCommand();
    bool hasPath() const;

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::Path path_;
    bool has_path_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    double lookahead_dist_ = 0.5; // m - how far ahead to aim
    double linear_speed_ = 1.0; // m/s - constant forward speed
    double goal_tolerance_ = 0.3; // m - stop when within this of last waypoint
    double max_steering_angle_ = 0.5;

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() const;
};

} 

#endif 
