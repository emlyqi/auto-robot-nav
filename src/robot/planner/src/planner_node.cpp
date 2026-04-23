#include <chrono>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );
  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
  );
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this)
  );
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  planner_.updateMap(*msg);

  // if we're actively chasing a goal, replan right away
  if (planner_.shouldReplan()) {
    auto path = planner_.planPath();
    path.header.stamp = this->get_clock()->now();
    path_pub_->publish(path);
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  planner_.updateGoal(*msg);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  planner_.updateOdometry(msg->pose.pose.position.x, msg->pose.pose.position.y);
} 

void PlannerNode::timerCallback() {
  if (!planner_.shouldReplan()) return;

  if (planner_.goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    planner_.markGoalReached(); // switch state back to WAITING_FOR_GOAL
    return;
  }

  // no replanning here bc now handled by map callback
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
