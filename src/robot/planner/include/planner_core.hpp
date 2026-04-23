#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <functional>
#include <queue>
#include <unordered_map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace robot
{

// (x, y) pair
struct CellIndex {
  int x; 
  int y;
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex& other) const { return !(*this == other); }
};

// hash fnc so CellIndex can be a key in std::unordered_map
struct CellIndexHash {
  std::size_t operator()(const CellIndex& idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// cell + its f-score, used in priority queue
struct AStarNode {
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// determines priority - lowest f wins
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) {
    // std::priority_queue is max-heap by default, flip comparison to make it min-heap
    return a.f_score > b.f_score; // smaller f has priority
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    void updateMap(const nav_msgs::msg::OccupancyGrid& msg);
    void updateGoal(const geometry_msgs::msg::PointStamped& msg);
    void updateOdometry(double x, double y);
    nav_msgs::msg::Path planPath();
    bool shouldReplan() const;
    bool goalReached() const;
    void markGoalReached();

  private:
    rclcpp::Logger logger_;

    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid map_;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;

    bool has_map_ = false;
    bool has_goal_ = false;

    bool worldToGrid(double wx, double wy, int& cx, int& cy) const;
    void gridToWorld(int cx, int cy, double& wx, double& wy) const;
    bool isOccupied(int cx, int cy) const;
    double heuristic(const CellIndex& a, const CellIndex& b) const;
};

}  

#endif  
