#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void updateCostmap(const nav_msgs::msg::OccupancyGrid& msg);
    void updateOdometry(const nav_msgs::msg::Odometry& msg);
    bool tryMerge();
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;
    
  private:
    rclcpp::Logger logger_;

    // global map params
    double resolution_ = 0.2;
    int width_ = 150; // 15m
    int height_ = 150;
    double origin_x_ = -15.0;
    double origin_y_ = -15.0;
    std::vector<int8_t> global_map_;

    // latest inbound data
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool has_costmap_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    // pose at last merge - used for distance threshold
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    bool has_last_ = false;

    double distance_threshold_ = 0.3; // m

    void mergeLatestCostmap();
};

}  

#endif  
