#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    void initializeCostmap();
    bool convertToGrid(double range, double angle, int& x_cell, int& y_cell) const;
    void markObstacle(int x_cell, int y_cell);
    void updateFromScan(const sensor_msgs::msg::LaserScan& scan);
    nav_msgs::msg::OccupancyGrid getOccupancyGrid() const;

  private:
    rclcpp::Logger logger_;
    
    double resolution_ = 0.05;
    int width_ = 200;
    int height_ = 200;
    double origin_x_ = -10.0;
    double origin_y_ = -10.0;
    int max_cost_ = 100;
    double inflation_radius_ = 1.0; // m
    std::vector<int8_t> grid_;

    void inflateObstacles();
};

}

#endif
