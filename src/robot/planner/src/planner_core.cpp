#include <cmath>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {} 

void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid& msg) {
    map_ = msg;
    has_map_ = true;
}

void PlannerCore::updateGoal(const geometry_msgs::msg::PointStamped& msg) {
    goal_x_ = msg.point.x;
    goal_y_ = msg.point.y;
    has_goal_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
}

void PlannerCore::updateOdometry(double x, double y) {
    robot_x_ = x;
    robot_y_ = y;
}

bool PlannerCore::worldToGrid(double wx, double wy, int& cx, int& cy) const {
    cx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
    cy = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
    int w = map_.info.width, h = map_.info.height;
    return cx >= 0 && cx < w && cy >= 0 && cy < h;
}

void PlannerCore::gridToWorld(int cx, int cy, double& wx, double& wy) const {
    wx = map_.info.origin.position.x + (cx + 0.5) * map_.info.resolution;
    wy = map_.info.origin.position.y + (cy + 0.5) * map_.info.resolution;
}

bool PlannerCore::isOccupied(int cx, int cy) const {
    int idx = cy * map_.info.width + cx;
    int8_t v = map_.data[idx];
    return v >= 50; // treat 50+ as blocked
}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) const {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// a*
nav_msgs::msg::Path PlannerCore::planPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id = "sim_world";

    if (!has_map_ || !has_goal_) return path;

    // start & goal as grid cells
    int sx, sy, gx, gy;
    if (!worldToGrid(robot_x_, robot_y_, sx, sy)) return path;
    if (!worldToGrid(goal_x_, goal_y_, gx, gy)) return path;
    CellIndex start(sx, sy);
    CellIndex goal(gx, gy);

    // a* data structures
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    g_score[start] = 0.0;
    open.emplace(start, heuristic(start, goal));

    // 8 connected neighbours
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const double SQRT2 = std::sqrt(2.0);
    const double step_cost[] = {SQRT2, 1.0, SQRT2, 1.0, 1.0, SQRT2, 1.0, SQRT2};

    const int w = map_.info.width;
    const int h = map_.info.height;

    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        // reached the goal - rebuild path by walking came_from backward
        if (current.index == goal) {
            std::vector<CellIndex> cells;
            CellIndex c = goal;
            while (c != start) {
                cells.push_back(c);
                c = came_from[c];
            }
            cells.push_back(start);
            std::reverse(cells.begin(), cells.end());

            for (const auto& cell : cells) {
                double wx, wy;
                gridToWorld(cell.x, cell.y, wx, wy);
                geometry_msgs::msg::PoseStamped pose; // = pose (posn + orientatn) + header (frame + time)
                pose.header.frame_id = "sim_world";
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
            return path;
        }

        // try each neighbour
        for (int i = 0; i < 8; ++i) {
            int nx = current.index.x + dx[i];
            int ny = current.index.y + dy[i];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (isOccupied(nx, ny)) continue;

            CellIndex neighbour(nx, ny);
            double tentative_g = g_score[current.index] + step_cost[i];

            auto it = g_score.find(neighbour);
            if (it == g_score.end() || tentative_g < it->second) {
                came_from[neighbour] = current.index;
                g_score[neighbour] = tentative_g;
                double f = tentative_g + heuristic(neighbour, goal);
                open.emplace(neighbour, f);
            }
        }
    }

    return path; // no path found
}

bool PlannerCore::shouldReplan() const {
    return state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL && has_map_ && has_goal_;
}

bool PlannerCore::goalReached() const {
    double dx = goal_x_ - robot_x_;
    double dy = goal_x_ - robot_y_;
    return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerCore::markGoalReached() {
    state_ = State::WAITING_FOR_GOAL;
    has_goal_ = false;
}

}