#include "planner_node.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>
#include <limits>

PlannerNode::PlannerNode()
    : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL)
{
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// Map callback
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    planPath();
  }
}

// Goal callback
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

// Odometry callback
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

// Timer callback
void PlannerNode::timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      goal_received_ = false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

// Check if goal is reached
bool PlannerNode::goalReached()
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // threshold
}

// Heuristic: Manhattan distance
double heuristic(const CellIndex &a, const CellIndex &b)
{
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// Check if a cell is valid (inside map bounds and not an obstacle)
bool isCellValid(const CellIndex &cell, const nav_msgs::msg::OccupancyGrid &map)
{
  if (cell.x < 0 || cell.y < 0 || cell.x >= static_cast<int>(map.info.width) || cell.y >= static_cast<int>(map.info.height))
    return false;
  int index = cell.y * map.info.width + cell.x;
  return map.data[index] == 0; // 0 = free, >0 = occupied
}

// Get neighbors (4-connected)
std::vector<CellIndex> getNeighbors(const CellIndex &cell)
{
  return {{cell.x + 1, cell.y}, {cell.x - 1, cell.y}, {cell.x, cell.y + 1}, {cell.x, cell.y - 1}};
}

// A* search algorithm
std::vector<CellIndex> aStar(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &start, const CellIndex &goal)
{
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  while (!open_set.empty())
  {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal)
    {
      // Reconstruct path
      std::vector<CellIndex> path;
      CellIndex c = goal;
      while (!(c == start))
      {
        path.push_back(c);
        c = came_from[c];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    closed_set.insert(current.index);

    for (const CellIndex &neighbor : getNeighbors(current.index))
    {
      if (!isCellValid(neighbor, map) || closed_set.count(neighbor) > 0)
        continue;

      double tentative_g = g_score[current.index] + 1.0; // cost between neighbors = 1

      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor])
      {
        came_from[neighbor] = current.index;
        g_score[neighbor] = tentative_g;
        double f_score = tentative_g + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f_score));
      }
    }
  }

  // Return empty if no path found
  return {};
}

void PlannerNode::planPath()
{
  if (!goal_received_ || current_map_.data.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";

  // Convert robot_pose_ to start CellIndex
  int start_x = static_cast<int>((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int start_y = static_cast<int>((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  CellIndex start{start_x, start_y};

  // Convert goal_ to goal CellIndex
  int goal_x = static_cast<int>((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int goal_y = static_cast<int>((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  CellIndex goal{goal_x, goal_y};

  RCLCPP_INFO(this->get_logger(), "Planning path from (%d, %d) to (%d, %d)", start.x, start.y, goal.x, goal.y);

  // Run A* search
  std::vector<CellIndex> waypoints = aStar(current_map_, start, goal);

  for (const auto &cell : waypoints)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();

    // Convert grid cell to world coordinates
    pose.pose.position.x = current_map_.info.origin.position.x + (cell.x + 0.5) * current_map_.info.resolution;
    pose.pose.position.y = current_map_.info.origin.position.y + (cell.y + 0.5) * current_map_.info.resolution;
    pose.pose.position.z = 0.0;

    // Keep default orientation
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "Path published with %zu waypoints", waypoints.size());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
