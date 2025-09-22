#include "planner_node.hpp"
#include <queue>
#include <unordered_set>
#include <cmath>
#include <limits>

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score; // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

PlannerNode::PlannerNode()
    : Node("planner_node"), state_(State::WAITING_FOR_GOAL)
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

  goal_received_ = false; // If endpoint to path has been received
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void timerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    if (goalReached())
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
      planPath();
    }
  }
}

bool PlannerNode::goalReached()
{
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < 0.5; // Need to be within 0.5m
}

double heuristic(const CellIndex &a, const CellIndex &b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool isCellValid(const CellIndex &cell, const nav_msgs::msg::OccupancyGrid &map)
{
  if (cell.x < 0 || cell.y < 0 || cell.x >= static_cast<int>(map.info.width) || cell.y >= static_cast<int>(map.info.height))
    return false;

  int index = cell.y * map.info.width + cell.x;
  int8_t cost = map.data[index];

  return cost < 75; // Avoid high cost areas (75-100 -> likely obstacle)
}

std::vector<CellIndex> getNeighbors(const CellIndex &cell)
{
  std::vector<CellIndex> neighbors;
  neighbors.push_back({cell.x + 1, cell.y});
  neighbors.push_back({cell.x - 1, cell.y});
  neighbors.push_back({cell.x, cell.y + 1});
  neighbors.push_back({cell.x, cell.y - 1});
  return neighbors;
}

std::vector<CellIndex> aStar(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &start, const CellIndex &goal)
{
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set; // Available nodes
  std::unordered_set<CellIndex, CellIndexHash> closed_set; // Visited 
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from; // Backtracking
  std::unordered_map<CellIndex, double, CellIndexHash> g_score; // Cost from start

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  while (!open_set.empty())
  {
    AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal)
    {
      std::vector<CellIndex> path;
      CellIndex c = goal;
      // Reconstruct path
      while (!(c == start))
      {
        path.push_back(c);
        c = came_from[c];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end()); // Path should go start -> goal
      return path;
    }

    closed_set.insert(current.index);

    // Go through neigbors
    for (const auto &neighbor : getNeighbors(current.index))
    {
      if (!isCellValid(neighbor, map) || closed_set.count(neighbor) > 0)
        continue;

      int index = neighbor.y * map.info.width + neighbor.x;
      double move_cost = 1.0;

      double tentative_g = g_score[current.index] + move_cost;

      // If new neighbor/cheaper paht -> update for future ref
      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor])
      {
        came_from[neighbor] = current.index;
        g_score[neighbor] = tentative_g;
        double f_score = tentative_g + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f_score));
      }
    }
  }

  return {}; // No path found
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

  // Convert pose to starting cell index + goal to goal cell index
  int start_x = static_cast<int>((robot_pose_.position.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int start_y = static_cast<int>((robot_pose_.position.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  CellIndex start{start_x, start_y};
  int goal_x = static_cast<int>((goal_.point.x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int goal_y = static_cast<int>((goal_.point.y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  CellIndex goal{goal_x, goal_y};

  // Basic validation
  if (!isCellValid(start, current_map_) || !isCellValid(goal, current_map_))
  {
    RCLCPP_ERROR(this->get_logger(), "Start/Goal not valid.");
    return;
  }

  // A*
  std::vector<CellIndex> waypoints = aStar(current_map_, start, goal);
  if (waypoints.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No path.");
    return;
  }

  // Convert waypoints to path
  for (const auto &cell : waypoints)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->get_clock()->now();

    pose.pose.position.x = current_map_.info.origin.position.x + (cell.x + 0.5) * current_map_.info.resolution;
    pose.pose.position.y = current_map_.info.origin.position.y + (cell.y + 0.5) * current_map_.info.resolution;
    pose.pose.position.z = 0.0;
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