#include <cmath>
#include <algorithm>
#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap_node"),
  resolution_(0.1), 
  width_(200),
  height_(200),
  origin_x_(-10.0),
  origin_y_(-10.0) 
{
  // Initialize costmap grid
  initializeCostmap();

  // Subscriber: /lidar
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  // Publisher: /costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  // Subscriber: robot odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
    });
}

void CostmapNode::initializeCostmap() {
  costmap_.assign(height_, std::vector<int>(width_, 0));
}

void CostmapNode::convertToGrid(double range, double angle, int &x, int &y) {
  double world_x = range * std::cos(angle);
  double world_y = range * std::sin(angle);

  x = static_cast<int>((world_x - origin_x_) / resolution_);
  y = static_cast<int>((world_y - origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x, int y) {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    costmap_[y][x] = 100; 
  }
}

void CostmapNode::inflateObstacles() {
  int inflation_cells = static_cast<int>(1.0 / resolution_); 
  std::vector<std::vector<int>> inflated = costmap_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (costmap_[y][x] == 100) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
              double distance = std::sqrt(dx*dx + dy*dy) * resolution_;
              if (distance <= 1.0) {
                int cost = static_cast<int>(100 * (1 - distance / 1.0));
                inflated[ny][nx] = std::max(inflated[ny][nx], cost);
              }
            }
          }
        }
      }
    }
  }
  costmap_ = inflated;
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = "map";

  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;
  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      grid.data[y * width_ + x] = costmap_[y][x];
    }
  }

  costmap_pub_->publish(grid);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  origin_x_ = robot_x_ - width_ * resolution_ / 2.0;
  origin_y_ = robot_y_ - height_ * resolution_ / 2.0;

  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      int x, y;
      convertToGrid(range, angle, x, y);
      markObstacle(x, y);
    }
  }

  inflateObstacles();
  publishCostmap();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
