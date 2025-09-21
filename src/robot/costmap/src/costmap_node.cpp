#include <cmath>
#include <algorithm>
#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap_node"),
  resolution_(0.1), 
  width_(300),
  height_(300),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0)
{
  // Initialize costmap grid
  initializeCostmap();

  // Subscriber: /lidar
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Subscriber: /odom/filtered (to get robot pose)
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1));

  // Publisher: /costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::initializeCostmap() {
  costmap_.assign(height_, std::vector<int>(width_, 0));
}

void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  
  // Extract yaw from quaternion
  auto q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void CostmapNode::convertToGrid(double world_x, double world_y, int &x, int &y) {
  // Convert from local coordinates (relative to robot) to grid coordinates
  // The grid origin should be at the center for a local costmap
  x = static_cast<int>((world_x + (width_ * resolution_ / 2.0)) / resolution_);
  y = static_cast<int>((world_y + (height_ * resolution_ / 2.0)) / resolution_);
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
              if (distance <= 1.0 && distance > 0) {
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

void CostmapNode::clearRobotFootprint() {
  int robot_radius_cells = static_cast<int>(0.3 / resolution_); // assume 30cm radius
  int cx = width_ / 2;
  int cy = height_ / 2;

  for (int dy = -robot_radius_cells; dy <= robot_radius_cells; ++dy) {
    for (int dx = -robot_radius_cells; dx <= robot_radius_cells; ++dx) {
      int nx = cx + dx;
      int ny = cy + dy;
      if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
        double distance = std::sqrt(dx*dx + dy*dy) * resolution_;
        if (distance <= 0.3) {
          costmap_[ny][nx] = 0; // free
        }
      }
    }
  }
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = this->get_clock()->now();
  grid.header.frame_id = "base_link";  // Keep as base_link for local costmap

  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;
  
  // Origin should be at bottom-left corner of the grid
  // For a local costmap centered on robot, this means negative offsets
  grid.info.origin.position.x = -(width_ * resolution_ / 2.0);
  grid.info.origin.position.y = -(height_ * resolution_ / 2.0);
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;

  grid.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      grid.data[y * width_ + x] = costmap_[y][x];
    }
  }

  costmap_pub_->publish(grid);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Only process if we have odometry data
  if (robot_x_ == 0.0 && robot_y_ == 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "No odometry data received yet, skipping laser scan");
    return;
  }

  initializeCostmap();  

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      // Convert laser scan point to local coordinates (robot-centric)
      double local_x = range * std::cos(angle);
      double local_y = range * std::sin(angle);

      int gx, gy;
      convertToGrid(local_x, local_y, gx, gy);
      markObstacle(gx, gy);
    }
  }

  clearRobotFootprint();
  inflateObstacles();
  publishCostmap();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}