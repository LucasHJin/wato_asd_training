#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  // Subscriber & Publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Costmap storage
  std::vector<std::vector<int>> costmap_; // 2D costmap array
  double resolution_; // meters per cell
  int width_, height_; // number of cells
  double robot_x_, robot_y_;
  double origin_x_, origin_y_; // world coordinates of costmap origin
  double robot_yaw_;

  // Callback for LaserScan
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Helper functions
  void initializeCostmap();
  void convertToGrid(double range, double angle, int &x, int &y);
  void markObstacle(int x, int y);
  void inflateObstacles();
  void publishCostmap();
  void clearRobotFootprint();
};

#endif
