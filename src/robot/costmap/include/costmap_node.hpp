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
  // Subs + pubs
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  // Params
  double resolution_;
  int width_;
  int height_;

  // Data
  std::vector<std::vector<int>> costmap_;

  // Callbacks
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  // Helpers
  void initializeCostmap();
  void convertToGrid(double range, double angle, int &x, int &y);
  void markObstacle(int x, int y);
  void inflateObstacles();
  void publishCostmap();
};

#endif
