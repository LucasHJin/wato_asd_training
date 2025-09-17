#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory_node"), last_x_(0.0), last_y_(0.0)
{
    // Subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Timer (1 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

    // Initialize global map
    global_map_.info.resolution = 0.1;
    global_map_.info.width = 300;
    global_map_.info.height = 300;
    global_map_.info.origin.position.x = -15.0;
    global_map_.info.origin.position.y = -15.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1);

    costmap_updated_ = true;
    should_update_map_ = true;
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    double distance = std::sqrt(std::pow(robot_x_ - last_x_, 2) + std::pow(robot_y_ - last_y_, 2));
    if (distance >= distance_threshold_) {
        last_x_ = robot_x_;
        last_y_ = robot_y_;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap()
{
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
        costmap_updated_ = false;
    }
}

void MapMemoryNode::integrateCostmap()
{
    int gw = global_map_.info.width;
    int gh = global_map_.info.height;
    double gres = global_map_.info.resolution;
    double gx0 = global_map_.info.origin.position.x;
    double gy0 = global_map_.info.origin.position.y;

    int cw = latest_costmap_.info.width;
    int ch = latest_costmap_.info.height;
    double cres = latest_costmap_.info.resolution;

    for (int cy = 0; cy < ch; ++cy) {
        for (int cx = 0; cx < cw; ++cx) {
            int cost = latest_costmap_.data[cy * cw + cx];
            if (cost < 0) continue; // skip unknown

            // --- Transform chain ---
            // Local costmap indices -> robot-local coords
            double lx = (cx - cw/2.0) * cres;
            double ly = (cy - ch/2.0) * cres;

            // Robot-local -> world coords using odometry
            double wx = robot_x_ + lx;
            double wy = robot_y_ + ly;

            // World -> global map indices
            int gx = static_cast<int>((wx - gx0) / gres);
            int gy = static_cast<int>((wy - gy0) / gres);

            if (gx >= 0 && gx < gw && gy >= 0 && gy < gh) {
                global_map_.data[gy * gw + gx] = cost;
            }
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
