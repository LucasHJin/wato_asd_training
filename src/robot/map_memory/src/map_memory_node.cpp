#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory_node"), last_x_(0.0), last_y_(0.0), distance_threshold_(1.5) // initialize threshold
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
    global_map_.header.frame_id = "map";
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
    latest_odom_ = *msg;  // Store the full odometry message
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    double distance = std::hypot(robot_x_ - last_x_, robot_y_ - last_y_);
    if (distance >= distance_threshold_)
    {
        last_x_ = robot_x_;
        last_y_ = robot_y_;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap()
{
    if (should_update_map_ && costmap_updated_)
    {
        integrateCostmap();

        global_map_.header.stamp = this->now(); 
        map_pub_->publish(global_map_);

        should_update_map_ = false;
        costmap_updated_ = false;
    }
}

void MapMemoryNode::integrateCostmap()
{
    // Global map info
    int gw = global_map_.info.width;
    int gh = global_map_.info.height;
    double gres = global_map_.info.resolution;
    double gx0 = global_map_.info.origin.position.x;
    double gy0 = global_map_.info.origin.position.y;

    // Local costmap info
    int cw = latest_costmap_.info.width;
    int ch = latest_costmap_.info.height;
    double cres = latest_costmap_.info.resolution;
    double costmap_origin_x = latest_costmap_.info.origin.position.x;
    double costmap_origin_y = latest_costmap_.info.origin.position.y;

    // Ensure resolutions match
    if (std::abs(gres - cres) > 1e-6)
    {
        RCLCPP_WARN(this->get_logger(), "Resolution mismatch: global=%.3f, local=%.3f", gres, cres);
        return;
    }

    // Get robot orientation from odometry
    auto q = latest_odom_.pose.pose.orientation;
    double robot_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // Loop over local costmap
    for (int cy = 0; cy < ch; ++cy)
    {
        for (int cx = 0; cx < cw; ++cx)
        {
            int8_t cost = latest_costmap_.data[cy * cw + cx];
            if (cost < 0) continue; // Skip unknown cells

            // Convert costmap grid coordinates to local world coordinates
            // (relative to robot center)
            double local_x = costmap_origin_x + (cx + 0.5) * cres;
            double local_y = costmap_origin_y + (cy + 0.5) * cres;

            // Transform from robot-local coordinates to global world coordinates
            // Apply rotation (robot orientation) then translation (robot position)
            double cos_yaw = std::cos(robot_yaw);
            double sin_yaw = std::sin(robot_yaw);
            
            double global_x = robot_x_ + (local_x * cos_yaw - local_y * sin_yaw);
            double global_y = robot_y_ + (local_x * sin_yaw + local_y * cos_yaw);

            // Convert global world coordinates to global grid indices
            int gx = static_cast<int>((global_x - gx0) / gres);
            int gy = static_cast<int>((global_y - gy0) / gres);

            // Check bounds and update global map
            if (gx >= 0 && gx < gw && gy >= 0 && gy < gh)
            {
                auto &global_cell = global_map_.data[gy * gw + gx];

                // Integration policy: 
                // - If global cell is unknown (-1), use new data
                // - Otherwise, take the maximum cost (more conservative)
                if (global_cell < 0)
                {
                    global_cell = cost;
                }
                else
                {
                    global_cell = std::max(global_cell, cost);
                }
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
