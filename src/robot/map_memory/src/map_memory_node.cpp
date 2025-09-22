#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory_node"), last_x_(0.0), last_y_(0.0), distance_threshold_(1.5)
{
    // Initialize subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

    // Initialize publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this));

    global_map_.header.frame_id = "map";
    global_map_.info.resolution = 0.1;
    global_map_.info.width = 300;
    global_map_.info.height = 300;
    global_map_.info.origin.position.x = -15.0;
    global_map_.info.origin.position.y = -15.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1);

    costmap_updated_ = false;
    should_update_map_ = false;
    first_update_ = true;

    // For making sure costmap has time to load before initial update
    first_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
            RCLCPP_INFO(this->get_logger(), "3 SECOND UPDATE");
            first_update_ = true;
            updateMap();
            first_timer_->cancel(); // stop this one-shot timer
        }
    );
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
    robot_yaw_ = std::atan2(2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + 
                                  msg->pose.pose.orientation.x * msg->pose.pose.orientation.y), 
                           1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + 
                                       msg->pose.pose.orientation.z * msg->pose.pose.orientation.z));

    if (first_update_) {
        // On first update -> last known position = current and update map
        last_x_ = robot_x_;
        last_y_ = robot_y_;
        should_update_map_ = true;
        costmap_updated_ = true;
        updateMap();
        RCLCPP_INFO(this->get_logger(), "UPDATED MAP");
        first_update_ = false;
    } else {
        // See if distance travelled is enough
        double distance = std::sqrt(std::pow(robot_x_ - last_x_, 2) + std::pow(robot_y_ - last_y_, 2));
        if (distance >= distance_threshold_) {
            last_x_ = robot_x_;
            last_y_ = robot_y_;
            should_update_map_ = true;
        }
    }
}

void MapMemoryNode::updateMap()
{
    // Update if data exists + moved enough
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
    // Global map
    int gw = global_map_.info.width;
    int gh = global_map_.info.height;
    double gres = global_map_.info.resolution;
    double gx0 = global_map_.info.origin.position.x;
    double gy0 = global_map_.info.origin.position.y;

    // Local costmap
    int cw = latest_costmap_.info.width;
    int ch = latest_costmap_.info.height;
    double cres = latest_costmap_.info.resolution;
    double costmap_origin_x = latest_costmap_.info.origin.position.x;
    double costmap_origin_y = latest_costmap_.info.origin.position.y;

    // Go through every cell in local costmap
    for (int cy = 0; cy < ch; ++cy)
    {
        for (int cx = 0; cx < cw; ++cx)
        {
            int8_t cost = latest_costmap_.data[cy * cw + cx];
            if (cost < 0) continue; 

            double local_x = costmap_origin_x + (cx + 0.5) * cres;
            double local_y = costmap_origin_y + (cy + 0.5) * cres;

            // NOTE -> need to apply rotation 
            double cos_yaw = std::cos(robot_yaw_);
            double sin_yaw = std::sin(robot_yaw_);
            
            double global_x = robot_x_ + (local_x * cos_yaw - local_y * sin_yaw);
            double global_y = robot_y_ + (local_x * sin_yaw + local_y * cos_yaw);

            // Coordinates -> grid indices
            int gx = static_cast<int>((global_x - gx0) / gres);
            int gy = static_cast<int>((global_y - gy0) / gres);

            if (gx >= 0 && gx < gw && gy >= 0 && gy < gh)
            {
                auto &global_cell = global_map_.data[gy * gw + gx];
                if (global_cell < 0)
                {
                    global_cell = cost; // Update if not yet updated
                }
                else
                {
                    global_cell = std::max(global_cell, cost); // Take only the higher one (safer)
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