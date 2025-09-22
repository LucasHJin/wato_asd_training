#include "control_node.hpp"
#include <cmath>
#include <optional>

ControlNode::ControlNode() : Node("pure_pursuit_controller")
{
    // Initialize parameters
    lookahead_distance_ = 1.0;  // Lookahead distance
    goal_tolerance_ = 0.1;     // Distance to consider the goal reached
    linear_speed_ = 1.0;       // Constant forward speed

    // Subscribers and Publishers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg)
        { current_path_ = msg; });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        { robot_odom_ = msg; });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]()
        { controlLoop(); });
}

void ControlNode::controlLoop()
{
    // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_) {
        return;
    }

    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        // Stop the robot
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);

    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint()
{
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_)
        return std::nullopt;

    geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;

    // Check if close enough to goal
    auto &last_point = current_path_->poses.back();
    double dist_to_goal = computeDistance(robot_pos, last_point.pose.position);
    if (dist_to_goal <= goal_tolerance_)
    {
        return std::nullopt; // Stop it
    }

    // Else go thorugh path and find point lhdist away (last point if not far enough)
    for (const auto &pose : current_path_->poses)
    {
        double dist = computeDistance(robot_pos, pose.pose.position);
        
        if (dist >= lookahead_distance_)
        {
            return pose;
        }
    }

    return last_point;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
    geometry_msgs::msg::Twist cmd_vel;

    if (!robot_odom_)
        return cmd_vel;

    double x = robot_odom_->pose.pose.position.x;
    double y = robot_odom_->pose.pose.position.y;
    double yaw = extractYaw(robot_odom_->pose.pose.orientation);

    // Vector from robot to goal
    double dx = target.pose.position.x - x;
    double dy = target.pose.position.y - y;

    // Anchor goal to be based on robot
    double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    double distance = std::sqrt(local_x * local_x + local_y * local_y);
    
    if (distance < 1e-6)
    {
        return cmd_vel; // 0 if close enough
    }

    // Turning speed to reach target
    double curvature = (2.0 * local_y) / (distance * distance);

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = curvature * cmd_vel.linear.x;

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat)
{
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}