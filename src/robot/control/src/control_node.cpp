#include "control_node.hpp"
#include <cmath>
#include <optional>
#include <algorithm>

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  // Initialize parameters
  lookahead_distance_ = 1.0; // Lookahead distance
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

// Main control loop
void ControlNode::controlLoop()
{
    if (!current_path_ || !robot_odom_)
        return;

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point)
    {
        // Stop the robot if goal is reached or no path
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    auto cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

// Find the lookahead point along the path
std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint()
{
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_)
        return std::nullopt;

    geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;

    // Find the closest point on the path to the robot
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < current_path_->poses.size(); ++i)
    {
        double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // Search forward from the closest point for the lookahead point
    for (size_t i = closest_idx; i < current_path_->poses.size(); ++i)
    {
        double dist = computeDistance(robot_pos, current_path_->poses[i].pose.position);
        
        // If we find a point at or beyond lookahead distance, use it
        if (dist >= lookahead_distance_)
        {
            return current_path_->poses[i];
        }
    }

    // If no point is beyond lookahead distance, check if we've reached the goal
    auto &last_point = current_path_->poses.back();
    double dist_to_goal = computeDistance(robot_pos, last_point.pose.position);
    
    if (dist_to_goal <= goal_tolerance_)
    {
        return std::nullopt; // Goal reached
    }

    // Return the last point if we're close to the end of the path
    return last_point;
}

// Compute velocity command toward target point
geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
    geometry_msgs::msg::Twist cmd_vel;

    if (!robot_odom_)
        return cmd_vel;

    // Robot position and heading
    double x = robot_odom_->pose.pose.position.x;
    double y = robot_odom_->pose.pose.position.y;
    double yaw = extractYaw(robot_odom_->pose.pose.orientation);

    // Target position relative to robot
    double dx = target.pose.position.x - x;
    double dy = target.pose.position.y - y;

    // Transform target into robot frame (correct transformation)
    double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    // Actual distance to the target point
    double actual_distance = std::sqrt(local_x * local_x + local_y * local_y);
    
    // Avoid division by zero
    if (actual_distance < 1e-6)
    {
        return cmd_vel; // Return zero velocity
    }

    // Pure pursuit curvature calculation using actual distance
    double curvature = (2.0 * local_y) / (actual_distance * actual_distance);

    // Limit curvature to prevent excessive turning
    const double max_curvature = 2.0; // Adjust as needed
    curvature = std::clamp(curvature, -max_curvature, max_curvature);

    // Set velocity commands
    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = curvature * linear_speed_;

    // Limit angular velocity
    const double max_angular_vel = 1.5; // rad/s
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_vel, max_angular_vel);

    return cmd_vel;
}

// Compute Euclidean distance between two points
double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

// Convert quaternion to yaw
double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat)
{
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Main entry point
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}