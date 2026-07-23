#include "gap_follow_node.hpp"

GapFollowNode::GapFollowNode() : Node("gap_follow_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Follow node started");

    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    gap_sub_ = this->create_subscription<reactive::msg::Gap>(
        "gap", 10,
        std::bind(&GapFollowNode::gap_callback, this, std::placeholders::_1));

    this->declare_parameter("use_fallback_method", false);

    use_fallback_method = this->get_parameter("use_fallback_method").as_bool();
    RCLCPP_INFO(this->get_logger(), "Using follow method: '%s'", use_fallback_method ? "drive_best_point" : "least_squares");
}

void GapFollowNode::gap_callback(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
    if (use_fallback_method)
    {
        drive_best_point(gap_msg);
    }
    else
    {
        least_squares_pathfinding(gap_msg);
    }
}

void GapFollowNode::drive_best_point(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
    // Speed depends on target point range
    float velocity;
    float target_range = gap_msg->target_range;
    if (target_range > 2)
        velocity = 2;
    else if (target_range > 1)
        velocity = 1.5;
    else if (target_range > 0.5)
        velocity = 1;
    else
        velocity = 0.5;

    // Publishing to drive
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = gap_msg->target_angle;
    drive_msg.drive.speed = velocity;
    drive_pub_->publish(drive_msg);
}

void GapFollowNode::least_squares_pathfinding(const reactive::msg::Gap::ConstSharedPtr gap_msg)
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFollowNode>());
    rclcpp::shutdown();
    return 0;
}
