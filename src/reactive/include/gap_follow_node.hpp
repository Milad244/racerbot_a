#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "reactive/msg/gap.hpp"

class GapFollowNode : public rclcpp::Node
{
public:
    GapFollowNode();

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<reactive::msg::Gap>::SharedPtr gap_sub_;

    bool use_fallback_method;

    /// @brief Callback invoked each time we find a valid gap from the laser scan.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void gap_callback(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Baseline method that publishes to drive, driving to the best gap found in the gap_msg.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void drive_best_point(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Uses the method of least squares to determine a curve the car should follow based on the gap it recieves.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void least_squares_pathfinding(const reactive::msg::Gap::ConstSharedPtr gap_msg);
};
