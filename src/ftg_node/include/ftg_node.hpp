#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
using std::vector;
/// @brief The ROS node responsible for controlling the car with Follow The Gap
class FollowTheGapNode : public rclcpp::Node
{
public:
    FollowTheGapNode();

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_msg_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;
    double max_lidar_range_;
    double fov_half_angle_;
    double disparity_threshold_;
    double car_width_;
    double minimum_gap_threshold_;

    /// @brief clean lidar scan
    /// @param scan_msg The scan data from the lidar
    /// @return preprocessed range vector
    vector<float> preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    /// @brief extend obstacles outward to account for car width
    /// @param scan_msg the scan data from the lidar
    /// @param ranges preprocessed range vector to modify in place
    void extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
        vector<float>& ranges);

    /// @brief creates a safety bubble around closest obstacle
    /// @param ranges preprocessed range vector to modify in place
    /// @param scan_msg the scan data from the lidar
    void draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, 
                        std::vector<float>& ranges);

    /// @brief find the index of the best point in the furthest valid gap
    /// @param scan_msg The scan data from the lidar
    /// @param ranges preprocessed and extended range vector
    /// @return index of best point, or -1 if no valid gap found
    int find_best_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
        const vector<float>& ranges);

    /// @brief func to be called when the lidar completes a scan
    /// @param scan_msg The scan data from the lidar
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};