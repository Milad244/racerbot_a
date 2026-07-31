#include "ftg_node.hpp"
#include <algorithm>
using std::vector;


FollowTheGapNode::FollowTheGapNode() : Node("follow_the_gap_node"),
    // tuning parameters
    max_lidar_range_(10.0),
    fov_half_angle_(M_PI / 2.0),
    disparity_threshold_(0.4),
    car_width_(0.35),
    minimum_gap_threshold_(1.0)
{
    RCLCPP_INFO(this->get_logger(), "Follow the gap node started");

    this->drive_msg_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    this->laser_scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&FollowTheGapNode::lidar_callback, this, std::placeholders::_1));
}
//preprocess (PP) lidar
vector<float> FollowTheGapNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    vector<float> ranges = scan_msg->ranges;

    for (size_t i = 0; i < ranges.size(); ++i) {
        float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
        if (std::abs(angle) > fov_half_angle_) ranges[i] = 0.0f;
        if (!std::isfinite(ranges[i]) || ranges[i] < scan_msg->range_min) ranges[i] = 0.0f;
        ranges[i] = std::min(ranges[i], static_cast<float>(max_lidar_range_));
    }

    for (size_t i = 1; i < ranges.size() - 1; ++i) {
        ranges[i] = (ranges[i-1] + ranges[i] + ranges[i+1]) / 3.0f;
    }

    return ranges;
}
void FollowTheGapNode::extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    if (ranges.empty()) return;

    float prev_reading = ranges[0];
    for (size_t i = 0; i < ranges.size(); ++i) {
        float current_reading = ranges[i];

        if (std::abs(prev_reading - current_reading) > disparity_threshold_) {
            float closer_range = std::min(prev_reading, current_reading);

            double theta = car_width_ / closer_range;
            size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0);

            size_t start = (i > index_increment) ? i - index_increment : 0;
            size_t end = std::min(i + index_increment + 1, ranges.size());

            for (size_t j = start; j < end; ++j) {
                ranges[j] = std::min(ranges[j], closer_range);
            }
        }
        prev_reading = ranges[i];
    }
}
//draw safety bubble around the closest point found. set all points within bubble to 0.
void FollowTheGapNode::draw_safety_bubble(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
                                          std::vector<float>& ranges)
{
    // Find closest obstacle (ignoring 0 ranges since 0 will already be avoided)
    bool found_closest = false;
    size_t closest_index = 0;
    for (size_t i = 1; i < ranges.size(); ++i) {
        if (ranges[i] != 0 && !found_closest) {
            found_closest = true;
            closest_index = i;
        }

        if (found_closest && ranges[i] != 0 && ranges[i] < ranges[closest_index]) {
            closest_index = i;
        }
    }

    if (!found_closest) return; // handle case where all ranges are 0

    // Draw safety bubble by calculating theta for the arc made by r = closest point with s = car width
    // width = 0.35 meters (as car width is 0.3 meters)
    double theta = 0.35 / ranges[closest_index];
    RCLCPP_INFO(this->get_logger(), "Range: %f", ranges[closest_index]);

    size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0); // Used on each side so divide by 2

    size_t start = (closest_index > index_increment) ? closest_index - index_increment : 0;
    size_t end = std::min(closest_index + index_increment, ranges.size());

    for (size_t i = start; i < end; ++i) {
        ranges[i] = 0.0f;
    }
}
int FollowTheGapNode::find_best_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    const vector<float>& ranges)
{
    int best_index = -1;
    float furthest_range = 0.0f;

    size_t i = 0;
    while (i < ranges.size()) {
        if (ranges[i] <= minimum_gap_threshold_) { ++i; continue; }

        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_) ++i;
        size_t gap_end = i;

        // i still think avg range is better but it might not be :p
        float sum = 0.0f;
        for (size_t j = gap_start; j < gap_end; ++j) sum += ranges[j];
        float avg_range = sum / (gap_end - gap_start);
        size_t min_width = static_cast<size_t>((car_width_ / avg_range) / scan_msg->angle_increment);

        if ((gap_end - gap_start) < min_width) continue;

        for (size_t j = gap_start; j < gap_end; ++j) {
            if (ranges[j] > furthest_range) {
                furthest_range = ranges[j];
                best_index = static_cast<int>(j);
            }
        }
    }
    return best_index;
}

void FollowTheGapNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto ranges = preprocess_lidar(scan_msg);
    draw_safety_bubble(scan_msg, ranges);
    int best_idx = find_best_gap(scan_msg, ranges);
                                            //skibidi milad
    if (best_idx == -1) {
        RCLCPP_WARN(this->get_logger(), "No valid gap found");
        return;
    }

    double steering_angle = scan_msg->angle_min + best_idx * scan_msg->angle_increment;
    steering_angle = std::clamp(steering_angle, -0.4189, 0.4189);

    // angle based speed
    float abs_angle = std::abs(steering_angle);
    float speed;
    if (abs_angle < 10.0 * M_PI / 180.0)      speed = 2.0f;
    else if (abs_angle < 20.0 * M_PI / 180.0) speed = 1.5f;
    else                                        speed = 0.5f;

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = scan_msg->header.stamp;
    drive_msg.drive.steering_angle = static_cast<float>(steering_angle);
    drive_msg.drive.speed = speed;
    drive_msg_publisher->publish(drive_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGapNode>());
    rclcpp::shutdown();
    return 0;
}