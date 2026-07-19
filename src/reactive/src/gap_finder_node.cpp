#include "gap_finder_node.hpp"
using std::vector;

GapFinderNode::GapFinderNode() : Node("gap_finder_node")
{
    RCLCPP_INFO(this->get_logger(), "Gap Finder node started");

    // Declare with a default value
    this->declare_parameter("max_lidar_range", 10.0);
    this->declare_parameter("car_width_extended", 0.35);
    this->declare_parameter("disparity_threshold", 2.0);
    this->declare_parameter("fov_half_angle", M_PI/2.0); // 90 degrees

    // Read into member variables
    max_lidar_range_ = this->get_parameter("max_lidar_range").as_double();
    car_width_extended_ = this->get_parameter("car_width_extended").as_double();
    disparity_threshold_ = this->get_parameter("disparity_threshold").as_double();
    fov_half_angle_ = this->get_parameter("fov_half_angle").as_double();
    
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&GapFinderNode::lidar_callback, this, std::placeholders::_1));
}

void GapFinderNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto ranges = preprocess_lidar(scan_msg);
    extend_obstacles(scan_msg, ranges);
    size_t gap_index = find_furthest_gap(scan_msg, ranges);
}

vector<float> GapFinderNode::preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    vector<float> ranges = scan_msg->ranges;

    // Capping values past max_lidar_range, and rejecting values past fov angle range
    for (size_t i = 0; i < ranges.size(); ++i) {
        if (std::abs(scan_msg->angle_min + scan_msg->angle_increment * i) > fov_half_angle_) {
            ranges[i] = 0;
        }
        ranges[i] = std::min(ranges[i], static_cast<float>(max_lidar_range_));
    }

    return ranges;
}

void GapFinderNode::extend_obstacles(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    if (ranges.empty()) return;

    float prev_reading = ranges[0];
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        float current_reading = ranges[i];

        if (std::abs(prev_reading - current_reading) > disparity_threshold_)
        {
            float closer_range = std::min(prev_reading, current_reading);
            
            // Safety bubble: arc length s = r * theta -> theta = car_width_extended_ / r
            double theta = car_width_extended_ / closer_range;
            // Index increment: Used on each side so divide by 2
            size_t index_increment = static_cast<size_t>((theta / scan_msg->angle_increment) / 2.0);
            
            size_t start = (i > index_increment) ? i - index_increment : 0;
            size_t end = std::min(i + index_increment + 1, ranges.size());
            
            for (size_t j = start; j < end; ++j) {
                ranges[j] = std::min(ranges[j], closer_range); // don't overwrite an already-closer point
            }
        }
        prev_reading = ranges[i];
    }
}

int GapFinderNode::find_furthest_gap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    vector<float>& ranges)
{
    // TODO
    return 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GapFinderNode>());
    rclcpp::shutdown();
    return 0;
}
