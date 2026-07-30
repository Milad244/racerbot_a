#include "ftg_node.hpp"
using std::vector;


FollowTheGapNode::FollowTheGapNode() : Node("follow_the_gap_node")
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

// 1. Find all gaps
std::vector<std::pair<int, int>> FollowTheGapNode::find_all_gaps(const vector<float>& ranges)
{
    std::vector<std::pair<int, int>> gaps;
    size_t i = 0;
    
    while (i < ranges.size()) {
        if (ranges[i] <= minimum_gap_threshold_) { 
            ++i; 
            continue; 
        }

        size_t gap_start = i;
        while (i < ranges.size() && ranges[i] > minimum_gap_threshold_) {
            ++i;
        }
        size_t gap_end = i;
        
        gaps.push_back({static_cast<int>(gap_start), static_cast<int>(gap_end)});
    }
    
    return gaps;
}

// 2. Go through each gap and pick the best gap
std::pair<int, int> FollowTheGapNode::pick_best_gap(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    const vector<float>& ranges, 
    const std::vector<std::pair<int, int>>& gaps)
{
    std::pair<int, int> best_gap = {-1, -1};
    float best_score = -1.0f;

    for (const auto& gap : gaps) {
        int gap_start = gap.first;
        int gap_end = gap.second;

        // Calculate average depth of the gap
        float sum = 0.0f;
        for (int j = gap_start; j < gap_end; ++j) {
            sum += ranges[j];
        }
        float avg_depth = sum / (gap_end - gap_start);
        
        // Ensure the gap is wide enough for the car
        size_t min_width = static_cast<size_t>((car_width_ / avg_depth) / scan_msg->angle_increment);
        if (static_cast<size_t>(gap_end - gap_start) < min_width) continue;

        // Scoring based on width * depth (as suggested in the image)
        float score = (gap_end - gap_start) * avg_depth;

        if (score > best_score) {
            best_score = score;
            best_gap = gap;
        }
    }
    
    return best_gap;
}

// 3. Go through the best gap and pick the best point
int FollowTheGapNode::pick_best_point(const std::pair<int, int>& best_gap)
{
    if (best_gap.first == -1 || best_gap.second == -1) {
        return -1; // Invalid gap
    }
    
    // Pick the center of the gap (as suggested in the image)
    return best_gap.first + (best_gap.second - best_gap.first) / 2;
}

void FollowTheGapNode::lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto ranges = preprocess_lidar(scan_msg);
    extend_obstacles(scan_msg, ranges);
    auto gaps = find_all_gaps(ranges);
    auto best_gap = pick_best_gap(scan_msg, ranges, gaps);
    int best_idx = pick_best_point(best_gap);
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