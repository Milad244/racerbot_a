#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "reactive/msg/gap.hpp"

#include <Eigen/Dense>

#define DEG2RAD(x) x *(M_PI / 180.0f)

class GapFollowNode : public rclcpp::Node
{
public:
    GapFollowNode();

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<reactive::msg::Gap>::SharedPtr gap_sub_;

    bool use_fallback_method;

    // Least Squares parameters
    int degree = 2;
    float k_p = 1.0;
    float lookahead_distance = 1.5; // meters

    float max_steering_angle = DEG2RAD(90.0f);

    /// @brief Callback invoked each time we find a valid gap from the laser scan.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void gap_callback(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Baseline method that publishes to drive, driving to the best gap found in the gap_msg.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void drive_best_point(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Uses the method of least squares to determine a curve the car should follow based on the gap it recieves.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void least_squares_pathfinding(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Converts a set of polar coordinates to Cartesian coordinates
    /// @param r radius of coordinate
    /// @param theta angle of coordinate (in radians)
    /// @returns The Cartesian coordinate
    std::pair<float, float> polar_to_cartesian(float r, float theta);

    /// @brief Determine coefficients for the best fitting curve, given a set of inputs (X coordinates) and outputs (Y coordinates)
    /// @param x Vector of X coordinates
    /// @param y Vector of Y coordinates
    /// @returns Vector of coefficients for the polynomial, ordered from lowest degree to highest degree
    Eigen::VectorXf fit_polynomial(const Eigen::VectorXf &x, const Eigen::VectorXf &y);

    /// @brief Given coefficients and an input value, returns the output value. Basically y = f(x).
    /// @param x Input value
    /// @param coefficients Coefficients of the function, ordered from lowest degree to highest degree
    /// @returns The output value 'y'
    float get_curve_output(float x, Eigen::VectorXf coefficients);

    /// @brief Simple method to determine steering angle based on coefficients that were calculated
    /// @param coefficients Coefficients of the function, ordered from lowest degree to highest degree
    /// @param target_x How far ahead the car should look on the curve
    /// @returns The angle the car should steer
    float compute_steering_angle_simple(Eigen::VectorXf coefficients, float target_x);
};
