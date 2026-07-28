#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "reactive/msg/gap.hpp"

#include <Eigen/Dense>
#include <limits>

#define DEG2RAD(x) x *(M_PI / 180.0)

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
    double steering_gain = 1.0;
    double k_samples = 200;

    double max_steering_angle = DEG2RAD(90.0f);
    double filtered_steering_angle;

    /// @brief Callback invoked each time we find a valid gap from the laser scan.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void gap_callback(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Baseline method that publishes to drive, driving to the best gap found in the gap_msg.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void drive_best_point(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Uses the method of least squares to determine a curve the car should follow based on the gap it recieves.
    /// @param gap_msg Shared pointer to the incoming Gap message.
    void least_squares_pathfinding(const reactive::msg::Gap::ConstSharedPtr gap_msg);

    /// @brief Determine coefficients for the best fitting curve, given a set of inputs (X coordinates) and outputs (Y coordinates)
    /// @param x Vector of X coordinates
    /// @param y Vector of Y coordinates
    /// @returns Vector of coefficients for the polynomial, ordered from lowest degree to highest degree
    Eigen::VectorXd fit_polynomial(const Eigen::VectorXd &x, const Eigen::VectorXd &y);

    /// @brief Given coefficients and an input value, returns the output value. Basically y = f(x).
    /// @param x Input value
    /// @param coefficients Coefficients of the function, ordered from lowest degree to highest degree
    /// @returns The output value 'y'
    double get_curve_output(double x, Eigen::VectorXd coefficients);

    /// @brief Determine steering angle based on coefficients that were calculated
    /// @param coefficients Coefficients of the function, ordered from lowest degree to highest degree
    /// @param theta Vector of angles in the gap
    /// @param max_lookahead The maximum lookahead distance that the value should be clamped too
    /// @returns The angle the car should steer
    double compute_steering_angle(Eigen::VectorXd coefficients, Eigen::VectorXd theta, double max_lookahead);

    /// @brief The mathematical function that determines the car's speed based on the target_angle
    /// @param angle The target angle
    /// @return The speed the car should drive at
    double angle_to_speed_function(double angle);
};
