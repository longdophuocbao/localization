#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        // Declare parameters
        this->declare_parameter<double>("lookahead_gain", 0.5);      // k (seconds)
        this->declare_parameter<double>("min_lookahead_dist", 1.5);  // L_min (meters)
        this->declare_parameter<double>("wheelbase", 2.0);           // L (meters)
        this->declare_parameter<double>("max_steer_angle", 0.6);     // Maximum steering angle (rad) ~ 35 deg
        this->declare_parameter<double>("steering_ratio", 1.0);      // Ratio between motor angle and actual wheel angle

        this->get_parameter("lookahead_gain", lookahead_gain_);
        this->get_parameter("min_lookahead_dist", min_lookahead_dist_);
        this->get_parameter("wheelbase", wheelbase_);
        this->get_parameter("max_steer_angle", max_steer_angle_);
        this->get_parameter("steering_ratio", steering_ratio_);

        // Subscriptions
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PurePursuitNode::odom_callback, this, std::placeholders::_1));

        // Publisher
        steer_pub_ = this->create_publisher<std_msgs::msg::Float64>("/motor/cmd_angle", 10);

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit steering controller started.");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (path_.poses.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for reference path on /plan...");
            return;
        }

        // 1. Get current robot pose
        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;

        // Convert orientation Quaternion to Yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double robot_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

        // 2. Compute current linear speed
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double speed = std::sqrt(vx * vx + vy * vy);

        // 3. Compute dynamic lookahead distance: L_t = max(L_min, k * v)
        double lookahead_dist = std::max(min_lookahead_dist_, lookahead_gain_ * speed);

        // 4. Find lookahead target point on path
        geometry_msgs::msg::Point target_pt;
        bool found_point = find_lookahead_point(robot_x, robot_y, lookahead_dist, target_pt);

        if (!found_point) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Could not find a valid lookahead point, taking last path point.");
            target_pt = path_.poses.back().pose.position;
        }

        // 5. Transform lookahead point to robot's local frame
        double dx = target_pt.x - robot_x;
        double dy = target_pt.y - robot_y;

        // Rotate to local frame (x is forward, y is left)
        double local_x = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
        double local_y = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

        // 6. Compute curvature: kappa = 2 * y_local / L_t^2
        double kappa = 2.0 * local_y / (lookahead_dist * lookahead_dist);

        // 7. Calculate target steering angle: delta = atan(kappa * L)
        double target_steer = std::atan2(kappa * wheelbase_, 1.0);

        // Clamp to safety limit
        target_steer = std::max(-max_steer_angle_, std::min(target_steer, max_steer_angle_));

        // Scale by steering ratio to get motor angle
        double motor_angle = target_steer * steering_ratio_;

        // 8. Publish steering motor command
        auto cmd_msg = std_msgs::msg::Float64();
        cmd_msg.data = motor_angle;
        steer_pub_->publish(cmd_msg);

        RCLCPP_DEBUG(this->get_logger(), "Speed: %.2f m/s, Lookahead: %.2f m, Target Steer: %.2f deg, Motor Angle: %.2f rad",
                     speed, lookahead_dist, target_steer * 180.0 / M_PI, motor_angle);
    }

    bool find_lookahead_point(double rx, double ry, double l_dist, geometry_msgs::msg::Point& target_pt) {
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;

        // Find closest point index on path
        for (size_t i = 0; i < path_.poses.size(); ++i) {
            double dx = path_.poses[i].pose.position.x - rx;
            double dy = path_.poses[i].pose.position.y - ry;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // Walk forward from closest point to find lookahead point
        for (size_t i = closest_idx; i < path_.poses.size(); ++i) {
            double dx = path_.poses[i].pose.position.x - rx;
            double dy = path_.poses[i].pose.position.y - ry;
            double dist = std::sqrt(dx * dx + dy * dy);
            if (dist >= l_dist) {
                target_pt = path_.poses[i].pose.position;
                return true;
            }
        }

        return false;
    }

    // Parameters
    double lookahead_gain_;
    double min_lookahead_dist_;
    double wheelbase_;
    double max_steer_angle_;
    double steering_ratio_;

    // Data
    nav_msgs::msg::Path path_;

    // ROS Connections
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
