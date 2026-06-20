#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fields2cover.h>

#include <vector>
#include <string>
#include <cmath>
#include <chrono>

class PathPlannerNode : public rclcpp::Node {
public:
    PathPlannerNode() : Node("path_planner_node") {
        // Parameters for planning
        this->declare_parameter<double>("robot_width", 2.0);
        this->declare_parameter<double>("robot_turn_radius", 4.0);
        this->declare_parameter<double>("headland_width", 3.0);
        
        this->get_parameter("robot_width", robot_width_);
        this->get_parameter("robot_turn_radius", robot_turn_radius_);
        this->get_parameter("headland_width", headland_width_);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);
        
        // Generate path
        generate_coverage_path();

        // Publish the path periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PathPlannerNode::timer_callback, this)
        );
    }

private:
    void generate_coverage_path() {
        RCLCPP_INFO(this->get_logger(), "Generating coverage path with Fields2Cover...");
        
        // 1. Define field boundary (Outer ring of field polygon)
        f2c::types::Field field;
        f2c::types::LinearRing outer_ring;
        // Define a simple 80m x 40m rectangle field
        outer_ring.addPoint(0.0, 0.0);
        outer_ring.addPoint(80.0, 0.0);
        outer_ring.addPoint(80.0, 40.0);
        outer_ring.addPoint(0.0, 40.0);
        outer_ring.addPoint(0.0, 0.0);

        f2c::types::Cell cell;
        cell.addRing(outer_ring);
        field.setField(f2c::types::Cells(cell));

        // 2. Generate headlands (turn boundaries)
        f2c::hg::ConstHL headland_gen;
        auto no_headland_field = headland_gen.generateHeadlands(field.getField(), headland_width_);

        // 3. Define robot properties
        f2c::types::Robot robot(robot_width_, robot_width_);
        robot.setMinTurningRadius(robot_turn_radius_);

        // 4. Generate coverage swaths (parallel tracks)
        f2c::sg::BruteForce swath_gen;
        f2c::obj::SwathLength objective;
        auto swaths = swath_gen.generateBestSwaths(objective, robot.getCovWidth(), no_headland_field.getGeometry(0));

        // 5. Order the swaths (route planning)
        f2c::rp::BoustrophedonOrder path_order;
        auto sorted_swaths = path_order.genSortedSwaths(swaths);

        // 6. Plan turning paths (Dubins curves) between swaths
        f2c::pp::DubinsCurves turns;
        auto f2c_path = f2c::pp::PathPlanning::planPath(robot, sorted_swaths, turns);

        // 7. Convert Fields2Cover path to ROS nav_msgs/msg/Path
        ros_path_.header.frame_id = "map";
        
        for (const auto& state : f2c_path.getStates()) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = state.point.getX();
            pose.pose.position.y = state.point.getY();
            pose.pose.position.z = 0.0;

            double yaw = state.angle;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);

            ros_path_.poses.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Path generated successfully. Total waypoints: %zu", ros_path_.poses.size());
    }

    void timer_callback() {
        ros_path_.header.stamp = this->get_clock()->now();
        for (auto& pose : ros_path_.poses) {
            pose.header.stamp = ros_path_.header.stamp;
        }
        path_pub_->publish(ros_path_);
    }

    double robot_width_;
    double robot_turn_radius_;
    double headland_width_;
    
    nav_msgs::msg::Path ros_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
