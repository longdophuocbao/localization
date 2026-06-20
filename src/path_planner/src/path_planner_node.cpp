#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fields2cover.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <robot_localization/srv/to_ll.hpp>
#include <robot_localization/srv/from_ll.hpp>

#include "path_planner/srv/record_vertex.hpp"
#include "path_planner/srv/generate_path.hpp"
#include "path_planner/srv/save_path.hpp"
#include "path_planner/srv/load_path.hpp"

#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <memory>
#include <fstream>
#include <sstream>
#include <iomanip>

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

        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize clients to navsat_transform_node for coordinate conversion
        to_ll_client_ = this->create_client<robot_localization::srv::ToLL>("/to_ll");
        from_ll_client_ = this->create_client<robot_localization::srv::FromLL>("/from_ll");

        // Initialize with default 80m x 40m field vertices so there's a starting boundary
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = 0.0;  p1.y = 0.0;  p1.z = 0.0;
        p2.x = 80.0; p2.y = 0.0;  p2.z = 0.0;
        p3.x = 80.0; p3.y = 40.0; p3.z = 0.0;
        p4.x = 0.0;  p4.y = 40.0; p4.z = 0.0;
        recorded_vertices_ = {p1, p2, p3, p4};
        
        // Generate initial path using default vertices
        generate_coverage_path_from_recorded();

        // Create services
        record_vertex_srv_ = this->create_service<path_planner::srv::RecordVertex>(
            "record_vertex",
            std::bind(&PathPlannerNode::record_vertex_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        generate_path_srv_ = this->create_service<path_planner::srv::GeneratePath>(
            "generate_path",
            std::bind(&PathPlannerNode::generate_path_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        save_path_srv_ = this->create_service<path_planner::srv::SavePath>(
            "save_path",
            std::bind(&PathPlannerNode::save_path_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        load_path_srv_ = this->create_service<path_planner::srv::LoadPath>(
            "load_path",
            std::bind(&PathPlannerNode::load_path_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Publish the path periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&PathPlannerNode::timer_callback, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Path planner node initialized. Services 'record_vertex', 'generate_path', 'save_path', and 'load_path' are ready.");
    }

private:
    void generate_coverage_path_from_recorded() {
        if (recorded_vertices_.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Cannot generate path: Field boundary needs at least 3 vertices (currently has %zu).",
                        recorded_vertices_.size());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generating default coverage path with %zu vertices...", recorded_vertices_.size());
        
        try {
            // 1. Define field boundary (Outer ring of field polygon)
            f2c::types::Field field;
            f2c::types::LinearRing outer_ring;
            for (const auto& pt : recorded_vertices_) {
                outer_ring.addPoint(pt.x, pt.y);
            }
            // Close the ring (must end at start point)
            outer_ring.addPoint(recorded_vertices_.front().x, recorded_vertices_.front().y);

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
            ros_path_.poses.clear();
            ros_path_.header.frame_id = "map";
            ros_path_.header.stamp = this->get_clock()->now();
            
            for (const auto& state : f2c_path.getStates()) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros_path_.header.stamp;
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
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate initial path: %s", e.what());
        }
    }

    void record_vertex_callback(
        const std::shared_ptr<path_planner::srv::RecordVertex::Request> request,
        std::shared_ptr<path_planner::srv::RecordVertex::Response> response)
    {
        if (request->action == path_planner::srv::RecordVertex::Request::ACTION_CLEAR) {
            recorded_vertices_.clear();
            response->success = true;
            response->message = "Recorded vertices cleared.";
            response->vertices = recorded_vertices_;
            RCLCPP_INFO(this->get_logger(), "Cleared all recorded vertices.");
            return;
        }

        if (request->action == path_planner::srv::RecordVertex::Request::ACTION_GET_STATUS) {
            response->success = true;
            response->message = "Vertices retrieved successfully.";
            response->vertices = recorded_vertices_;
            return;
        }

        if (request->action == path_planner::srv::RecordVertex::Request::ACTION_ADD_VERTEX) {
            geometry_msgs::msg::Point pt;
            if (request->use_current_pose) {
                try {
                    // Lookup current position of base_link relative to map frame
                    geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
                        "map", "base_link", tf2::TimePointZero);
                    
                    pt.x = transformStamped.transform.translation.x;
                    pt.y = transformStamped.transform.translation.y;
                    pt.z = transformStamped.transform.translation.z;
                } catch (const tf2::TransformException & ex) {
                    RCLCPP_ERROR(this->get_logger(), "Could not look up transform from map to base_link: %s", ex.what());
                    response->success = false;
                    response->message = std::string("Could not get robot pose from TF: ") + ex.what();
                    response->vertices = recorded_vertices_;
                    return;
                }
            } else {
                pt = request->custom_vertex;
            }

            recorded_vertices_.push_back(pt);
            response->success = true;
            response->message = "Vertex added successfully.";
            response->vertices = recorded_vertices_;
            RCLCPP_INFO(this->get_logger(), "Added vertex: x=%.2f, y=%.2f, z=%.2f. Total vertices: %zu",
                        pt.x, pt.y, pt.z, recorded_vertices_.size());
            return;
        }

        response->success = false;
        response->message = "Unknown action specified.";
        response->vertices = recorded_vertices_;
    }

    void generate_path_callback(
        const std::shared_ptr<path_planner::srv::GeneratePath::Request> request,
        std::shared_ptr<path_planner::srv::GeneratePath::Response> response)
    {
        if (recorded_vertices_.size() < 3) {
            response->success = false;
            response->message = "At least 3 vertices are required to generate a path.";
            return;
        }

        // Determine parameters to use (override node parameters if requested)
        double r_width = (request->robot_width > 0.0) ? request->robot_width : robot_width_;
        double r_turn_radius = (request->robot_turn_radius > 0.0) ? request->robot_turn_radius : robot_turn_radius_;
        double h_width = (request->headland_width >= 0.0) ? request->headland_width : headland_width_;

        RCLCPP_INFO(this->get_logger(), "Generating coverage path with parameters: robot_width=%.2f, turn_radius=%.2f, headland_width=%.2f",
                    r_width, r_turn_radius, h_width);

        try {
            // 1. Define field boundary (Outer ring of field polygon)
            f2c::types::Field field;
            f2c::types::LinearRing outer_ring;
            for (const auto& pt : recorded_vertices_) {
                outer_ring.addPoint(pt.x, pt.y);
            }
            // Close the ring (must end at start point)
            outer_ring.addPoint(recorded_vertices_.front().x, recorded_vertices_.front().y);

            f2c::types::Cell cell;
            cell.addRing(outer_ring);
            field.setField(f2c::types::Cells(cell));

            // 2. Generate headlands (turn boundaries)
            f2c::hg::ConstHL headland_gen;
            auto no_headland_field = headland_gen.generateHeadlands(field.getField(), h_width);

            // 3. Define robot properties
            f2c::types::Robot robot(r_width, r_width);
            robot.setMinTurningRadius(r_turn_radius);

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
            nav_msgs::msg::Path path;
            path.header.frame_id = "map";
            path.header.stamp = this->get_clock()->now();
            
            for (const auto& state : f2c_path.getStates()) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = path.header.stamp;
                pose.pose.position.x = state.point.getX();
                pose.pose.position.y = state.point.getY();
                pose.pose.position.z = 0.0;

                double yaw = state.angle;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = std::sin(yaw / 2.0);
                pose.pose.orientation.w = std::cos(yaw / 2.0);

                path.poses.push_back(pose);
            }

            // Update published path
            ros_path_ = path;
            path_pub_->publish(ros_path_);

            response->success = true;
            response->message = "Path generated successfully.";
            response->path = path;

            RCLCPP_INFO(this->get_logger(), "Path generated successfully. Total waypoints: %zu", path.poses.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate path: %s", e.what());
            response->success = false;
            response->message = std::string("Failed to generate path: ") + e.what();
        }
    }

    void save_path_callback(
        const std::shared_ptr<path_planner::srv::SavePath::Request> request,
        std::shared_ptr<path_planner::srv::SavePath::Response> response)
    {
        std::string file_path = request->file_path;
        if (file_path.empty()) {
            response->success = false;
            response->message = "File path is empty.";
            return;
        }

        // Check if /to_ll service is available
        if (!to_ll_client_->wait_for_service(std::chrono::seconds(2))) {
            response->success = false;
            response->message = "Service '/to_ll' of navsat_transform_node is not available. Ensure localization launch is running.";
            return;
        }

        std::vector<geometry_msgs::msg::PoseStamped> points_to_convert;
        if (request->only_vertices) {
            for (const auto& pt : recorded_vertices_) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position = pt;
                pose.pose.orientation.w = 1.0; // Default orientation
                points_to_convert.push_back(pose);
            }
            RCLCPP_INFO(this->get_logger(), "Saving %zu field boundary vertices to CSV...", points_to_convert.size());
        } else {
            if (ros_path_.poses.empty()) {
                response->success = false;
                response->message = "No path waypoints available to save. Generate a path first.";
                return;
            }
            points_to_convert = ros_path_.poses;
            RCLCPP_INFO(this->get_logger(), "Saving %zu path waypoints to CSV...", points_to_convert.size());
        }

        std::ofstream file(file_path);
        if (!file.is_open()) {
            response->success = false;
            response->message = "Could not open or create file: " + file_path;
            return;
        }

        // Write CSV header
        file << "latitude,longitude,altitude,yaw\n";

        size_t saved_count = 0;
        for (const auto& pose : points_to_convert) {
            // Request coordinate conversion from Map to Latitude/Longitude
            auto srv_req = std::make_shared<robot_localization::srv::ToLL::Request>();
            srv_req->map_point = pose.pose.position;

            auto result = to_ll_client_->async_send_request(srv_req);
            if (result.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
                auto res = result.get();
                
                // Convert orientation quaternion to 2D yaw angle
                double yaw = 2.0 * std::atan2(pose.pose.orientation.z, pose.pose.orientation.w);

                file << std::fixed << std::setprecision(8)
                     << res->ll_point.latitude << ","
                     << res->ll_point.longitude << ","
                     << res->ll_point.altitude << ","
                     << yaw << "\n";
                saved_count++;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Timeout converting point x=%.2f, y=%.2f to Latitude/Longitude",
                             pose.pose.position.x, pose.pose.position.y);
            }
        }

        file.close();

        if (saved_count == points_to_convert.size()) {
            response->success = true;
            response->message = "Saved path successfully. Total: " + std::to_string(saved_count) + " points.";
            RCLCPP_INFO(this->get_logger(), "Saved %zu points to %s successfully.", saved_count, file_path.c_str());
        } else {
            response->success = false;
            response->message = "Partial success. Saved " + std::to_string(saved_count) + "/" + std::to_string(points_to_convert.size()) + " points.";
            RCLCPP_WARN(this->get_logger(), "Only saved %zu/%zu points to %s due to coordinate conversion timeouts.",
                        saved_count, points_to_convert.size(), file_path.c_str());
        }
    }

    void load_path_callback(
        const std::shared_ptr<path_planner::srv::LoadPath::Request> request,
        std::shared_ptr<path_planner::srv::LoadPath::Response> response)
    {
        std::string file_path = request->file_path;
        if (file_path.empty()) {
            response->success = false;
            response->message = "File path is empty.";
            return;
        }

        // Check if /from_ll service is available
        if (!from_ll_client_->wait_for_service(std::chrono::seconds(2))) {
            response->success = false;
            response->message = "Service '/from_ll' of navsat_transform_node is not available. Ensure localization launch is running.";
            return;
        }

        std::ifstream file(file_path);
        if (!file.is_open()) {
            response->success = false;
            response->message = "Could not open file: " + file_path;
            return;
        }

        std::string line;
        // Read and skip header line
        if (!std::getline(file, line)) {
            response->success = false;
            response->message = "File is empty: " + file_path;
            return;
        }

        struct GpsPoint {
            double lat;
            double lon;
            double alt;
            double yaw;
        };

        std::vector<GpsPoint> gps_points;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line);
            std::string lat_s, lon_s, alt_s, yaw_s;
            if (std::getline(ss, lat_s, ',') &&
                std::getline(ss, lon_s, ',') &&
                std::getline(ss, alt_s, ',') &&
                std::getline(ss, yaw_s, ',')) 
            {
                try {
                    GpsPoint gp;
                    gp.lat = std::stod(lat_s);
                    gp.lon = std::stod(lon_s);
                    gp.alt = std::stod(alt_s);
                    gp.yaw = std::stod(yaw_s);
                    gps_points.push_back(gp);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Ignoring malformed line: '%s' (%s)", line.c_str(), e.what());
                }
            }
        }
        file.close();

        if (gps_points.empty()) {
            response->success = false;
            response->message = "No valid GPS points found in file.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Converting %zu GPS points from WGS84 to Map frame...", gps_points.size());

        std::vector<geometry_msgs::msg::PoseStamped> loaded_poses;
        size_t converted_count = 0;

        for (const auto& gp : gps_points) {
            // Request coordinate conversion from Latitude/Longitude to Map
            auto srv_req = std::make_shared<robot_localization::srv::FromLL::Request>();
            srv_req->ll_point.latitude = gp.lat;
            srv_req->ll_point.longitude = gp.lon;
            srv_req->ll_point.altitude = gp.alt;

            auto result = from_ll_client_->async_send_request(srv_req);
            if (result.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
                auto res = result.get();
                
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = this->get_clock()->now();
                pose.pose.position = res->map_point;

                // Reconstruct orientation quaternion from yaw
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = std::sin(gp.yaw / 2.0);
                pose.pose.orientation.w = std::cos(gp.yaw / 2.0);

                loaded_poses.push_back(pose);
                converted_count++;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Timeout converting lat=%.8f, lon=%.8f to Map frame", gp.lat, gp.lon);
            }
        }

        if (converted_count == 0) {
            response->success = false;
            response->message = "Failed to convert any GPS points to Map coordinates.";
            return;
        }

        if (request->as_vertices) {
            // Store coordinates as boundary vertices and trigger path planning
            recorded_vertices_.clear();
            for (const auto& pose : loaded_poses) {
                recorded_vertices_.push_back(pose.pose.position);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu points as field boundary vertices. Triggering path generation...", recorded_vertices_.size());
            generate_coverage_path_from_recorded();
            
            response->success = true;
            response->message = "Loaded vertices and regenerated path successfully.";
            response->point_count = recorded_vertices_.size();
        } else {
            // Store directly as generated path
            ros_path_.poses = loaded_poses;
            ros_path_.header.frame_id = "map";
            ros_path_.header.stamp = this->get_clock()->now();
            path_pub_->publish(ros_path_);
            
            RCLCPP_INFO(this->get_logger(), "Loaded %zu points directly as path waypoints.", ros_path_.poses.size());
            response->success = true;
            response->message = "Loaded path directly successfully.";
            response->point_count = ros_path_.poses.size();
        }
    }

    void timer_callback() {
        if (!ros_path_.poses.empty()) {
            ros_path_.header.stamp = this->get_clock()->now();
            for (auto& pose : ros_path_.poses) {
                pose.header.stamp = ros_path_.header.stamp;
            }
            path_pub_->publish(ros_path_);
        }
    }

    double robot_width_;
    double robot_turn_radius_;
    double headland_width_;
    
    std::vector<geometry_msgs::msg::Point> recorded_vertices_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr to_ll_client_;
    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr from_ll_client_;

    rclcpp::Service<path_planner::srv::RecordVertex>::SharedPtr record_vertex_srv_;
    rclcpp::Service<path_planner::srv::GeneratePath>::SharedPtr generate_path_srv_;
    rclcpp::Service<path_planner::srv::SavePath>::SharedPtr save_path_srv_;
    rclcpp::Service<path_planner::srv::LoadPath>::SharedPtr load_path_srv_;

    nav_msgs::msg::Path ros_path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    
    // Spin using a MultiThreadedExecutor to support nested/concurrent service calls (such as calling /to_ll or /from_ll)
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
