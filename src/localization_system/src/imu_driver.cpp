#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <atomic>
#include "serial_port.hpp"

class ImuDriverNode : public rclcpp::Node {
public:
    ImuDriverNode() : Node("imu_driver"), is_running_(true) {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM3");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<double>("yaw_offset_deg", 90.0);

        this->get_parameter("port", port_);
        this->get_parameter("baud", baud_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("yaw_offset_deg", yaw_offset_deg_);

        RCLCPP_INFO(this->get_logger(), "Starting IMU driver on %s @ %d baud", port_.c_str(), baud_);

        // Publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // Pre-configure IMU message
        imu_msg_.header.frame_id = frame_id_;

        // Set covariances
        imu_msg_.orientation_covariance = {
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        };
        imu_msg_.angular_velocity_covariance = {
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        };
        imu_msg_.linear_acceleration_covariance = {
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        };

        // Start serial loop in thread
        read_thread_ = std::thread(&ImuDriverNode::serial_loop, this);
    }

    ~ImuDriverNode() {
        is_running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        serial_conn_.close_port();
    }

private:
    int16_t parse_short(uint8_t b1, uint8_t b2) {
        return static_cast<int16_t>((b2 << 8) | b1);
    }

    void euler_to_quaternion(double roll, double pitch, double yaw,
                             double& qx, double& qy, double& qz, double& qw) {
        // Convert degrees to radians
        double r = roll * (M_PI / 180.0);
        double p = pitch * (M_PI / 180.0);
        
        // Apply yaw offset and convert to ENU
        double y = yaw * (M_PI / 180.0) + (yaw_offset_deg_ * (M_PI / 180.0));
        y = std::fmod(y + M_PI, 2.0 * M_PI);
        if (y < 0) {
            y += 2.0 * M_PI;
        }
        y -= M_PI;

        qx = std::sin(r/2) * std::cos(p/2) * std::cos(y/2) - std::cos(r/2) * std::sin(p/2) * std::sin(y/2);
        qy = std::cos(r/2) * std::sin(p/2) * std::cos(y/2) + std::sin(r/2) * std::cos(p/2) * std::sin(y/2);
        qz = std::cos(r/2) * std::cos(p/2) * std::sin(y/2) - std::sin(r/2) * std::sin(p/2) * std::cos(y/2);
        qw = std::cos(r/2) * std::cos(p/2) * std::cos(y/2) + std::sin(r/2) * std::sin(p/2) * std::sin(y/2);
    }

    void serial_loop() {
        std::vector<uint8_t> buf;
        uint8_t read_buf[1024];

        while (rclcpp::ok() && is_running_) {
            try {
                if (!serial_conn_.is_open()) {
                    if (serial_conn_.open_port(port_, baud_)) {
                        RCLCPP_INFO(this->get_logger(), "Successfully connected to IMU at %s", port_.c_str());
                        buf.clear();
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Failed to connect to IMU at %s, retrying...", port_.c_str());
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        continue;
                    }
                }

                ssize_t bytes_read = serial_conn_.read_data(read_buf, sizeof(read_buf));
                if (bytes_read > 0) {
                    buf.insert(buf.end(), read_buf, read_buf + bytes_read);
                } else if (bytes_read < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Read error from IMU");
                    serial_conn_.close_port();
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                // Process packets
                while (buf.size() >= 11) {
                    auto it = std::find(buf.begin(), buf.end(), 0x55);
                    if (it == buf.end()) {
                        buf.clear();
                        break;
                    }
                    size_t idx = std::distance(buf.begin(), it);
                    if (idx > 0) {
                        buf.erase(buf.begin(), buf.begin() + idx);
                        continue;
                    }

                    if (buf.size() < 11) {
                        break;
                    }

                    // Checksum verification
                    uint8_t chksum = 0;
                    for (size_t i = 0; i < 10; ++i) {
                        chksum += buf[i];
                    }
                    if (chksum != buf[10]) {
                        buf.erase(buf.begin());
                        continue;
                    }

                    // Extract packet
                    std::vector<uint8_t> packet(buf.begin(), buf.begin() + 11);
                    buf.erase(buf.begin(), buf.begin() + 11);

                    uint8_t p_type = packet[1];
                    auto current_time = this->get_clock()->now();
                    imu_msg_.header.stamp = current_time;

                    if (p_type == 0x51) { // Acceleration
                        int16_t ax_raw = parse_short(packet[2], packet[3]);
                        int16_t ay_raw = parse_short(packet[4], packet[5]);
                        int16_t az_raw = parse_short(packet[6], packet[7]);
                        
                        imu_msg_.linear_acceleration.x = static_cast<double>(ax_raw) / 32768.0 * 16.0 * 9.80665;
                        imu_msg_.linear_acceleration.y = static_cast<double>(ay_raw) / 32768.0 * 16.0 * 9.80665;
                        imu_msg_.linear_acceleration.z = static_cast<double>(az_raw) / 32768.0 * 16.0 * 9.80665;
                        
                    } else if (p_type == 0x52) { // Angular velocity
                        int16_t wx_raw = parse_short(packet[2], packet[3]);
                        int16_t wy_raw = parse_short(packet[4], packet[5]);
                        int16_t wz_raw = parse_short(packet[6], packet[7]);

                        imu_msg_.angular_velocity.x = static_cast<double>(wx_raw) / 32768.0 * 2000.0 * (M_PI / 180.0);
                        imu_msg_.angular_velocity.y = static_cast<double>(wy_raw) / 32768.0 * 2000.0 * (M_PI / 180.0);
                        imu_msg_.angular_velocity.z = static_cast<double>(wz_raw) / 32768.0 * 2000.0 * (M_PI / 180.0);

                    } else if (p_type == 0x53) { // Angle
                        int16_t r_raw = parse_short(packet[2], packet[3]);
                        int16_t p_raw = parse_short(packet[4], packet[5]);
                        int16_t y_raw = parse_short(packet[6], packet[7]);

                        double roll = static_cast<double>(r_raw) / 32768.0 * 180.0;
                        double pitch = static_cast<double>(p_raw) / 32768.0 * 180.0;
                        double yaw = static_cast<double>(y_raw) / 32768.0 * 180.0;

                        double qx, qy, qz, qw;
                        euler_to_quaternion(roll, pitch, yaw, qx, qy, qz, qw);

                        imu_msg_.orientation.x = qx;
                        imu_msg_.orientation.y = qy;
                        imu_msg_.orientation.z = qz;
                        imu_msg_.orientation.w = qw;

                        // Publish orientation when it comes in (usually last in group)
                        imu_pub_->publish(imu_msg_);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error in IMU serial loop: %s", e.what());
                serial_conn_.close_port();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }

    // Parameters
    std::string port_;
    int baud_;
    std::string frame_id_;
    double yaw_offset_deg_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    sensor_msgs::msg::Imu imu_msg_;

    // Threading
    std::thread read_thread_;
    std::atomic<bool> is_running_;
    SerialPort serial_conn_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
