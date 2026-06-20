#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <cstring>
#include <thread>
#include <mutex>
#include "serial_port.hpp"
#include "localization_system/srv/calibrate_imu.hpp"
#include <atomic>

class ImuDriverNode : public rclcpp::Node {
public:
    ImuDriverNode() : Node("imu_driver") {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyAMA0");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<double>("yaw_offset_deg", 90.0);

        // Declare calibration parameters (with default low-noise assumptions)
        this->declare_parameter<double>("linear_acc_var_x", 0.01);
        this->declare_parameter<double>("linear_acc_var_y", 0.01);
        this->declare_parameter<double>("linear_acc_var_z", 0.01);
        
        this->declare_parameter<double>("angular_vel_var_x", 0.001);
        this->declare_parameter<double>("angular_vel_var_y", 0.001);
        this->declare_parameter<double>("angular_vel_var_z", 0.001);

        this->declare_parameter<double>("gyro_bias_x", 0.0);
        this->declare_parameter<double>("gyro_bias_y", 0.0);
        this->declare_parameter<double>("gyro_bias_z", 0.0);

        this->get_parameter("port", port_);
        this->get_parameter("baud", baud_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("yaw_offset_deg", yaw_offset_deg_);

        RCLCPP_INFO(this->get_logger(), "Starting IMU driver on %s @ %d baud", port_.c_str(), baud_);

        // Publisher
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        // Pre-configure IMU message
        imu_msg_.header.frame_id = frame_id_;

        // Initial setup of covariances
        update_covariances_from_parameters();

        // Create Calibration Service (standard executor group is fine since read_loop runs in a separate thread)
        calibrate_srv_ = this->create_service<localization_system::srv::CalibrateImu>(
            "calibrate_imu",
            std::bind(&ImuDriverNode::calibrate_imu_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Start serial read thread
        run_read_thread_ = true;
        read_thread_ = std::thread(&ImuDriverNode::read_loop, this);
    }

    ~ImuDriverNode() {
        run_read_thread_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        serial_conn_.close_port();
        RCLCPP_INFO(this->get_logger(), "IMU driver stopped cleanly.");
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
        qw = std::cos(r/2) * std::cos(p/2) * std::cos(y/2) + std::sin(r/2) * std::cos(p/2) * std::cos(y/2);
    }

    void update_covariances_from_parameters() {
        double acc_var_x, acc_var_y, acc_var_z;
        double gyro_var_x, gyro_var_y, gyro_var_z;

        this->get_parameter("linear_acc_var_x", acc_var_x);
        this->get_parameter("linear_acc_var_y", acc_var_y);
        this->get_parameter("linear_acc_var_z", acc_var_z);
        this->get_parameter("angular_vel_var_x", gyro_var_x);
        this->get_parameter("angular_vel_var_y", gyro_var_y);
        this->get_parameter("angular_vel_var_z", gyro_var_z);

        imu_msg_.orientation_covariance = {
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        };
        
        imu_msg_.angular_velocity_covariance = {
            gyro_var_x, 0.0, 0.0,
            0.0, gyro_var_y, 0.0,
            0.0, 0.0, gyro_var_z
        };

        imu_msg_.linear_acceleration_covariance = {
            acc_var_x, 0.0, 0.0,
            0.0, acc_var_y, 0.0,
            0.0, 0.0, acc_var_z
        };
    }

    void calibrate_imu_callback(
        const std::shared_ptr<localization_system::srv::CalibrateImu::Request> request,
        std::shared_ptr<localization_system::srv::CalibrateImu::Response> response)
    {
        if (is_calibrating_) {
            response->success = false;
            response->message = "IMU Calibration is already in progress.";
            return;
        }

        double duration = request->duration_seconds > 0.0 ? request->duration_seconds : 5.0;
        RCLCPP_INFO(this->get_logger(), "Starting IMU calibration for %.2f seconds. Keep the tractor stationary.", duration);

        {
            std::lock_guard<std::mutex> lock(calib_mutex_);
            calib_ax_.clear(); calib_ay_.clear(); calib_az_.clear();
            calib_wx_.clear(); calib_wy_.clear(); calib_wz_.clear();
            is_calibrating_ = true;
        }

        // Safe sleep: the read_loop thread runs independently and gathers data.
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000)));

        {
            std::lock_guard<std::mutex> lock(calib_mutex_);
            is_calibrating_ = false;
        }

        std::vector<double> ax_copy, ay_copy, az_copy;
        std::vector<double> wx_copy, wy_copy, wz_copy;
        {
            std::lock_guard<std::mutex> lock(calib_mutex_);
            ax_copy = calib_ax_;
            ay_copy = calib_ay_;
            az_copy = calib_az_;
            wx_copy = calib_wx_;
            wy_copy = calib_wy_;
            wz_copy = calib_wz_;
        }

        if (ax_copy.size() < 20 || wx_copy.size() < 20) {
            response->success = false;
            response->message = "Failed to collect enough samples (" + std::to_string(ax_copy.size()) + "/20). Check sensor connection.";
            return;
        }

        // 1. Calculate Acceleration Mean and Variance
        double ax_mean = std::accumulate(ax_copy.begin(), ax_copy.end(), 0.0) / ax_copy.size();
        double ay_mean = std::accumulate(ay_copy.begin(), ay_copy.end(), 0.0) / ay_copy.size();
        double az_mean = std::accumulate(az_copy.begin(), az_copy.end(), 0.0) / az_copy.size();

        double ax_var = 0.0; for (double val : ax_copy) ax_var += (val - ax_mean) * (val - ax_mean);
        ax_var /= (ax_copy.size() - 1);
        double ay_var = 0.0; for (double val : ay_copy) ay_var += (val - ay_mean) * (val - ay_mean);
        ay_var /= (ay_copy.size() - 1);
        double az_var = 0.0; for (double val : az_copy) az_var += (val - az_mean) * (val - az_mean);
        az_var /= (az_copy.size() - 1);

        // 2. Calculate Gyro Mean and Variance
        double wx_mean = std::accumulate(wx_copy.begin(), wx_copy.end(), 0.0) / wx_copy.size();
        double wy_mean = std::accumulate(wy_copy.begin(), wy_copy.end(), 0.0) / wy_copy.size();
        double wz_mean = std::accumulate(wz_copy.begin(), wz_copy.end(), 0.0) / wz_copy.size();

        double wx_var = 0.0; for (double val : wx_copy) wx_var += (val - wx_mean) * (val - wx_mean);
        wx_var /= (wx_copy.size() - 1);
        double wy_var = 0.0; for (double val : wy_copy) wy_var += (val - wy_mean) * (val - wy_mean);
        wy_var /= (wy_copy.size() - 1);
        double wz_var = 0.0; for (double val : wz_copy) wz_var += (val - wz_mean) * (val - wz_mean);
        wz_var /= (wz_copy.size() - 1);

        // 3. Update dynamic ROS parameters
        this->set_parameter(rclcpp::Parameter("linear_acc_var_x", ax_var));
        this->set_parameter(rclcpp::Parameter("linear_acc_var_y", ay_var));
        this->set_parameter(rclcpp::Parameter("linear_acc_var_z", az_var));
        
        this->set_parameter(rclcpp::Parameter("angular_vel_var_x", wx_var));
        this->set_parameter(rclcpp::Parameter("angular_vel_var_y", wy_var));
        this->set_parameter(rclcpp::Parameter("angular_vel_var_z", wz_var));

        this->set_parameter(rclcpp::Parameter("gyro_bias_x", wx_mean));
        this->set_parameter(rclcpp::Parameter("gyro_bias_y", wy_mean));
        this->set_parameter(rclcpp::Parameter("gyro_bias_z", wz_mean));

        response->success = true;
        response->message = "IMU calibration succeeded. Noise thresholds and bias parameters updated.";
        response->acc_mean = {ax_mean, ay_mean, az_mean};
        response->acc_var = {ax_var, ay_var, az_var};
        response->gyro_mean = {wx_mean, wy_mean, wz_mean};
        response->gyro_var = {wx_var, wy_var, wz_var};

        RCLCPP_INFO(this->get_logger(), "IMU Calibrated successfully over %zu samples.", ax_copy.size());
        RCLCPP_INFO(this->get_logger(), "Accel Variances (noise threshold): [%.6f, %.6f, %.6f]", ax_var, ay_var, az_var);
        RCLCPP_INFO(this->get_logger(), "Gyro Biases: [%.6f, %.6f, %.6f], Gyro Variances: [%.6f, %.6f, %.6f]",
                    wx_mean, wy_mean, wz_mean, wx_var, wy_var, wz_var);
    }

    void read_loop() {
        while (run_read_thread_ && rclcpp::ok()) {
            // Connect if port is not open
            if (!serial_conn_.is_open()) {
                if (serial_conn_.open_port(port_, baud_)) {
                    RCLCPP_INFO(this->get_logger(), "Successfully connected to IMU at %s", port_.c_str());
                    buf_.clear();
                } else {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                          "Failed to connect to IMU at %s, retrying...", port_.c_str());
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
            }

            // Read available serial data
            uint8_t read_buf[1024];
            ssize_t bytes_read = serial_conn_.read_data(read_buf, sizeof(read_buf));
            if (bytes_read > 0) {
                buf_.insert(buf_.end(), read_buf, read_buf + bytes_read);
            } else if (bytes_read < 0) {
                RCLCPP_ERROR(this->get_logger(), "Read error from IMU, closing port.");
                serial_conn_.close_port();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            } else {
                // No data available, sleep briefly to avoid pegging the CPU
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            // Process WT901 packets (each packet is 11 bytes starting with 0x55)
            while (buf_.size() >= 11) {
                auto it = std::find(buf_.begin(), buf_.end(), 0x55);
                if (it == buf_.end()) {
                    buf_.clear();
                    break;
                }
                size_t idx = std::distance(buf_.begin(), it);
                if (idx > 0) {
                    buf_.erase(buf_.begin(), buf_.begin() + idx);
                    continue;
                }

                if (buf_.size() < 11) {
                    break;
                }

                // Checksum verification
                uint8_t chksum = 0;
                for (size_t i = 0; i < 10; ++i) {
                    chksum += buf_[i];
                }
                if (chksum != buf_[10]) {
                    buf_.erase(buf_.begin());
                    continue;
                }

                // Extract packet
                std::vector<uint8_t> packet(buf_.begin(), buf_.begin() + 11);
                buf_.erase(buf_.begin(), buf_.begin() + 11);

                uint8_t p_type = packet[1];
                auto current_time = this->now();
                imu_msg_.header.stamp = current_time;

                if (p_type == 0x51) { // Acceleration
                    int16_t ax_raw = parse_short(packet[2], packet[3]);
                    int16_t ay_raw = parse_short(packet[4], packet[5]);
                    int16_t az_raw = parse_short(packet[6], packet[7]);
                    
                    double ax = static_cast<double>(ax_raw) / 32768.0 * 16.0 * 9.80665;
                    double ay = static_cast<double>(ay_raw) / 32768.0 * 16.0 * 9.80665;
                    double az = static_cast<double>(az_raw) / 32768.0 * 16.0 * 9.80665;
                    
                    imu_msg_.linear_acceleration.x = ax;
                    imu_msg_.linear_acceleration.y = ay;
                    imu_msg_.linear_acceleration.z = az;

                    {
                        std::lock_guard<std::mutex> lock(calib_mutex_);
                        if (is_calibrating_) {
                            calib_ax_.push_back(ax);
                            calib_ay_.push_back(ay);
                            calib_az_.push_back(az);
                        }
                    }
                    
                } else if (p_type == 0x52) { // Angular velocity
                    int16_t wx_raw = parse_short(packet[2], packet[3]);
                    int16_t wy_raw = parse_short(packet[4], packet[5]);
                    int16_t wz_raw = parse_short(packet[6], packet[7]);

                    double gyro_bias_x, gyro_bias_y, gyro_bias_z;
                    this->get_parameter("gyro_bias_x", gyro_bias_x);
                    this->get_parameter("gyro_bias_y", gyro_bias_y);
                    this->get_parameter("gyro_bias_z", gyro_bias_z);

                    double wx = static_cast<double>(wx_raw) / 32768.0 * 2000.0 * (M_PI / 180.0);
                    double wy = static_cast<double>(wy_raw) / 32768.0 * 2000.0 * (M_PI / 180.0);
                    double wz = static_cast<double>(wz_raw) / 32768.0 * 2000.0 * (M_PI / 180.0);

                    imu_msg_.angular_velocity.x = wx - gyro_bias_x;
                    imu_msg_.angular_velocity.y = wy - gyro_bias_y;
                    imu_msg_.angular_velocity.z = wz - gyro_bias_z;

                    {
                        std::lock_guard<std::mutex> lock(calib_mutex_);
                        if (is_calibrating_) {
                            calib_wx_.push_back(wx);
                            calib_wy_.push_back(wy);
                            calib_wz_.push_back(wz);
                        }
                    }

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

                    // Update message covariance values from parameters dynamically
                    update_covariances_from_parameters();

                    // Publish complete IMU message
                    imu_pub_->publish(imu_msg_);
                }
            }
        }
    }

    // Parameters
    std::string port_;
    int baud_;
    std::string frame_id_;
    double yaw_offset_deg_;

    // Calibration states
    bool is_calibrating_ = false;
    std::mutex calib_mutex_;
    
    std::vector<double> calib_ax_, calib_ay_, calib_az_;
    std::vector<double> calib_wx_, calib_wy_, calib_wz_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    sensor_msgs::msg::Imu imu_msg_;
    rclcpp::Service<localization_system::srv::CalibrateImu>::SharedPtr calibrate_srv_;

    // Threading
    std::thread read_thread_;
    std::atomic<bool> run_read_thread_;

    // Serial
    SerialPort serial_conn_;
    std::vector<uint8_t> buf_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
