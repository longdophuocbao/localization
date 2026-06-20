#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fstream>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>

#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <cmath>
#include <cstring>
#include <sstream>

class CanMotorNode : public rclcpp::Node {
public:
    CanMotorNode() : Node("can_motor_node") {
        // Declare parameters
        this->declare_parameter<std::string>("can_device", "can0");
        this->declare_parameter<int>("motor_id", 1);
        this->declare_parameter<int>("torque_limit", 1500); // Default speed control torque limit
        this->declare_parameter<double>("poll_rate", 50.0); // Hz for polling status
        this->declare_parameter<int>("default_speed_limit_dps", 180); // Default speed limit for position control
        this->declare_parameter<int>("calibration_torque_threshold", 500); // IQ current limit to register limit
        this->declare_parameter<double>("steering_ratio", 1.0); // Transmission ratio between motor and steering wheel

        this->get_parameter("can_device", can_device_);
        this->get_parameter("motor_id", motor_id_);
        this->get_parameter("torque_limit", torque_limit_);
        this->get_parameter("poll_rate", poll_rate_);
        this->get_parameter("default_speed_limit_dps", default_speed_limit_dps_);
        this->get_parameter("calibration_torque_threshold", calibration_torque_threshold_);
        this->get_parameter("steering_ratio", steering_ratio_);
        speed_limit_dps_ = default_speed_limit_dps_;

        motor_can_id_ = 0x140 + motor_id_;

        RCLCPP_INFO(this->get_logger(), "Starting CAN motor node for device %s, Motor ID %d (CAN ID: 0x%X)",
                    can_device_.c_str(), motor_id_, motor_can_id_);

        // Initialize SocketCAN
        if (!init_can()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize SocketCAN on %s. The node will retry in background.", can_device_.c_str());
        }

        // Subscriptions
        speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor/cmd_speed", 10, std::bind(&CanMotorNode::speed_callback, this, std::placeholders::_1));
        angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor/cmd_angle", 10, std::bind(&CanMotorNode::angle_callback, this, std::placeholders::_1));
        speed_limit_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor/cmd_speed_limit", 10, std::bind(&CanMotorNode::speed_limit_callback, this, std::placeholders::_1));
        torque_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor/cmd_torque", 10, std::bind(&CanMotorNode::torque_callback, this, std::placeholders::_1));
        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/motor/enable", 10, std::bind(&CanMotorNode::enable_callback, this, std::placeholders::_1));

        // Services
        calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/motor/calibrate",
            std::bind(&CanMotorNode::calibrate_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Publishers
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/motor/joint_states", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/motor/status", 10);

        // Timer for polling and publishing
        double timer_period_sec = 1.0 / poll_rate_;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period_sec),
            std::bind(&CanMotorNode::timer_callback, this)
        );

        // Start reading thread
        read_thread_active_ = true;
        read_thread_ = std::thread(&CanMotorNode::read_loop, this);

        // Attempt to auto-enable motor on startup
        RCLCPP_INFO(this->get_logger(), "Sending enable command to motor...");
        send_enable_command(true);
    }

    ~CanMotorNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down. Disabling motor for safety...");
        send_enable_command(false);

        read_thread_active_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }

        if (can_socket_ >= 0) {
            close(can_socket_);
        }
    }

private:
    bool init_can() {
        std::lock_guard<std::mutex> lock(can_mutex_);
        if (can_socket_ >= 0) {
            close(can_socket_);
            can_socket_ = -1;
        }

        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return false;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_device_.c_str(), IFNAMSIZ - 1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN device %s not found (ioctl SIOCGIFINDEX failed)", can_device_.c_str());
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Binding CAN socket failed");
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        // Set non-blocking read
        int flags = fcntl(can_socket_, F_GETFL, 0);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

        can_ready_ = true;
        return true;
    }

    void transmit_frame(const struct can_frame& frame) {
        std::lock_guard<std::mutex> lock(can_mutex_);
        if (!can_ready_ || can_socket_ < 0) {
            return;
        }
        ssize_t bytes_written = write(can_socket_, &frame, sizeof(struct can_frame));
        if (bytes_written != sizeof(struct can_frame)) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "CAN transmission failed.");
        }
    }

    void send_enable_command(bool enable) {
        struct can_frame frame;
        frame.can_id = motor_can_id_;
        frame.can_dlc = 8;
        std::memset(frame.data, 0, 8);
        frame.data[0] = enable ? 0x88 : 0x80; // 0x88: Run, 0x80: Off
        transmit_frame(frame);
    }

    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Convert rad/s to dps (degrees per second)
        double speed_dps = msg->data * 180.0 / M_PI;
        // Value unit: 0.01 dps / LSB
        int32_t speed_val = static_cast<int32_t>(speed_dps * 100.0);

        struct can_frame frame;
        frame.can_id = motor_can_id_;
        frame.can_dlc = 8;
        std::memset(frame.data, 0, 8);
        frame.data[0] = 0xA2; // Speed closed-loop control command
        frame.data[1] = 0x00;
        
        // Torque current limit (iqControl)
        int16_t iq_limit = static_cast<int16_t>(torque_limit_);
        frame.data[2] = iq_limit & 0xFF;
        frame.data[3] = (iq_limit >> 8) & 0xFF;

        // Speed control value
        frame.data[4] = speed_val & 0xFF;
        frame.data[5] = (speed_val >> 8) & 0xFF;
        frame.data[6] = (speed_val >> 16) & 0xFF;
        frame.data[7] = (speed_val >> 24) & 0xFF;

        transmit_frame(frame);
    }

    void angle_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Convert radians to degrees
        double angle_deg = msg->data * 180.0 / M_PI;
        // Value unit: 0.01 degree / LSB
        int32_t angle_val = static_cast<int32_t>(angle_deg * 100.0);

        struct can_frame frame;
        frame.can_id = motor_can_id_;
        frame.can_dlc = 8;
        std::memset(frame.data, 0, 8);
        frame.data[0] = 0xA4; // Multi-turn position closed-loop command 2 (with speed limit)
        frame.data[1] = 0x00;
        
        // Speed limit (dps)
        uint16_t limit = speed_limit_dps_;
        frame.data[2] = limit & 0xFF;
        frame.data[3] = (limit >> 8) & 0xFF;

        // Angle control value
        frame.data[4] = angle_val & 0xFF;
        frame.data[5] = (angle_val >> 8) & 0xFF;
        frame.data[6] = (angle_val >> 16) & 0xFF;
        frame.data[7] = (angle_val >> 24) & 0xFF;

        transmit_frame(frame);
    }

    void speed_limit_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Convert rad/s to dps
        double limit_dps = std::abs(msg->data) * 180.0 / M_PI;
        speed_limit_dps_ = static_cast<uint16_t>(std::round(limit_dps));
        if (speed_limit_dps_ == 0) speed_limit_dps_ = 1;
        RCLCPP_INFO(this->get_logger(), "Updated motor speed limit to %d dps", speed_limit_dps_);
    }

    void torque_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Assuming current command is direct raw iq value (-2048 to 2048) or converted scale
        int16_t iq_val = static_cast<int16_t>(msg->data);
        if (iq_val < -2048) iq_val = -2048;
        if (iq_val > 2048) iq_val = 2048;

        struct can_frame frame;
        frame.can_id = motor_can_id_;
        frame.can_dlc = 8;
        std::memset(frame.data, 0, 8);
        frame.data[0] = 0xA1; // Torque closed-loop control command
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;

        frame.data[4] = iq_val & 0xFF;
        frame.data[5] = (iq_val >> 8) & 0xFF;

        transmit_frame(frame);
    }

    void calibrate_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        (void)request;
        if (is_calibrating_) {
            response->success = false;
            response->message = "Calibration already in progress.";
            return;
        }

        is_calibrating_ = true;
        
        // Start background calibration thread
        std::thread(&CanMotorNode::calibration_loop, this).detach();

        response->success = true;
        response->message = "Calibration routine started in background. Motor will turn left, then right, then center.";
    }

    double unwrap_angle(double current, double reference) {
        double diff = current - reference;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        return reference + diff;
    }

    void send_speed_dps(double speed_dps) {
        int32_t speed_val = static_cast<int32_t>(speed_dps * 100.0);
        struct can_frame frame;
        frame.can_id = motor_can_id_;
        frame.can_dlc = 8;
        std::memset(frame.data, 0, 8);
        frame.data[0] = 0xA2; // Speed closed-loop control command
        frame.data[1] = 0x00;
        int16_t iq_limit = static_cast<int16_t>(torque_limit_);
        frame.data[2] = iq_limit & 0xFF;
        frame.data[3] = (iq_limit >> 8) & 0xFF;
        frame.data[4] = speed_val & 0xFF;
        frame.data[5] = (speed_val >> 8) & 0xFF;
        frame.data[6] = (speed_val >> 16) & 0xFF;
        frame.data[7] = (speed_val >> 24) & 0xFF;
        transmit_frame(frame);
    }

    void send_poll_cmd() {
        struct can_frame frame;
        frame.can_id = motor_can_id_;
        frame.can_dlc = 8;
        std::memset(frame.data, 0, 8);
        frame.data[0] = 0x9C; // Read status 2
        transmit_frame(frame);
    }

    void send_position_command(double angle_rad, uint16_t speed_limit_dps) {
        double angle_deg = angle_rad * 180.0 / M_PI;
        int32_t angle_val = static_cast<int32_t>(angle_deg * 100.0);

        struct can_frame frame;
        frame.can_id = motor_can_id_;
        frame.can_dlc = 8;
        std::memset(frame.data, 0, 8);
        frame.data[0] = 0xA4; // Position control with speed limit
        frame.data[1] = 0x00;
        frame.data[2] = speed_limit_dps & 0xFF;
        frame.data[3] = (speed_limit_dps >> 8) & 0xFF;
        frame.data[4] = angle_val & 0xFF;
        frame.data[5] = (angle_val >> 8) & 0xFF;
        frame.data[6] = (angle_val >> 16) & 0xFF;
        frame.data[7] = (angle_val >> 24) & 0xFF;
        transmit_frame(frame);
    }

    void calibration_loop() {
        RCLCPP_INFO(this->get_logger(), "=== [Calibration] Starting steering calibration ===");
        
        // Ensure motor is enabled
        send_enable_command(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Record start position
        double start_pos = (encoder_val_ / 65536.0) * 2.0 * M_PI;
        double left_limit = start_pos;
        double right_limit = start_pos;

        // Phase 1: Rotate LEFT (positive direction)
        RCLCPP_INFO(this->get_logger(), "[Calibration] Phase 1: Rotating LEFT slowly to detect limit...");
        bool left_success = false;
        auto phase1_start = this->get_clock()->now();
        
        while (rclcpp::ok() && is_calibrating_) {
            // Check timeout (15 seconds)
            if ((this->get_clock()->now() - phase1_start).seconds() > 15.0) {
                RCLCPP_ERROR(this->get_logger(), "[Calibration] Phase 1 TIMEOUT!");
                break;
            }

            // Send speed 20 dps (positive)
            send_speed_dps(20.0);
            
            // Poll state
            send_poll_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            double cur_pos = unwrap_angle((encoder_val_ / 65536.0) * 2.0 * M_PI, start_pos);
            int16_t cur_torque = std::abs(iq_current_);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[Calibration] LEFT - Angle: %.2f deg, Torque: %d",
                                 cur_pos * 180.0 / M_PI, cur_torque);

            if (cur_torque >= calibration_torque_threshold_) {
                left_limit = cur_pos;
                left_success = true;
                send_speed_dps(0.0);
                RCLCPP_INFO(this->get_logger(), "[Calibration] LEFT limit detected at %.2f deg (Torque: %d)",
                            left_limit * 180.0 / M_PI, cur_torque);
                break;
            }
        }

        if (!left_success) {
            RCLCPP_ERROR(this->get_logger(), "[Calibration] Calibration aborted: LEFT limit not found.");
            send_speed_dps(0.0);
            is_calibrating_ = false;
            return;
        }

        // Wait for motor to settle
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Phase 2: Rotate RIGHT (negative direction)
        RCLCPP_INFO(this->get_logger(), "[Calibration] Phase 2: Rotating RIGHT slowly to detect limit...");
        bool right_success = false;
        auto phase2_start = this->get_clock()->now();

        while (rclcpp::ok() && is_calibrating_) {
            // Check timeout (25 seconds - needs to travel from Left limit all the way to Right limit)
            if ((this->get_clock()->now() - phase2_start).seconds() > 25.0) {
                RCLCPP_ERROR(this->get_logger(), "[Calibration] Phase 2 TIMEOUT!");
                break;
            }

            // Send speed -20 dps (negative)
            send_speed_dps(-20.0);
            
            // Poll state
            send_poll_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            double cur_pos = unwrap_angle((encoder_val_ / 65536.0) * 2.0 * M_PI, start_pos);
            int16_t cur_torque = std::abs(iq_current_);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[Calibration] RIGHT - Angle: %.2f deg, Torque: %d",
                                 cur_pos * 180.0 / M_PI, cur_torque);

            if (cur_torque >= calibration_torque_threshold_) {
                right_limit = cur_pos;
                right_success = true;
                send_speed_dps(0.0);
                RCLCPP_INFO(this->get_logger(), "[Calibration] RIGHT limit detected at %.2f deg (Torque: %d)",
                            right_limit * 180.0 / M_PI, cur_torque);
                break;
            }
        }

        if (!right_success) {
            RCLCPP_ERROR(this->get_logger(), "[Calibration] Calibration aborted: RIGHT limit not found.");
            send_speed_dps(0.0);
            is_calibrating_ = false;
            return;
        }

        // Wait to settle
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Phase 3: Calculate Center and return
        double center_pos = (left_limit + right_limit) / 2.0;
        RCLCPP_INFO(this->get_logger(), "[Calibration] Calibration success!");
        RCLCPP_INFO(this->get_logger(), "[Calibration]   LEFT Limit:  %.2f deg", left_limit * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "[Calibration]   RIGHT Limit: %.2f deg", right_limit * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "[Calibration]   CENTER Pos:  %.2f deg (relative to start)", (center_pos - start_pos) * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "[Calibration] Moving to Center position...");

        // Move to center smoothly using speed limit 30 dps
        auto move_start = this->get_clock()->now();
        while (rclcpp::ok() && is_calibrating_) {
            if ((this->get_clock()->now() - move_start).seconds() > 10.0) {
                RCLCPP_WARN(this->get_logger(), "[Calibration] Move to center timeout.");
                break;
            }

            send_position_command(center_pos, 30);
            send_poll_cmd();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            double cur_pos = unwrap_angle((encoder_val_ / 65536.0) * 2.0 * M_PI, start_pos);
            double diff = std::abs(cur_pos - center_pos);
            if (diff < (1.0 * M_PI / 180.0)) { // within 1 degree
                RCLCPP_INFO(this->get_logger(), "[Calibration] Center position reached.");
                break;
            }
        }

        // Calculate max_steer_angle (wheel angle in radians)
        double motor_limit_range = std::abs(left_limit - right_limit) / 2.0;
        double max_steer = motor_limit_range / steering_ratio_;
        
        RCLCPP_INFO(this->get_logger(), "[Calibration] Calculated max_steer_angle: %.4f rad (%.2f deg)",
                    max_steer, max_steer * 180.0 / M_PI);

        // 1. Save to yaml file
        save_max_steer_angle_to_file(max_steer);

        // 2. Try to update running pure_pursuit_node parameter dynamically
        try {
            auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "pure_pursuit_node");
            if (param_client->wait_for_service(std::chrono::seconds(1))) {
                param_client->set_parameters({
                    rclcpp::Parameter("max_steer_angle", max_steer)
                });
                RCLCPP_INFO(this->get_logger(), "[Calibration] Successfully set max_steer_angle on running pure_pursuit_node.");
            } else {
                RCLCPP_WARN(this->get_logger(), "[Calibration] pure_pursuit_node not active. Dynamic parameter update skipped.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[Calibration] Error setting parameter: %s", e.what());
        }

        is_calibrating_ = false;
        RCLCPP_INFO(this->get_logger(), "=== [Calibration] Calibration routine finished successfully ===");
    }

    void save_max_steer_angle_to_file(double max_steer) {
        std::string file_path = "/home/long2/localization/src/motor_control/config/motor_control_params.yaml";
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open parameter file for reading: %s", file_path.c_str());
            return;
        }

        std::vector<std::string> lines;
        std::string line;
        bool updated = false;

        while (std::getline(file, line)) {
            if (line.find("max_steer_angle:") != std::string::npos) {
                size_t colon_idx = line.find(":");
                if (colon_idx != std::string::npos) {
                    std::ostringstream ss;
                    ss << line.substr(0, colon_idx + 1) << " " << max_steer;
                    line = ss.str();
                    updated = true;
                }
            }
            lines.push_back(line);
        }
        file.close();

        if (updated) {
            std::ofstream out_file(file_path);
            if (out_file.is_open()) {
                for (const auto& l : lines) {
                    out_file << l << "\n";
                }
                out_file.close();
                RCLCPP_INFO(this->get_logger(), "Successfully saved new max_steer_angle (%.4f rad) to %s", max_steer, file_path.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Could not open parameter file for writing: %s", file_path.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Could not find max_steer_angle parameter in %s", file_path.c_str());
        }
    }

    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received motor enable command: %s", msg->data ? "TRUE" : "FALSE");
        send_enable_command(msg->data);
    }

    void timer_callback() {
        if (!can_ready_ || can_socket_ < 0) {
            // Retry connecting to SocketCAN in background
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "CAN port not ready, retrying connection...");
            init_can();
            return;
        }

        // Poll motor state alternately to avoid bus congestion
        static int poll_cycle = 0;
        struct can_frame frame;
        frame.can_id = motor_can_id_;
        frame.can_dlc = 8;
        std::memset(frame.data, 0, 8);

        if (poll_cycle == 0) {
            frame.data[0] = 0x9C; // Read status 2 (speed, angle, temp, torque)
            poll_cycle = 1;
        } else {
            frame.data[0] = 0x9A; // Read status 1 (voltage, current, state, error)
            poll_cycle = 0;
        }
        transmit_frame(frame);

        // Publish current state to ROS
        publish_feedback();
    }

    void publish_feedback() {
        // JointState
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name = {"ktech_motor_joint"};
        
        // Convert single-turn raw encoder value to position in radians (assuming 16-bit encoder, range 0~65535)
        double pos_rad = (encoder_val_ / 65536.0) * 2.0 * M_PI;
        joint_state.position = {pos_rad};

        // Convert speed (dps) to velocity (rad/s)
        double vel_rads = speed_dps_ * M_PI / 180.0;
        joint_state.velocity = {vel_rads};

        // Effort as iq current in raw units or proportional Amps
        joint_state.effort = {static_cast<double>(iq_current_)};
        joint_state_pub_->publish(joint_state);

        // Diagnostic / Status info as JSON
        auto status_msg = std_msgs::msg::String();
        std::stringstream ss;
        ss << "{"
           << "\"motor_id\":" << motor_id_ << ","
           << "\"temperature_c\":" << static_cast<int>(temperature_) << ","
           << "\"voltage_v\":" << voltage_ << ","
           << "\"current_a\":" << current_ << ","
           << "\"enabled\":" << (motor_state_ == 0x00 ? "true" : "false") << ","
           << "\"error_code\":\"0x" << std::hex << static_cast<int>(error_state_) << std::dec << "\""
           << "}";
        status_msg.data = ss.str();
        status_pub_->publish(status_msg);

        // Log warnings on motor errors
        if (error_state_ != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Motor reports error! Code: 0x%02X (Voltage: %.1fV, Temp: %d C)",
                                 error_state_, voltage_, temperature_);
        }
    }

    void read_loop() {
        struct can_frame frame;
        while (read_thread_active_ && rclcpp::ok()) {
            if (can_socket_ < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
            if (nbytes > 0) {
                // Check if the reply is from our configured motor
                if (frame.can_id == motor_can_id_) {
                    uint8_t cmd_byte = frame.data[0];

                    // Check if response format is Status 2 (replies for 0x9C, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6)
                    if (cmd_byte == 0x9C || cmd_byte == 0xA2 || cmd_byte == 0xA3 || 
                        cmd_byte == 0xA4 || cmd_byte == 0xA5 || cmd_byte == 0xA6) {
                        
                        temperature_ = static_cast<int8_t>(frame.data[1]);
                        iq_current_ = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
                        speed_dps_ = static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]);
                        encoder_val_ = static_cast<uint16_t>((frame.data[7] << 8) | frame.data[6]);

                    }
                    // Check if response format is Status 1 (replies for 0x9A, 0x9B)
                    else if (cmd_byte == 0x9A || cmd_byte == 0x9B) {
                        temperature_ = static_cast<int8_t>(frame.data[1]);
                        voltage_ = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]) * 0.01;
                        current_ = static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]) * 0.01;
                        motor_state_ = frame.data[6];
                        error_state_ = frame.data[7];
                    }
                }
            } else {
                // Non-blocking read returns -1 with EAGAIN when no data is available
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    // Parameters
    std::string can_device_;
    int motor_id_;
    int torque_limit_;
    double poll_rate_;

    // Connection
    int can_socket_ = -1;
    uint32_t motor_can_id_ = 0;
    bool can_ready_ = false;
    std::mutex can_mutex_;

    // Reading thread
    std::thread read_thread_;
    bool read_thread_active_ = false;

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_limit_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Calibration settings
    int calibration_torque_threshold_ = 500;
    bool is_calibrating_ = false;
    double steering_ratio_ = 1.0;

    // Speed limit parameters
    int default_speed_limit_dps_ = 180;
    uint16_t speed_limit_dps_ = 180;

    // Motor State variables (updated from CAN frame reads)
    int8_t temperature_ = 0;
    int16_t iq_current_ = 0;
    int16_t speed_dps_ = 0;
    uint16_t encoder_val_ = 0;
    double voltage_ = 0.0;
    double current_ = 0.0;
    uint8_t motor_state_ = 0;
    uint8_t error_state_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
