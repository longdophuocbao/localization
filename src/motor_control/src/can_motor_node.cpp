#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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

        this->get_parameter("can_device", can_device_);
        this->get_parameter("motor_id", motor_id_);
        this->get_parameter("torque_limit", torque_limit_);
        this->get_parameter("poll_rate", poll_rate_);

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
        torque_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/motor/cmd_torque", 10, std::bind(&CanMotorNode::torque_callback, this, std::placeholders::_1));
        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/motor/enable", 10, std::bind(&CanMotorNode::enable_callback, this, std::placeholders::_1));

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
        frame.data[0] = 0xA3; // Multi-turn position closed-loop command 1
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;

        // Angle control value
        frame.data[4] = angle_val & 0xFF;
        frame.data[5] = (angle_val >> 8) & 0xFF;
        frame.data[6] = (angle_val >> 16) & 0xFF;
        frame.data[7] = (angle_val >> 24) & 0xFF;

        transmit_frame(frame);
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
        
        // Convert single-turn raw encoder value to position in radians (assuming 14-bit encoder, range 0~16383)
        double pos_rad = (encoder_val_ / 16384.0) * 2.0 * M_PI;
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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

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
