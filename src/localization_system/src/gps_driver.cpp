#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <cstring>
#include "serial_port.hpp"

#pragma pack(push, 1)
struct UbxNavPvt {
    uint32_t iTOW;       // GPS time of week of the navigation epoch (ms)
    uint16_t year;       // Year (UTC)
    uint8_t  month;      // Month, range 1..12 (UTC)
    uint8_t  day;        // Day of month, range 1..31 (UTC)
    uint8_t  hour;       // Hour of day, range 0..23 (UTC)
    uint8_t  min;        // Minute of hour, range 0..59 (UTC)
    uint8_t  sec;        // Seconds of minute, range 0..60 (UTC)
    uint8_t  valid;      // Validity flags
    uint32_t tAcc;       // Time accuracy estimate (ns)
    int32_t  nano;       // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
    uint8_t  fixType;    // GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
    uint8_t  flags;      // Navigation status flags
    uint8_t  flags2;     // Additional flags
    uint8_t  numSV;      // Number of satellites used in Nav Solution
    int32_t  lon;        // Longitude (1e-7 deg)
    int32_t  lat;        // Latitude (1e-7 deg)
    int32_t  height;     // Height above ellipsoid (mm)
    int32_t  hMSL;       // Height above mean sea level (mm)
    uint32_t hAcc;       // Horizontal accuracy estimate (mm)
    uint32_t vAcc;       // Vertical accuracy estimate (mm)
};
#pragma pack(pop)

class GpsDriverNode : public rclcpp::Node {
public:
    GpsDriverNode() : Node("gps_driver") {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM4");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<std::string>("frame_id", "gps_link");

        this->get_parameter("port", port_);
        this->get_parameter("baud", baud_);
        this->get_parameter("frame_id", frame_id_);

        RCLCPP_INFO(this->get_logger(), "Starting GPS driver on %s @ %d baud", port_.c_str(), baud_);

        // Publisher
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

        // Pre-configure GPS message
        fix_msg_.header.frame_id = frame_id_;

        // Start ROS 2 timer callback at 100 Hz (every 10ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GpsDriverNode::timer_callback, this)
        );
    }

    ~GpsDriverNode() {
        serial_conn_.close_port();
        RCLCPP_INFO(this->get_logger(), "GPS driver stopped cleanly.");
    }

private:
    void timer_callback() {
        // Attempt connection if not open
        if (!serial_conn_.is_open()) {
            if (serial_conn_.open_port(port_, baud_)) {
                RCLCPP_INFO(this->get_logger(), "Successfully connected to GPS at %s", port_.c_str());
                buf_.clear();
            } else {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                      "Failed to connect to GPS at %s, retrying...", port_.c_str());
                return;
            }
        }

        // Read available data
        uint8_t read_buf[1024];
        ssize_t bytes_read = serial_conn_.read_data(read_buf, sizeof(read_buf));
        if (bytes_read > 0) {
            buf_.insert(buf_.end(), read_buf, read_buf + bytes_read);
        } else if (bytes_read < 0) {
            RCLCPP_ERROR(this->get_logger(), "Read error from GPS, closing port.");
            serial_conn_.close_port();
            return;
        } else {
            // No data currently available (normal for non-blocking read)
            return;
        }

        // Process UBX packets
        while (buf_.size() >= 100) {
            // Find UBX sync char 0xb5 0x62
            size_t idx = std::string::npos;
            for (size_t i = 0; i + 1 < buf_.size(); ++i) {
                if (buf_[i] == 0xb5 && buf_[i+1] == 0x62) {
                    idx = i;
                    break;
                }
            }

            if (idx == std::string::npos) {
                // Clear all except the last byte if it might be the sync start
                if (!buf_.empty() && buf_.back() == 0xb5) {
                    uint8_t last = buf_.back();
                    buf_.clear();
                    buf_.push_back(last);
                } else {
                    buf_.clear();
                }
                break;
            }

            if (idx > 0) {
                buf_.erase(buf_.begin(), buf_.begin() + idx);
                continue;
            }

            if (buf_.size() < 6) {
                break;
            }

            uint16_t payload_len = (buf_[5] << 8) | buf_[4];
            size_t packet_len = 6 + payload_len + 2;

            if (buf_.size() < packet_len) {
                break; // Need more data
            }

            std::vector<uint8_t> packet(buf_.begin(), buf_.begin() + packet_len);
            buf_.erase(buf_.begin(), buf_.begin() + packet_len);

            // Verify Fletcher checksum
            uint8_t ck_a = 0;
            uint8_t ck_b = 0;
            for (size_t i = 2; i < packet.size() - 2; ++i) {
                ck_a = (ck_a + packet[i]) & 0xFF;
                ck_b = (ck_b + ck_a) & 0xFF;
            }

            if (ck_a != packet[packet.size() - 2] || ck_b != packet[packet.size() - 1]) {
                continue;
            }

            uint8_t cls = packet[2];
            uint8_t msg_id = packet[3];

            if (cls == 0x01 && msg_id == 0x07) { // UBX-NAV-PVT
                if (payload_len >= 48) {
                    UbxNavPvt pvt;
                    std::memcpy(&pvt, &packet[6], sizeof(UbxNavPvt));

                    auto current_time = this->get_clock()->now();
                    fix_msg_.header.stamp = current_time;

                    // Set coordinates
                    fix_msg_.latitude = pvt.lat / 10000000.0;
                    fix_msg_.longitude = pvt.lon / 10000000.0;
                    fix_msg_.altitude = pvt.height / 1000.0; // Height above ellipsoid

                    // Set status
                    // ZED-F9P RTK status check (flags: 0x40 is RTK float, 0x80 is RTK fixed)
                    bool is_rtk = (pvt.flags & 0xC0) != 0;
                    if (pvt.fixType >= 2) {
                        if (is_rtk) {
                            fix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                        } else {
                            fix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                        }
                    } else {
                        fix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    }

                    fix_msg_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

                    // Convert hAcc/vAcc to position covariance (diagonal)
                    double h_var = std::pow(pvt.hAcc / 1000.0, 2);
                    double v_var = std::pow(pvt.vAcc / 1000.0, 2);

                    // Ensure variance is non-zero
                    h_var = std::max(h_var, 0.0001);
                    v_var = std::max(v_var, 0.0001);

                    fix_msg_.position_covariance = {
                        h_var, 0.0, 0.0,
                        0.0, h_var, 0.0,
                        0.0, 0.0, v_var
                    };
                    fix_msg_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

                    gps_pub_->publish(fix_msg_);
                }
            }
        }
    }

    // Parameters
    std::string port_;
    int baud_;
    std::string frame_id_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    sensor_msgs::msg::NavSatFix fix_msg_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Serial
    SerialPort serial_conn_;
    std::vector<uint8_t> buf_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
