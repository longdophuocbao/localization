#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <thread>
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

struct UbxNavHpposllh {
    uint8_t  version;       // Message version (0x00)
    uint8_t  reserved1[3];  // Reserved
    uint32_t iTOW;          // GPS time of week [ms]
    int32_t  lon;           // Longitude [deg * 1e-7]
    int32_t  lat;           // Latitude [deg * 1e-7]
    int32_t  height;        // Height above ellipsoid [mm]
    int32_t  hMSL;          // Height above mean sea level [mm]
    int8_t   lonHp;         // Longitude high precision [deg * 1e-9, range -99..99]
    int8_t   latHp;         // Latitude high precision [deg * 1e-9, range -99..99]
    int8_t   heightHp;      // Height HP [0.1 mm, range -9..9]
    int8_t   hMSLHp;        // Height MSL HP [0.1 mm, range -9..9]
    uint32_t hAcc;          // Horizontal accuracy estimate [0.1 mm]
    uint32_t vAcc;          // Vertical accuracy estimate [0.1 mm]
};
#pragma pack(pop)

class GpsDriverNode : public rclcpp::Node {
public:
    GpsDriverNode() : Node("gps_driver") {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00");
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
    void send_ubx_cfg_msg(uint8_t target_class, uint8_t target_id, uint8_t rate) {
        std::vector<uint8_t> packet = {
            0xb5, 0x62,       // Sync chars
            0x06, 0x01,       // Class: CFG, ID: MSG
            0x08, 0x00,       // Length: 8 bytes
            target_class,
            target_id,
            rate, rate, rate, rate, rate, rate  // Rates on all 6 interfaces
        };

        // Compute Fletcher checksum
        uint8_t ck_a = 0;
        uint8_t ck_b = 0;
        for (size_t i = 2; i < packet.size(); ++i) {
            ck_a = (ck_a + packet[i]) & 0xFF;
            ck_b = (ck_b + ck_a) & 0xFF;
        }
        packet.push_back(ck_a);
        packet.push_back(ck_b);

        if (serial_conn_.is_open()) {
            serial_conn_.write_data(packet.data(), packet.size());
            RCLCPP_INFO(this->get_logger(), "Sent UBX-CFG-MSG to set rate of class 0x%02X ID 0x%02X to %d", target_class, target_id, rate);
        }
    }

    void timer_callback() {
        // Attempt connection if not open
        if (!serial_conn_.is_open()) {
            if (serial_conn_.open_port(port_, baud_)) {
                RCLCPP_INFO(this->get_logger(), "Successfully connected to GPS at %s", port_.c_str());
                buf_.clear();

                // Configure GPS receiver to output NAV-PVT and NAV-HPPOSLLH
                send_ubx_cfg_msg(0x01, 0x07, 1); // Enable UBX-NAV-PVT
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                send_ubx_cfg_msg(0x01, 0x14, 1); // Enable UBX-NAV-HPPOSLLH
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

        // Process UBX packets (minimum header + checksum is 8 bytes)
        while (buf_.size() >= 8) {
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

            // Header starts at buf_[0]
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

                    last_pvt_fix_type_ = pvt.fixType;
                    last_pvt_flags_ = pvt.flags;
                    last_pvt_iTOW_ = pvt.iTOW;
                    last_pvt_time_ = this->get_clock()->now();

                    // Check if we have received a high precision fix recently.
                    // If not, publish from standard PVT message as a fallback.
                    auto now = this->get_clock()->now();
                    double dt = (now - last_hpposllh_time_).seconds();
                    if (dt > 2.0) {
                        fix_msg_.header.stamp = now;

                        // Set coordinates
                        fix_msg_.latitude = pvt.lat / 10000000.0;
                        fix_msg_.longitude = pvt.lon / 10000000.0;
                        fix_msg_.altitude = pvt.height / 1000.0; // Height above ellipsoid

                        // Set status
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
            } else if (cls == 0x01 && msg_id == 0x14) { // UBX-NAV-HPPOSLLH
                if (payload_len >= 36) {
                    UbxNavHpposllh hppos;
                    std::memcpy(&hppos, &packet[6], sizeof(UbxNavHpposllh));

                    last_hpposllh_time_ = this->get_clock()->now();

                    auto current_time = this->get_clock()->now();
                    fix_msg_.header.stamp = current_time;

                    // Combine high-precision fields:
                    // lat/lon (1e-7 deg) + latHp/lonHp (1e-9 deg)
                    // height (mm) + heightHp (0.1 mm)
                    fix_msg_.latitude = static_cast<double>(hppos.lat) * 1e-7 + static_cast<double>(hppos.latHp) * 1e-9;
                    fix_msg_.longitude = static_cast<double>(hppos.lon) * 1e-7 + static_cast<double>(hppos.lonHp) * 1e-9;
                    fix_msg_.altitude = (static_cast<double>(hppos.height) + static_cast<double>(hppos.heightHp) * 0.1) / 1000.0;

                    // Retrieve status info from PVT cache if reasonably fresh (< 2s)
                    double pvt_dt = (current_time - last_pvt_time_).seconds();
                    bool is_rtk = false;
                    uint8_t fix_type = 0;
                    if (pvt_dt < 2.0) {
                        is_rtk = (last_pvt_flags_ & 0xC0) != 0;
                        fix_type = last_pvt_fix_type_;
                    }

                    if (fix_type >= 2) {
                        if (is_rtk) {
                            fix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                        } else {
                            fix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                        }
                    } else {
                        // Fallback: if we have HPPOSLLH packets we assume some kind of fix
                        fix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    }
                    fix_msg_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

                    // Convert accuracies (hAcc and vAcc are in 0.1 mm in HPPOSLLH) to variance (meters^2)
                    double h_var = std::pow(hppos.hAcc * 0.0001, 2);
                    double v_var = std::pow(hppos.vAcc * 0.0001, 2);
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

    // PVT cache
    uint8_t last_pvt_fix_type_ = 0;
    uint8_t last_pvt_flags_ = 0;
    uint32_t last_pvt_iTOW_ = 0;
    rclcpp::Time last_pvt_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Fallback/fallback-timing check
    rclcpp::Time last_hpposllh_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GpsDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
