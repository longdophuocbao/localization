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
#include <mutex>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
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
        // Initialize structures explicitly
        std::memset(&cached_pvt_, 0, sizeof(cached_pvt_));

        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<std::string>("frame_id", "gps_link");

        // NTRIP parameters
        this->declare_parameter<std::string>("ntrip_server", "140.110.11.120");
        this->declare_parameter<int>("ntrip_port", 2101);
        this->declare_parameter<std::string>("ntrip_mountpoint", "MOAPRS0");
        this->declare_parameter<std::string>("ntrip_user", "nsysudmee001");
        this->declare_parameter<std::string>("ntrip_password", "GgQc4hu7");
        this->declare_parameter<int>("ntrip_ggainterval", 5);

        this->get_parameter("port", port_);
        this->get_parameter("baud", baud_);
        this->get_parameter("frame_id", frame_id_);

        this->get_parameter("ntrip_server", ntrip_server_);
        this->get_parameter("ntrip_port", ntrip_port_);
        this->get_parameter("ntrip_mountpoint", ntrip_mountpoint_);
        this->get_parameter("ntrip_user", ntrip_user_);
        this->get_parameter("ntrip_password", ntrip_password_);
        this->get_parameter("ntrip_ggainterval", ntrip_ggainterval_);

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

        // Start NTRIP thread
        run_ntrip_thread_ = true;
        ntrip_thread_ = std::thread(&GpsDriverNode::ntrip_thread_loop, this);
    }

    ~GpsDriverNode() {
        run_ntrip_thread_ = false;
        if (ntrip_thread_.joinable()) {
            ntrip_thread_.join();
        }
        std::lock_guard<std::mutex> lock(serial_mutex_);
        serial_conn_.close_port();
        RCLCPP_INFO(this->get_logger(), "GPS driver stopped cleanly.");
    }

private:
    std::string base64_encode(const std::string& in) {
        std::string out;
        int val = 0, valb = -6;
        for (uint8_t c : in) {
            val = (val << 8) + c;
            valb += 8;
            while (valb >= 0) {
                out.push_back("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }
        if (valb > -6) out.push_back("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"[((val << 8) >> (valb + 8)) & 0x3F]);
        while (out.size() % 4) out.push_back('=');
        return out;
    }

    std::string make_gga_sentence(const UbxNavPvt& pvt, double lat_val, double lon_val) {
        // 1. Time: hhmmss.00
        char time_str[32];
        std::snprintf(time_str, sizeof(time_str), "%02d%02d%02d.00", pvt.hour, pvt.min, pvt.sec);

        // 2. Latitude: DDMM.MMMM
        double lat = std::abs(lat_val);
        int lat_deg = static_cast<int>(lat);
        double lat_min = (lat - lat_deg) * 60.0;
        char lat_dir = (lat_val >= 0.0) ? 'N' : 'S';
        char lat_str[32];
        std::snprintf(lat_str, sizeof(lat_str), "%02d%08.5f", lat_deg, lat_min);

        // 3. Longitude: DDDMM.MMMM
        double lon = std::abs(lon_val);
        int lon_deg = static_cast<int>(lon);
        double lon_min = (lon - lon_deg) * 60.0;
        char lon_dir = (lon_val >= 0.0) ? 'E' : 'W';
        char lon_str[32];
        std::snprintf(lon_str, sizeof(lon_str), "%03d%08.5f", lon_deg, lon_min);

        // 4. Fix status: 
        // 0 = invalid, 1 = GPS fix (SPS), 2 = DGPS fix, 4 = Real Time Kinematic (RTK) fixed, 5 = Float RTK
        int gga_fix = 0;
        if (pvt.fixType >= 2) {
            int carrSoln = (pvt.flags >> 6) & 0x03;
            if (carrSoln == 2) {
                gga_fix = 4; // RTK Fixed
            } else if (carrSoln == 1) {
                gga_fix = 5; // RTK Float
            } else {
                gga_fix = 1; // GPS SPS
            }
        }

        // 5. Altitude (hMSL) in meters
        double alt = pvt.hMSL / 1000.0;

        // 6. Format string
        char gga_body[256];
        std::snprintf(gga_body, sizeof(gga_body), "GPGGA,%s,%s,%c,%s,%c,%d,%02d,1.0,%.1f,M,0.0,M,,",
                      time_str, lat_str, lat_dir, lon_str, lon_dir, gga_fix, pvt.numSV, alt);

        // Compute checksum
        uint8_t checksum = 0;
        for (size_t i = 0; gga_body[i] != '\0'; ++i) {
            checksum ^= gga_body[i];
        }

        char checksum_str[16];
        std::snprintf(checksum_str, sizeof(checksum_str), "*%02X", checksum);

        return "$" + std::string(gga_body) + std::string(checksum_str) + "\r\n";
    }

    void ntrip_thread_loop() {
        while (run_ntrip_thread_ && rclcpp::ok()) {
            if (ntrip_server_.empty() || ntrip_port_ <= 0) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            // Wait for initial valid GPS fix (fixType >= 2) before connecting
            bool has_valid_fix = false;
            {
                std::lock_guard<std::mutex> lock(gga_mutex_);
                if (latest_gga_data_.has_data && latest_gga_data_.pvt.fixType >= 2) {
                    has_valid_fix = true;
                }
            }
            if (!has_valid_fix) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Waiting for a valid 2D/3D GPS fix before connecting to NTRIP...");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Connecting to NTRIP caster: %s:%d", ntrip_server_.c_str(), ntrip_port_);
            
            int sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create NTRIP socket.");
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            struct timeval tv;
            tv.tv_sec = 5;
            tv.tv_usec = 0;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

            struct hostent* server = gethostbyname(ntrip_server_.c_str());
            if (server == nullptr) {
                RCLCPP_ERROR(this->get_logger(), "NTRIP server hostname lookup failed: %s", ntrip_server_.c_str());
                close(sock);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            struct sockaddr_in serv_addr;
            std::memset(&serv_addr, 0, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            std::memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
            serv_addr.sin_port = htons(ntrip_port_);

            if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Connection to NTRIP caster failed.");
                close(sock);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Connected to NTRIP caster. Sending HTTP GET request...");

            // Use HTTP/1.1 GET request which successfully authenticated in testing
            std::string req = "GET /" + ntrip_mountpoint_ + " HTTP/1.1\r\n"
                              "Host: " + ntrip_server_ + "\r\n"
                              "Ntrip-Version: Ntrip/2.0\r\n"
                              "User-Agent: NTRIP ROS2 C++ Client\r\n"
                              "Connection: close\r\n";
            if (!ntrip_user_.empty()) {
                std::string credentials = ntrip_user_ + ":" + ntrip_password_;
                req += "Authorization: Basic " + base64_encode(credentials) + "\r\n";
            }
            req += "\r\n";

            if (send(sock, req.c_str(), req.size(), 0) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send HTTP request to NTRIP caster.");
                close(sock);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            // Read HTTP response headers (do NOT send GGA before headers)
            char resp_buf[1024];
            std::string header;
            bool header_done = false;
            while (!header_done && run_ntrip_thread_ && rclcpp::ok()) {
                ssize_t bytes_read = recv(sock, resp_buf, 1, 0);
                if (bytes_read <= 0) {
                    break;
                }
                header.push_back(resp_buf[0]);
                if (header.size() >= 4 && header.substr(header.size() - 4) == "\r\n\r\n") {
                    header_done = true;
                }
            }

            if (!header_done) {
                RCLCPP_ERROR(this->get_logger(), "NTRIP caster closed connection or timed out while reading headers. Received so far:\n%s", header.c_str());
                close(sock);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                continue;
            }

            if (header.find("200 OK") == std::string::npos && header.find("ICY 200") == std::string::npos) {
                RCLCPP_ERROR(this->get_logger(), "NTRIP caster authentication failed or mountpoint not found. Response:\n%s", header.c_str());
                close(sock);
                std::this_thread::sleep_for(std::chrono::seconds(10));
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "NTRIP caster authenticated successfully. Streaming RTCM corrections.");

            // Send initial GGA sentence immediately upon successful header authentication
            std::string initial_gga;
            bool has_gga = false;
            {
                std::lock_guard<std::mutex> lock(gga_mutex_);
                if (latest_gga_data_.has_data) {
                    initial_gga = make_gga_sentence(latest_gga_data_.pvt, latest_gga_data_.latitude, latest_gga_data_.longitude);
                    has_gga = true;
                }
            }
            if (has_gga) {
                RCLCPP_INFO(this->get_logger(), "Sending initial GGA message to stream: %s", initial_gga.c_str());
                send(sock, initial_gga.c_str(), initial_gga.size(), 0);
            }

            auto last_gga_time = std::chrono::steady_clock::now();
            uint8_t rtcm_buf[4096];
            
            while (run_ntrip_thread_ && rclcpp::ok()) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - last_gga_time).count() >= ntrip_ggainterval_) {
                    std::string gga_sent;
                    bool has_gga_now = false;
                    {
                        std::lock_guard<std::mutex> lock(gga_mutex_);
                        if (latest_gga_data_.has_data) {
                            gga_sent = make_gga_sentence(latest_gga_data_.pvt, latest_gga_data_.latitude, latest_gga_data_.longitude);
                            has_gga_now = true;
                        }
                    }

                    if (has_gga_now) {
                        RCLCPP_DEBUG(this->get_logger(), "Sending periodic GGA: %s", gga_sent.c_str());
                        if (send(sock, gga_sent.c_str(), gga_sent.size(), 0) < 0) {
                            RCLCPP_WARN(this->get_logger(), "Failed to send periodic GGA to NTRIP caster.");
                            break;
                        }
                    }
                    last_gga_time = now;
                }

                ssize_t bytes_read = recv(sock, rtcm_buf, sizeof(rtcm_buf), 0);
                if (bytes_read < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        continue;
                    }
                    RCLCPP_WARN(this->get_logger(), "Error reading from NTRIP socket: %s (errno: %d)", std::strerror(errno), errno);
                    break;
                } else if (bytes_read == 0) {
                    RCLCPP_WARN(this->get_logger(), "NTRIP caster closed stream connection.");
                    break;
                }

                // Write RTCM to serial port
                bool is_open = false;
                {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    is_open = serial_conn_.is_open();
                }
                if (is_open) {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    ssize_t written = serial_conn_.write_data(rtcm_buf, bytes_read);
                    if (written < 0) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to write RTCM corrections to GPS serial port.");
                    }
                }
            }

            close(sock);
            RCLCPP_WARN(this->get_logger(), "NTRIP connection lost, retrying in 5 seconds...");
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

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

        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (serial_conn_.is_open()) {
            serial_conn_.write_data(packet.data(), packet.size());
            RCLCPP_INFO(this->get_logger(), "Sent UBX-CFG-MSG to set rate of class 0x%02X ID 0x%02X to %d", target_class, target_id, rate);
        }
    }

    void timer_callback() {
        // Attempt connection if not open
        bool is_connected = false;
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            is_connected = serial_conn_.is_open();
        }
        if (!is_connected) {
            bool opened = false;
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                opened = serial_conn_.open_port(port_, baud_);
            }
            if (opened) {
                RCLCPP_INFO(this->get_logger(), "Successfully connected to GPS at %s", port_.c_str());
                buf_.clear();
            } else {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                      "Failed to connect to GPS at %s, retrying...", port_.c_str());
                return;
            }

            // Call send_ubx_cfg_msg outside the locked region (since it locks internally)
            send_ubx_cfg_msg(0x01, 0x07, 1); // Enable UBX-NAV-PVT
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            send_ubx_cfg_msg(0x01, 0x14, 1); // Enable UBX-NAV-HPPOSLLH
        }

        // Read available data
        uint8_t read_buf[1024];
        ssize_t bytes_read = -1;
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            bytes_read = serial_conn_.read_data(read_buf, sizeof(read_buf));
        }
        if (bytes_read > 0) {
            buf_.insert(buf_.end(), read_buf, read_buf + bytes_read);
        } else if (bytes_read < 0) {
            RCLCPP_ERROR(this->get_logger(), "Read error from GPS, closing port.");
            std::lock_guard<std::mutex> lock(serial_mutex_);
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

                    cached_pvt_ = pvt;
                    last_pvt_fix_type_ = pvt.fixType;
                    last_pvt_flags_ = pvt.flags;
                    last_pvt_iTOW_ = pvt.iTOW;
                    last_pvt_time_ = this->get_clock()->now();

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

                        // Cache GGA data
                        {
                            std::lock_guard<std::mutex> lock(gga_mutex_);
                            latest_gga_data_.pvt = pvt;
                            latest_gga_data_.latitude = fix_msg_.latitude;
                            latest_gga_data_.longitude = fix_msg_.longitude;
                            latest_gga_data_.has_data = true;
                        }
                    }
                }
            } else if (cls == 0x01 && msg_id == 0x14) { // UBX-NAV-HPPOSLLH
                if (payload_len >= 36) {
                    UbxNavHpposllh hppos;
                    std::memcpy(&hppos, &packet[6], sizeof(UbxNavHpposllh));

                    last_hpposllh_time_ = this->get_clock()->now();

                    auto current_time = this->get_clock()->now();
                    fix_msg_.header.stamp = current_time;

                    fix_msg_.latitude = static_cast<double>(hppos.lat) * 1e-7 + static_cast<double>(hppos.latHp) * 1e-9;
                    fix_msg_.longitude = static_cast<double>(hppos.lon) * 1e-7 + static_cast<double>(hppos.lonHp) * 1e-9;
                    fix_msg_.altitude = (static_cast<double>(hppos.height) + static_cast<double>(hppos.heightHp) * 0.1) / 1000.0;

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
                        fix_msg_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    }
                    fix_msg_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

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

                    // Cache GGA data
                    if (pvt_dt < 2.0) {
                        std::lock_guard<std::mutex> lock(gga_mutex_);
                        latest_gga_data_.pvt = cached_pvt_;
                        latest_gga_data_.latitude = fix_msg_.latitude;
                        latest_gga_data_.longitude = fix_msg_.longitude;
                        latest_gga_data_.has_data = true;
                    }
                }
            }
        }
    }

    // Parameters
    std::string port_;
    int baud_;
    std::string frame_id_;

    // NTRIP configuration
    std::string ntrip_server_;
    int ntrip_port_;
    std::string ntrip_mountpoint_;
    std::string ntrip_user_;
    std::string ntrip_password_;
    int ntrip_ggainterval_;

    // NTRIP state
    std::thread ntrip_thread_;
    std::atomic<bool> run_ntrip_thread_;
    std::mutex serial_mutex_;

    // GGA position data cache
    struct GgaPositionData {
        UbxNavPvt pvt;
        double latitude;
        double longitude;
        bool has_data;
        GgaPositionData() {
            std::memset(&pvt, 0, sizeof(pvt));
            latitude = 0.0;
            longitude = 0.0;
            has_data = false;
        }
    };
    GgaPositionData latest_gga_data_;
    std::mutex gga_mutex_;
    UbxNavPvt cached_pvt_;

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
