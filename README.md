# Hệ Thống Định Vị và Hoạch Định Đường Đi Cho Máy Cày Tự Hành (Autonomous Tractor Navigation System)

Dự án này là hệ thống định vị (Localization), điều khiển động cơ (Motor Control) và hoạch định đường đi phủ kín thực địa (Coverage Path Planning - CPP) chạy trên nền tảng **ROS 2**. Hệ thống được thiết kế đặc thù cho các thiết bị nông nghiệp tự hành (như máy cày, máy kéo) hoạt động ngoài đồng ruộng với độ rung chấn cơ học lớn.

---

## 1. Cấu Trúc Thư Mục Hệ Thống

Không gian làm việc (Workspace) bao gồm các gói ROS 2 chính sau:

```text
localization/
├── src/
│   ├── tractor_bringup/                       # Gói chứa các file launch hệ thống
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── launch/
│   │       └── localization_with_bridge.launch.py # Khởi chạy toàn bộ hệ thống & Foxglove Bridge
│   ├── localization_system/                   # Hệ thống định vị
│   │   ├── config/ukf.yaml                    # Cấu hình bộ lọc Kalman mở rộng (UKF)
│   │   ├── src/gps_driver.cpp                 # Driver đọc GPS u-blox qua cổng Serial
│   │   ├── src/imu_driver.cpp                 # Driver đọc IMU WT901 & tích hợp bộ hiệu chuẩn rung động
│   │   └── srv/CalibrateImu.srv               # Định nghĩa service hiệu chuẩn IMU
│   ├── path_planner/                          # Gói sinh đường đi tự động (Fields2Cover)
│   │   ├── config/path_planner_params.yaml    # Tham số mặc định (robot_width, turn_radius, ...)
│   │   ├── src/path_planner_node.cpp          # Node chính xử lý thuật toán CPP & 4 services quản lý đường đi
│   │   └── srv/                               # Các dịch vụ lưu/tải/ghi tọa độ
│   │       ├── RecordVertex.srv
│   │       ├── GeneratePath.srv
│   │       ├── SavePath.srv
│   │       └── LoadPath.srv
│   └── motor_control/                         # Điều khiển động cơ qua CAN bus
│       └── src/can_motor_node.cpp             # Node giao tiếp CAN bus điều khiển góc lái/tốc độ
```

---

## 2. Các Yêu Cầu Phần Cứng & Phần Mềm

### Phần mềm:
*   **Hệ điều hành**: Linux (Ubuntu 22.04 LTS khuyến nghị).
*   **ROS 2 Distro**: ROS 2 Humble / Kilted.
*   **Thư viện hoạch định**: [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) (Thư viện hoạch định đường đi phủ kín cho nông nghiệp).
*   **Gói bổ trợ**: `robot_localization` (UKF), `geographic_msgs`, `foxglove_bridge` (giám sát dữ liệu trực quan).

### Phần cứng:
*   **GPS**: Bộ thu nhận GPS Precision RTK (u-blox) kết nối cổng Serial/USB.
*   **IMU**: Cảm biến gia tốc & góc quay 9-Trục (WT901) giao tiếp Serial.
*   **Cơ cấu chấp hành**: Động cơ điều khiển góc lái/torque kết nối qua CAN Bus (`can0`).

---

## 3. Hướng Dẫn Biên Dịch

Di chuyển vào thư mục gốc của không gian làm việc và thực hiện biên dịch bằng lệnh `colcon`:

```bash
cd ~/localization
colcon build --symlink-install
```

Sau khi biên dịch thành công, source môi trường để ROS 2 nhận diện các gói tin và dịch vụ mới:

```bash
source install/setup.bash
```

---

## 4. Hướng Dẫn Vận Hành

### Bước 1: Khởi chạy hệ thống định vị & cầu nối trực quan hóa dữ liệu (Foxglove)
Lệnh này sẽ khởi chạy driver GPS, driver IMU, bộ lọc định vị UKF (`robot_localization`), tĩnh TF và Foxglove Bridge (để xem bản đồ/đường đi trên PC giám sát):

```bash
ros2 launch tractor_bringup localization_with_bridge.launch.py
```

### Bước 2: Hiệu chuẩn IMU (Loại bỏ nhiễu do động cơ máy cày rung lắc)
Vì máy cày có độ rung chấn cơ học cực kỳ lớn khi nổ máy, bước này là **bắt buộc** trước khi chạy thực tế. Đỗ máy cày tại chỗ cố định (động cơ có thể nổ ở trạng thái ga-lăng-ti) và gọi dịch vụ hiệu chuẩn:

```bash
ros2 service call /calibrate_imu localization_system/srv/CalibrateImu "{duration_seconds: 10.0}"
```
*   **Cơ chế hoạt động**: Thu thập dữ liệu gia tốc và vận tốc góc trong 10 giây. Tính toán ra giá trị sai số tĩnh (bias) của Gyro và độ lệch chuẩn nhiễu (variance). Từ đó, tự động cập nhật các tham số cấu hình hệ thống và tăng mức độ nhiễu trong gói tin `/imu/data`. Bộ lọc UKF sẽ tự động tin cậy GPS nhiều hơn và bỏ qua các dao động rung lắc nhỏ của IMU, tránh hiện tượng trôi vị trí (drift).

### Bước 3: Chạy Node hoạch định đường đi (Path Planner)
Khởi động node hoạch định đường đi với cấu hình tham số xe:

```bash
ros2 run path_planner path_planner_node --ros-args --params-file src/path_planner/config/path_planner_params.yaml
```

---

## 5. Tài Liệu Chi Tiết Các ROS 2 Services

Hệ thống cung cấp các API dịch vụ để quản lý và vận hành thông qua các lệnh gọi trực tiếp hoặc tích hợp vào giao diện điều khiển (HMI):

### 5.1 Ghi lại ranh giới ruộng (`/record_vertex`)
Cho phép ghi lại tọa độ các đỉnh của thửa ruộng để làm ranh giới lập bản đồ.
*   **Thêm đỉnh dựa trên vị trí GPS hiện tại của máy cày**:
    ```bash
    ros2 service call /record_vertex path_planner/srv/RecordVertex "{action: 0, use_current_pose: true}"
    ```
*   **Thêm đỉnh bằng tọa độ x, y chỉ định thủ công**:
    ```bash
    ros2 service call /record_vertex path_planner/srv/RecordVertex "{action: 0, use_current_pose: false, custom_vertex: {x: 15.5, y: 20.0, z: 0.0}}"
    ```
*   **Xóa toàn bộ các đỉnh cũ để đo lại ruộng mới**:
    ```bash
    ros2 service call /record_vertex path_planner/srv/RecordVertex "{action: 1}"
    ```
*   **Xem danh sách các đỉnh hiện tại**:
    ```bash
    ros2 service call /record_vertex path_planner/srv/RecordVertex "{action: 2}"
    ```

### 5.2 Tạo đường đi phủ kín thực địa (`/generate_path`)
Tính toán đường đi song song tối ưu và đường quay đầu (Dubins Curves) dựa trên ranh giới các đỉnh đã ghi lại.
```bash
ros2 service call /generate_path path_planner/srv/GeneratePath "{robot_width: 2.0, robot_turn_radius: 4.0, headland_width: 3.0}"
```
*   *Lưu ý*: Có thể truyền các tham số `= 0.0` để hệ thống tự động sử dụng cấu hình mặc định trong file `path_planner_params.yaml`. Đường đi sinh ra sẽ được xuất bản liên tục lên topic `/plan` (`nav_msgs/msg/Path`).

### 5.3 Lưu lộ trình ra file dạng GPS (`/save_path`)
Chuyển đổi các điểm tọa độ từ hệ tọa độ phẳng `map` sang hệ kinh độ/vĩ độ toàn cầu (WGS84 GPS) và lưu dưới dạng file CSV.
*   **Chỉ lưu các đỉnh ranh giới của ruộng** (Để sau này nạp lại và sinh lại đường đi):
    ```bash
    ros2 service call /save_path path_planner/srv/SavePath "{file_path: '/home/long2/ranh_gioi_ruong.csv', only_vertices: true}"
    ```
*   **Lưu toàn bộ lộ trình chi tiết** (Lưu toàn bộ danh sách điểm waypoints của đường đi để máy cày chạy theo trực tiếp):
    ```bash
    ros2 service call /save_path path_planner/srv/SavePath "{file_path: '/home/long2/lo_trinh_chi_tiet.csv', only_vertices: false}"
    ```

### 5.4 Tải lộ trình từ file GPS (`/load_path`)
Tải tệp tin GPS CSV lên và tự động chuyển đổi ngược từ Kinh độ/Vĩ độ về tọa độ phẳng Cartesian cục bộ thông qua dịch vụ `/from_ll` của bộ lọc định vị.
*   **Tải lên dưới dạng đỉnh ranh giới** (Hệ thống sẽ nạp các điểm làm đỉnh ranh giới ruộng và tự động tính toán lại đường đi):
    ```bash
    ros2 service call /load_path path_planner/srv/LoadPath "{file_path: '/home/long2/ranh_gioi_ruong.csv', as_vertices: true}"
    ```
*   **Tải lên chạy trực tiếp lộ trình** (Hệ thống sẽ nạp trực tiếp đường đi lên bộ nhớ và publish thẳng ra topic `/plan` mà không cần tính toán lại):
    ```bash
    ros2 service call /load_path path_planner/srv/LoadPath "{file_path: '/home/long2/lo_trinh_chi_tiet.csv', as_vertices: false}"
    ```

---

## 6. Cấu Trúc File Lưu Trữ (CSV Format)

Định dạng file lưu trữ tọa độ GPS luôn tuân thủ cấu trúc chuẩn sau:
```csv
latitude,longitude,altitude,yaw
21.02851234,105.85421234,0.00000000,1.23456789
21.02861234,105.85431234,0.00000000,1.23456789
```
*   `latitude`: Vĩ độ (độ thập phân).
*   `longitude`: Kinh độ (độ thập phân).
*   `altitude`: Cao độ (mét).
*   `yaw`: Hướng mũi xe (radian, từ $-\pi$ đến $\pi$).

---

## 7. Các Lưu Ý Kỹ Thuật Quan Trọng

1.  **Deadlock Prevention**: Do các dịch vụ `/save_path` và `/load_path` có gọi lồng dịch vụ chuyển đổi tọa độ của `navsat_transform_node`, các node C++ được chạy bằng bộ thực thi đa luồng `rclcpp::executors::MultiThreadedExecutor`. Không chuyển ngược về SingleThreadedExecutor vì có thể gây treo hệ thống khi gọi dịch vụ.
2.  **Độ chính xác tọa độ**: Tọa độ GPS được lưu với độ chính xác 8 chữ số thập phân (`std::setprecision(8)`), đảm bảo sai số vị trí lưu trữ nhỏ hơn **1.1 milimet**, phù hợp cho việc canh tác nông nghiệp chính xác (Precision Agriculture).
