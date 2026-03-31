#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sl_lidar.h"
#include "std_srvs/srv/empty.hpp"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x) * M_PI / 180.0)

namespace
{

float get_angle(const sl_lidar_response_measurement_node_hq_t & node)
{
  return node.angle_z_q14 * 90.f / 16384.f;
}

}  // namespace

class SllidarNode : public rclcpp::Node
{
public:
  SllidarNode()
  : Node("sllidar_node")
  {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    stop_motor_service_ = create_service<std_srvs::srv::Empty>(
      "stop_motor",
      std::bind(
        &SllidarNode::handle_stop_motor, this, std::placeholders::_1,
        std::placeholders::_2));
    start_motor_service_ = create_service<std_srvs::srv::Empty>(
      "start_motor",
      std::bind(
        &SllidarNode::handle_start_motor, this, std::placeholders::_1,
        std::placeholders::_2));

    declare_and_load_parameters();
  }

  ~SllidarNode() override
  {
    shutdown_driver();
  }

  bool initialize()
  {
    using sl::IChannel;
    using sl::ILidarDriver;
    using sl::LidarScanMode;
    using sl::createLidarDriver;
    using sl::createSerialPortChannel;
    using sl::createTcpChannel;
    using sl::createUdpChannel;
    RCLCPP_INFO(
      get_logger(), "RPLIDAR ROS2 driver starting, SDK version: %d.%d.%d",
      SL_LIDAR_SDK_VERSION_MAJOR, SL_LIDAR_SDK_VERSION_MINOR, SL_LIDAR_SDK_VERSION_PATCH);

    driver_ = *createLidarDriver();
    IChannel * channel = nullptr;

    if (channel_type_ == "tcp") {
      channel = *createTcpChannel(tcp_ip_, tcp_port_);
    } else if (channel_type_ == "udp") {
      channel = *createUdpChannel(udp_ip_, udp_port_);
    } else {
      channel = *createSerialPortChannel(serial_port_, serial_baudrate_);
    }

    if (SL_IS_FAIL(driver_->connect(channel))) {
      if (channel_type_ == "tcp") {
        RCLCPP_ERROR(
          get_logger(), "Cannot connect to lidar at %s:%d over TCP.",
          tcp_ip_.c_str(), tcp_port_);
      } else if (channel_type_ == "udp") {
        RCLCPP_ERROR(
          get_logger(), "Cannot connect to lidar at %s:%d over UDP.",
          udp_ip_.c_str(), udp_port_);
      } else {
        RCLCPP_ERROR(
          get_logger(), "Cannot bind to lidar serial port %s.",
          serial_port_.c_str());
      }
      delete driver_;
      driver_ = nullptr;
      return false;
    }

    if (!get_device_info() || !check_health()) {
      shutdown_driver();
      return false;
    }

    driver_->setMotorSpeed();

    sl_result op_result;
    if (scan_mode_.empty()) {
      op_result = driver_->startScan(false, true, 0, &current_scan_mode_);
    } else {
      std::vector<LidarScanMode> all_supported_scan_modes;
      op_result = driver_->getAllSupportedScanModes(all_supported_scan_modes);
      if (SL_IS_OK(op_result)) {
        sl_u16 selected_scan_mode = sl_u16(-1);
        for (auto & mode : all_supported_scan_modes) {
          if (mode.scan_mode == scan_mode_) {
            selected_scan_mode = mode.id;
            break;
          }
        }

        if (selected_scan_mode == sl_u16(-1)) {
          RCLCPP_ERROR(
            get_logger(), "scan mode '%s' is not supported by this lidar.",
            scan_mode_.c_str());
          for (auto & mode : all_supported_scan_modes) {
            RCLCPP_ERROR(
              get_logger(), "  %s: max_distance=%.1f m, sample_rate=%.1f KHz",
              mode.scan_mode, mode.max_distance, 1000.0 / mode.us_per_sample);
          }
          op_result = SL_RESULT_OPERATION_FAIL;
        } else {
          op_result = driver_->startScanExpress(
            false, selected_scan_mode, 0, &current_scan_mode_);
        }
      }
    }

    if (!SL_IS_OK(op_result)) {
      RCLCPP_ERROR(get_logger(), "Cannot start scan: %08x", op_result);
      shutdown_driver();
      return false;
    }

    points_per_circle_ =
      static_cast<int>(1000 * 1000 / current_scan_mode_.us_per_sample / scan_frequency_);
    angle_compensate_multiple_ = points_per_circle_ / 360.0f + 1.0f;
    if (angle_compensate_multiple_ < 1.0f) {
      angle_compensate_multiple_ = 1.0f;
    }
    max_distance_ = static_cast<float>(current_scan_mode_.max_distance);

    RCLCPP_INFO(
      get_logger(),
      "current scan mode: %s, sample rate: %d KHz, max_distance: %.1f m, scan frequency: %.1f Hz",
      current_scan_mode_.scan_mode,
      static_cast<int>(1000 / current_scan_mode_.us_per_sample + 0.5),
      max_distance_, scan_frequency_);

    return true;
  }

  void poll_once()
  {
    if (driver_ == nullptr) {
      return;
    }

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    const auto start_scan_time = now();
    const auto op_result = driver_->grabScanDataHq(nodes, count);
    const auto end_scan_time = now();
    const double scan_duration = (end_scan_time - start_scan_time).seconds();

    if (op_result != SL_RESULT_OK) {
      return;
    }

    const auto ascend_result = driver_->ascendScanData(nodes, count);
    float angle_min = DEG2RAD(0.0f);
    float angle_max = DEG2RAD(360.0f);

    if (ascend_result == SL_RESULT_OK) {
      if (angle_compensate_) {
        const int angle_compensate_nodes_count =
          static_cast<int>(360 * angle_compensate_multiple_);
        int angle_compensate_offset = 0;
        std::vector<sl_lidar_response_measurement_node_hq_t> compensated_nodes(
          angle_compensate_nodes_count);
        std::memset(
          compensated_nodes.data(), 0,
          compensated_nodes.size() * sizeof(sl_lidar_response_measurement_node_hq_t));

        for (size_t i = 0; i < count; ++i) {
          if (nodes[i].dist_mm_q2 == 0) {
            continue;
          }
          const float angle = get_angle(nodes[i]);
          const int angle_value = static_cast<int>(angle * angle_compensate_multiple_);
          if ((angle_value - angle_compensate_offset) < 0) {
            angle_compensate_offset = angle_value;
          }

          for (int j = 0; j < static_cast<int>(angle_compensate_multiple_); ++j) {
            int index = angle_value - angle_compensate_offset + j;
            if (index >= angle_compensate_nodes_count) {
              index = angle_compensate_nodes_count - 1;
            }
            compensated_nodes[index] = nodes[i];
          }
        }

        publish_scan(
          compensated_nodes.data(), compensated_nodes.size(), start_scan_time,
          scan_duration, angle_min, angle_max);
      } else {
        size_t start_node = 0;
        size_t end_node = count - 1;

        while (start_node < count && nodes[start_node].dist_mm_q2 == 0) {
          ++start_node;
        }
        while (end_node > start_node && nodes[end_node].dist_mm_q2 == 0) {
          --end_node;
        }

        if (start_node >= count || end_node <= start_node) {
          return;
        }

        angle_min = DEG2RAD(get_angle(nodes[start_node]));
        angle_max = DEG2RAD(get_angle(nodes[end_node]));
        publish_scan(
          &nodes[start_node], end_node - start_node + 1, start_scan_time,
          scan_duration, angle_min, angle_max);
      }
    } else if (ascend_result == SL_RESULT_OPERATION_FAIL) {
      angle_max = DEG2RAD(359.0f);
      publish_scan(nodes, count, start_scan_time, scan_duration, angle_min, angle_max);
    }
  }

private:
  void declare_and_load_parameters()
  {
    channel_type_ = declare_parameter<std::string>("channel_type", "serial");
    tcp_ip_ = declare_parameter<std::string>("tcp_ip", "192.168.0.7");
    tcp_port_ = declare_parameter<int>("tcp_port", 20108);
    udp_ip_ = declare_parameter<std::string>("udp_ip", "192.168.11.2");
    udp_port_ = declare_parameter<int>("udp_port", 8089);
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    serial_baudrate_ = declare_parameter<int>("serial_baudrate", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_frame");
    inverted_ = declare_parameter<bool>("inverted", false);
    angle_compensate_ = declare_parameter<bool>("angle_compensate", false);
    scan_mode_ = declare_parameter<std::string>("scan_mode", "");
    if (channel_type_ == "udp") {
      scan_frequency_ = declare_parameter<double>("scan_frequency", 20.0);
    } else {
      scan_frequency_ = declare_parameter<double>("scan_frequency", 10.0);
    }
  }

  bool get_device_info()
  {
    sl_lidar_response_device_info_t devinfo;
    const auto op_result = driver_->getDeviceInfo(devinfo);
    if (SL_IS_FAIL(op_result)) {
      if (op_result == SL_RESULT_OPERATION_TIMEOUT) {
        RCLCPP_ERROR(get_logger(), "Lidar operation timed out while reading device info.");
      } else {
        RCLCPP_ERROR(get_logger(), "Unexpected lidar device info error: %x", op_result);
      }
      return false;
    }

    char serial_number[35] = {0};
    for (int pos = 0; pos < 16; ++pos) {
      std::sprintf(serial_number + (pos * 2), "%02X", devinfo.serialnum[pos]);
    }

    RCLCPP_INFO(get_logger(), "RPLIDAR S/N: %s", serial_number);
    RCLCPP_INFO(
      get_logger(), "Firmware Ver: %d.%02d",
      devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
    RCLCPP_INFO(get_logger(), "Hardware Rev: %d", static_cast<int>(devinfo.hardware_version));
    return true;
  }

  bool check_health()
  {
    sl_lidar_response_device_health_t healthinfo;
    const auto op_result = driver_->getHealth(healthinfo);
    if (!SL_IS_OK(op_result)) {
      RCLCPP_ERROR(get_logger(), "Cannot retrieve lidar health code: %x", op_result);
      return false;
    }

    switch (healthinfo.status) {
      case SL_LIDAR_STATUS_OK:
        RCLCPP_INFO(get_logger(), "RPLidar health status: OK.");
        return true;
      case SL_LIDAR_STATUS_WARNING:
        RCLCPP_WARN(get_logger(), "RPLidar health status: Warning.");
        return true;
      case SL_LIDAR_STATUS_ERROR:
        RCLCPP_ERROR(
          get_logger(), "RPLidar internal error detected. Please reboot the device.");
        return false;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown lidar health status: %d", healthinfo.status);
        return false;
    }
  }

  void publish_scan(
    const sl_lidar_response_measurement_node_hq_t * nodes, size_t node_count,
    const rclcpp::Time & start, double scan_time, float angle_min, float angle_max)
  {
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id_;

    const bool reversed = angle_max > angle_min;
    if (reversed) {
      scan_msg.angle_min = static_cast<float>(M_PI - angle_max);
      scan_msg.angle_max = static_cast<float>(M_PI - angle_min);
    } else {
      scan_msg.angle_min = static_cast<float>(M_PI - angle_min);
      scan_msg.angle_max = static_cast<float>(M_PI - angle_max);
    }

    const double divisor = std::max<size_t>(node_count - 1, 1);
    scan_msg.angle_increment =
      static_cast<float>((scan_msg.angle_max - scan_msg.angle_min) / divisor);
    scan_msg.scan_time = static_cast<float>(scan_time);
    scan_msg.time_increment = static_cast<float>(scan_time / divisor);
    scan_msg.range_min = 0.15f;
    scan_msg.range_max = max_distance_;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);

    const bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
    for (size_t i = 0; i < node_count; ++i) {
      const float read_value = static_cast<float>(nodes[i].dist_mm_q2 / 4.0f / 1000.0f);
      const size_t index = reverse_data ? (node_count - 1 - i) : i;
      scan_msg.ranges[index] =
        read_value == 0.0f ? std::numeric_limits<float>::infinity() : read_value;
      scan_msg.intensities[index] = static_cast<float>(nodes[i].quality >> 2);
    }

    scan_pub_->publish(scan_msg);
  }

  void handle_stop_motor(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    if (driver_ == nullptr) {
      return;
    }
    RCLCPP_DEBUG(get_logger(), "Stop motor");
    driver_->setMotorSpeed(0);
  }

  void handle_start_motor(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
  {
    if (driver_ == nullptr) {
      return;
    }
    if (driver_->isConnected()) {
      RCLCPP_DEBUG(get_logger(), "Start motor");
      driver_->setMotorSpeed();
      driver_->startScan(0, 1);
    } else {
      RCLCPP_WARN(get_logger(), "Lost lidar connection");
    }
  }

  void shutdown_driver()
  {
    if (driver_ == nullptr) {
      return;
    }
    driver_->setMotorSpeed(0);
    driver_->stop();
    delete driver_;
    driver_ = nullptr;
  }

  sl::ILidarDriver * driver_ = nullptr;
  sl::LidarScanMode current_scan_mode_{};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_service_;

  std::string channel_type_;
  std::string tcp_ip_;
  int tcp_port_ = 20108;
  std::string udp_ip_;
  int udp_port_ = 8089;
  std::string serial_port_;
  int serial_baudrate_ = 115200;
  std::string frame_id_;
  bool inverted_ = false;
  bool angle_compensate_ = false;
  std::string scan_mode_;
  double scan_frequency_ = 10.0;
  float angle_compensate_multiple_ = 1.0f;
  int points_per_circle_ = 360;
  float max_distance_ = 16.0f;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SllidarNode>();
  if (!node->initialize()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok()) {
    node->poll_once();
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}
