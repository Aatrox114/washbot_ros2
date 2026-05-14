#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

static constexpr uint8_t FRAME_HEAD_1 = 0xAA;
static constexpr uint8_t FRAME_HEAD_2 = 0x55;

static constexpr uint8_t TYPE_CMD_VEL = 0x01;
static constexpr uint8_t TYPE_ODOM    = 0x02;

static constexpr uint8_t LEN_CMD_VEL = 7;
static constexpr uint8_t LEN_ODOM    = 20;

static uint16_t crc16_modbus(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

static int16_t clamp_to_i16(double value)
{
  if (value > 32767.0) return 32767;
  if (value < -32768.0) return -32768;
  return static_cast<int16_t>(std::lround(value));
}

static void push_u8(std::vector<uint8_t> &buf, uint8_t value)
{
  buf.push_back(value);
}

static void push_i16_le(std::vector<uint8_t> &buf, int16_t value)
{
  uint16_t v = static_cast<uint16_t>(value);
  buf.push_back(static_cast<uint8_t>(v & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

static int16_t read_i16_le(const std::vector<uint8_t> &buf, size_t idx)
{
  uint16_t v = static_cast<uint16_t>(buf[idx]) |
               (static_cast<uint16_t>(buf[idx + 1]) << 8);
  return static_cast<int16_t>(v);
}

static int32_t read_i32_le(const std::vector<uint8_t> &buf, size_t idx)
{
  uint32_t v = static_cast<uint32_t>(buf[idx]) |
               (static_cast<uint32_t>(buf[idx + 1]) << 8) |
               (static_cast<uint32_t>(buf[idx + 2]) << 16) |
               (static_cast<uint32_t>(buf[idx + 3]) << 24);
  return static_cast<int32_t>(v);
}

static speed_t baudrate_to_flag(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
    default:
      return B115200;
  }
}

class SerialPort
{
public:
  SerialPort() = default;

  ~SerialPort()
  {
    close_port();
  }

  bool open_port(const std::string &port, int baudrate)
  {
    close_port();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      return false;
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
      close_port();
      return false;
    }

    cfmakeraw(&tty);

    speed_t speed = baudrate_to_flag(baudrate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIOFLUSH);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      close_port();
      return false;
    }

    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const
  {
    return fd_ >= 0;
  }

  bool write_all(const std::vector<uint8_t> &data)
  {
    if (fd_ < 0) return false;

    size_t sent = 0;
    while (sent < data.size()) {
      ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
      if (n > 0) {
        sent += static_cast<size_t>(n);
      } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          std::this_thread::sleep_for(1ms);
          continue;
        }
        return false;
      }
    }
    return true;
  }

  bool read_byte(uint8_t &byte)
  {
    if (fd_ < 0) return false;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(fd_, &readfds)) {
      ssize_t n = ::read(fd_, &byte, 1);
      return n == 1;
    }

    return false;
  }

private:
  int fd_ = -1;
};

class BaseDriverNode : public rclcpp::Node
{
public:
  BaseDriverNode()
  : Node("base_driver_node")
  {
    port_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);

    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    status_topic_ = this->declare_parameter<std::string>("status_topic", "/base_status");

    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

    cmd_timeout_s_ = this->declare_parameter<double>("cmd_timeout_s", 0.5);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 50.0);

    max_vx_ = this->declare_parameter<double>("max_vx", 1.0);
    max_vy_ = this->declare_parameter<double>("max_vy", 1.0);
    max_wz_ = this->declare_parameter<double>("max_wz", 2.0);

    if (send_rate_hz_ <= 0.0) {
      throw std::runtime_error("send_rate_hz must be positive");
    }

    if (!serial_.open_port(port_, baudrate_)) {
      throw std::runtime_error("Failed to open serial port: " + port_);
    }

    RCLCPP_INFO(this->get_logger(), "Serial opened: %s, baudrate: %d", port_.c_str(), baudrate_);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    status_pub_ = this->create_publisher<std_msgs::msg::UInt8>(status_topic_, 20);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_,
      20,
      std::bind(&BaseDriverNode::cmd_callback, this, std::placeholders::_1)
    );

    last_cmd_time_ = this->now();

    auto period_ms = static_cast<int>(1000.0 / send_rate_hz_);
    if (period_ms < 1) period_ms = 1;

    send_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&BaseDriverNode::send_timer_callback, this)
    );

    running_.store(true);
    read_thread_ = std::thread(&BaseDriverNode::read_loop, this);

    RCLCPP_INFO(this->get_logger(), "Base driver node started.");
  }

  ~BaseDriverNode() override
  {
    running_.store(false);

    if (read_thread_.joinable()) {
      read_thread_.join();
    }

    send_cmd_vel_frame(0.0, 0.0, 0.0);
    serial_.close_port();

    RCLCPP_INFO(this->get_logger(), "Base driver node stopped.");
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    target_vx_ = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    target_vy_ = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    target_wz_ = std::clamp(msg->angular.z, -max_wz_, max_wz_);

    last_cmd_time_ = this->now();
  }

  void send_timer_callback()
  {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);

      double dt = (this->now() - last_cmd_time_).seconds();
      if (dt <= cmd_timeout_s_) {
        vx = target_vx_;
        vy = target_vy_;
        wz = target_wz_;
      } else {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
      }
    }

    send_cmd_vel_frame(vx, vy, wz);
  }

  void send_cmd_vel_frame(double vx, double vy, double wz)
  {
    int16_t vx_mm_s = clamp_to_i16(vx * 1000.0);
    int16_t vy_mm_s = clamp_to_i16(vy * 1000.0);
    int16_t wz_mrad_s = clamp_to_i16(wz * 1000.0);

    std::vector<uint8_t> frame;
    frame.reserve(2 + 1 + 1 + LEN_CMD_VEL + 2);

    push_u8(frame, FRAME_HEAD_1);
    push_u8(frame, FRAME_HEAD_2);
    push_u8(frame, TYPE_CMD_VEL);
    push_u8(frame, LEN_CMD_VEL);

    push_u8(frame, tx_seq_++);
    push_i16_le(frame, vx_mm_s);
    push_i16_le(frame, vy_mm_s);
    push_i16_le(frame, wz_mrad_s);

    uint16_t crc = crc16_modbus(frame.data(), frame.size());
    push_u8(frame, static_cast<uint8_t>(crc & 0xFF));
    push_u8(frame, static_cast<uint8_t>((crc >> 8) & 0xFF));

    if (!serial_.write_all(frame)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to write serial frame");
    }
  }

  void read_loop()
  {
    while (running_.load()) {
      uint8_t byte = 0;
      if (serial_.read_byte(byte)) {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        rx_buffer_.push_back(byte);
        parse_rx_buffer();
      } else {
        std::this_thread::sleep_for(1ms);
      }
    }
  }

  void parse_rx_buffer()
  {
    while (rx_buffer_.size() >= 6) {
      if (rx_buffer_[0] != FRAME_HEAD_1 || rx_buffer_[1] != FRAME_HEAD_2) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      uint8_t type = rx_buffer_[2];
      uint8_t len = rx_buffer_[3];

      size_t total_len = 2 + 1 + 1 + static_cast<size_t>(len) + 2;
      if (rx_buffer_.size() < total_len) {
        return;
      }

      uint16_t recv_crc = static_cast<uint16_t>(rx_buffer_[total_len - 2]) |
                          (static_cast<uint16_t>(rx_buffer_[total_len - 1]) << 8);

      uint16_t calc_crc = crc16_modbus(rx_buffer_.data(), total_len - 2);

      if (recv_crc != calc_crc) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "CRC check failed");
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      std::vector<uint8_t> payload(
        rx_buffer_.begin() + 4,
        rx_buffer_.begin() + 4 + len
      );

      handle_frame(type, payload);

      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + total_len);
    }
  }

  void handle_frame(uint8_t type, const std::vector<uint8_t> &payload)
  {
    if (type == TYPE_ODOM) {
      handle_odom_frame(payload);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Unknown frame type: 0x%02X",
        type
      );
    }
  }

  void handle_odom_frame(const std::vector<uint8_t> &payload)
  {
    if (payload.size() != LEN_ODOM) {
      RCLCPP_WARN(this->get_logger(), "Invalid odom payload length: %zu", payload.size());
      return;
    }

    uint8_t seq = payload[0];
    (void)seq;

    int32_t x_mm = read_i32_le(payload, 1);
    int32_t y_mm = read_i32_le(payload, 5);
    int32_t yaw_mrad = read_i32_le(payload, 9);

    int16_t vx_mm_s = read_i16_le(payload, 13);
    int16_t vy_mm_s = read_i16_le(payload, 15);
    int16_t wz_mrad_s = read_i16_le(payload, 17);

    uint8_t status = payload[19];

    double x = static_cast<double>(x_mm) / 1000.0;
    double y = static_cast<double>(y_mm) / 1000.0;
    double yaw = static_cast<double>(yaw_mrad) / 1000.0;

    double vx = static_cast<double>(vx_mm_s) / 1000.0;
    double vy = static_cast<double>(vy_mm_s) / 1000.0;
    double wz = static_cast<double>(wz_mrad_s) / 1000.0;

    auto stamp = this->now();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;

    odom.pose.covariance[0] = 0.05;
    odom.pose.covariance[7] = 0.05;
    odom.pose.covariance[35] = 0.10;

    odom.twist.covariance[0] = 0.05;
    odom.twist.covariance[7] = 0.05;
    odom.twist.covariance[35] = 0.10;

    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id = base_frame_;

      tf_msg.transform.translation.x = x;
      tf_msg.transform.translation.y = y;
      tf_msg.transform.translation.z = 0.0;

      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(tf_msg);
    }

    std_msgs::msg::UInt8 status_msg;
    status_msg.data = status;
    status_pub_->publish(status_msg);
  }

private:
  std::string port_;
  int baudrate_;

  std::string cmd_topic_;
  std::string odom_topic_;
  std::string status_topic_;

  std::string odom_frame_;
  std::string base_frame_;

  bool publish_tf_;

  double cmd_timeout_s_;
  double send_rate_hz_;

  double max_vx_;
  double max_vy_;
  double max_wz_;

  SerialPort serial_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr send_timer_;

  std::mutex cmd_mutex_;
  double target_vx_ = 0.0;
  double target_vy_ = 0.0;
  double target_wz_ = 0.0;
  rclcpp::Time last_cmd_time_;

  uint8_t tx_seq_ = 0;

  std::atomic<bool> running_{false};
  std::thread read_thread_;

  std::mutex rx_mutex_;
  std::vector<uint8_t> rx_buffer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<BaseDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << "base_driver_node error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
