#include "imu_driver.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>
#include <iomanip>           

#include <rclcpp/rclcpp.hpp>  // ROS2 logging
#include <yaml-cpp/yaml.h>    // yaml-cpp

namespace io
{
namespace
{
constexpr std::chrono::milliseconds kCommandDelay{10};
constexpr std::chrono::milliseconds kPostConfigDelay{100};
constexpr std::size_t kQueueCapacity = 5000;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

void sleep_for_command()
{
  std::this_thread::sleep_for(kCommandDelay);
}

void sleep_for_post_config()
{
  std::this_thread::sleep_for(kPostConfigDelay);
}
}  // namespace

// file-scope logger
static const rclcpp::Logger kLogger = rclcpp::get_logger("dm_imu");

DmImu::DmImu(const std::string & config_path)
: imu_seial_baud_(921600),
  queue_(kQueueCapacity)
{
  // use yaml-cpp instead of utils::load/read
  YAML::Node yaml = YAML::LoadFile(config_path);
  imu_serial_port_ = yaml["imu_com_port"].as<std::string>();
  
  data_ = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  pending_bytes_.reserve(256);

  init_imu_serial();

  enter_setting_mode();
  sleep_for_command();

  turn_on_accel();
  sleep_for_command();

  turn_on_gyro();
  sleep_for_command();

  turn_off_euler();
  sleep_for_command();

  turn_on_quat();
  sleep_for_command();

  // 目标约 300Hz，设置 3ms 输出间隔（≈333Hz，为固件支持的最接近取值）
  set_output_interval_ms(3);
  sleep_for_command();

  save_imu_para();
  sleep_for_command();

  exit_setting_mode();
  sleep_for_post_config();

  rec_thread_ = std::thread(&DmImu::get_imu_data_thread, this);

  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);

  RCLCPP_INFO(kLogger, "[DmImu] initialized");
}

DmImu::~DmImu()
{
  stop_thread_ = true;

  if (rec_thread_.joinable()) {
    rec_thread_.join();
  }

  if (serial_.isOpen()) {
    serial_.close();
  }

  RCLCPP_INFO(kLogger, "[DmImu] destroyed");
}

void DmImu::init_imu_serial()
{
  try {
    serial_.setPort(imu_serial_port_);

    uint32_t baud_rate = 921600;
    switch (imu_seial_baud_) {
      case 9600:
      case 19200:
      case 38400:
      case 57600:
      case 115200:
      case 230400:
      case 460800:
      case 921600:
        baud_rate = static_cast<uint32_t>(imu_seial_baud_);
        break;
      default:
        RCLCPP_WARN_STREAM(
          kLogger,
          "[DmImu] unsupported baud rate " << imu_seial_baud_
            << " detected, fallback to 921600");
        imu_seial_baud_ = 921600;
        break;
    }

    serial_.setBaudrate(baud_rate);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    auto timeout = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(timeout);
    serial_.open();

    sleep_for_post_config();
    RCLCPP_INFO_STREAM(kLogger, "[DmImu] serial port " << imu_serial_port_ << " opened");
  } catch (const serial::IOException & e) {
    RCLCPP_ERROR_STREAM(kLogger, "[DmImu] serial IO exception: " << e.what());
    throw;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(kLogger, "[DmImu] serial initialization failed: " << e.what());
    throw;
  }

  if (!serial_.isOpen()) {
    throw std::runtime_error("[DmImu] failed to open imu serial port");
  }
}

void DmImu::enter_setting_mode()
{
  const uint8_t txbuf[4] = {0xAA, 0x06, 0x01, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in enter_setting_mode: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::turn_on_accel()
{
  const uint8_t txbuf[4] = {0xAA, 0x01, 0x14, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in turn_on_accel: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::turn_on_gyro()
{
  const uint8_t txbuf[4] = {0xAA, 0x01, 0x15, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in turn_on_gyro: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::turn_off_euler()
{
  const uint8_t txbuf[4] = {0xAA, 0x01, 0x06, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in turn_off_euler: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::turn_on_quat()
{
  const uint8_t txbuf[4] = {0xAA, 0x01, 0x17, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in turn_on_quat: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::set_output_interval_ms(uint16_t interval_ms)
{
  if (interval_ms == 0) {
    interval_ms = 4;
  }

  const uint8_t delay_low = static_cast<uint8_t>(interval_ms & 0xFF);
  const uint8_t delay_high = static_cast<uint8_t>((interval_ms >> 8) & 0xFF);
  const uint8_t txbuf[5] = {0xAA, 0x02, delay_low, delay_high, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in set_output_interval_ms: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::save_imu_para()
{
  const uint8_t txbuf[4] = {0xAA, 0x03, 0x01, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in save_imu_para: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::exit_setting_mode()
{
  const uint8_t txbuf[4] = {0xAA, 0x06, 0x00, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in exit_setting_mode: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::restart_imu()
{
  const uint8_t txbuf[4] = {0xAA, 0x00, 0x00, 0x0D};
  for (int i = 0; i < 5; ++i) {
    try {
      serial_.write(txbuf, sizeof(txbuf));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] write error in restart_imu: " << e.what());
    }
    sleep_for_command();
  }
}

void DmImu::get_imu_data_thread()
{
  int error_num = 0;
  int packet_count = 0;
  int pushed_count = 0;
  std::array<uint8_t, 128> buffer{};

  RCLCPP_INFO(kLogger, "[DmImu] data thread started");

  while (!stop_thread_) {
    if (!serial_.isOpen()) {
      RCLCPP_WARN(kLogger, "[DmImu] serial port not open when reading");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    try {
      const auto bytes_read = serial_.read(buffer.data(), buffer.size());
      if (bytes_read == 0) {
        continue;
      }

      pending_bytes_.insert(
        pending_bytes_.end(), buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(bytes_read));

      bool parsed_frame = true;
      while (parsed_frame && pending_bytes_.size() >= 4) {
        parsed_frame = false;

        if (pending_bytes_[0] != 0x55 || pending_bytes_[1] != 0xAA || pending_bytes_[2] != 0x01) {
          pending_bytes_.erase(pending_bytes_.begin());
          ++error_num;
          if (error_num > 1200) {
            RCLCPP_WARN(kLogger, "[DmImu] header resync triggered");
            error_num = 0;
          }
          parsed_frame = true;
          continue;
        }

        const uint8_t reg_type = pending_bytes_[3];
        std::size_t frame_size = 0;
        switch (reg_type) {
          case 0x01:
          case 0x02:
          case 0x03:
            frame_size = 19;  // header(4) + payload(12) + crc(2) + tail(1)
            break;
          case 0x04:
            frame_size = 23;  // header(4) + payload(16) + crc(2) + tail(1)
            break;
          default:
            pending_bytes_.erase(pending_bytes_.begin());
            ++error_num;
            if (error_num > 1200) {
              RCLCPP_WARN_STREAM(
                kLogger,
                "[DmImu] unknown register type 0x"
                  << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(reg_type));
              error_num = 0;
            }
            parsed_frame = true;
            continue;
        }

        if (pending_bytes_.size() < frame_size) {
          break;
        }

        if (pending_bytes_[frame_size - 1] != 0x0A) {
          pending_bytes_.erase(pending_bytes_.begin());
          ++error_num;
          if (error_num > 1200) {
            RCLCPP_WARN_STREAM(
              kLogger,
              "[DmImu] invalid frame tail for reg 0x"
                << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(reg_type));
            error_num = 0;
          }
          parsed_frame = true;
          continue;
        }

        const uint8_t * frame = pending_bytes_.data();

        ++packet_count;
        error_num = 0;

        if (reg_type == 0x01) {
          std::memcpy(&data_.accx, frame + 4, 4);
          std::memcpy(&data_.accy, frame + 8, 4);
          std::memcpy(&data_.accz, frame + 12, 4);
        } else if (reg_type == 0x02) {
          std::memcpy(&data_.gyrox, frame + 4, 4);
          std::memcpy(&data_.gyroy, frame + 8, 4);
          std::memcpy(&data_.gyroz, frame + 12, 4);
        } else if (reg_type == 0x03) {
          std::memcpy(&data_.roll, frame + 4, 4);
          std::memcpy(&data_.pitch, frame + 8, 4);
          std::memcpy(&data_.yaw, frame + 12, 4);

          const auto timestamp = std::chrono::steady_clock::now();
          Eigen::Quaterniond q =
            Eigen::AngleAxisd(data_.yaw * kDegToRad, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(data_.pitch * kDegToRad, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(data_.roll * kDegToRad, Eigen::Vector3d::UnitX());
          q.normalize();
          queue_.push({q, timestamp});
          ++pushed_count;
        } else if (reg_type == 0x04) {
          // IMU 固件输出的四元数顺序为 W,X,Y,Z（float32 小端）
          float qw = 1.0F;
          float qx = 0.0F;
          float qy = 0.0F;
          float qz = 0.0F;
          std::memcpy(&qw, frame + 4, 4);
          std::memcpy(&qx, frame + 8, 4);
          std::memcpy(&qy, frame + 12, 4);
          std::memcpy(&qz, frame + 16, 4);

          const auto timestamp = std::chrono::steady_clock::now();
          Eigen::Quaterniond q(static_cast<double>(qw), static_cast<double>(qx),
            static_cast<double>(qy), static_cast<double>(qz));
          q.normalize();
          queue_.push({q, timestamp});
          ++pushed_count;
        }

        if (packet_count % 1000 == 0) {
          // RCLCPP_INFO_STREAM(kLogger,
          //   "[DmImu] processed " << packet_count << " packets, pushed " << pushed_count << " quaternions");
        }

        pending_bytes_.erase(pending_bytes_.begin(), pending_bytes_.begin() + static_cast<std::ptrdiff_t>(frame_size));
        parsed_frame = true;
      }
    } catch (const serial::SerialException & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] serial read exception: " << e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(kLogger, "[DmImu] unexpected read error: " << e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  RCLCPP_INFO(kLogger, "[DmImu] data thread stopped");
}

Eigen::Quaterniond DmImu::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) {
    data_ahead_ = data_behind_;
  }

  while (data_behind_.timestamp <= timestamp) {
    QuaterniondData next_sample;
    queue_.pop(next_sample);
    data_ahead_ = data_behind_;
    data_behind_ = next_sample;
  }

  const auto t_ab = data_behind_.timestamp - data_ahead_.timestamp;
  if (t_ab.count() == 0) {
    return data_behind_.q.normalized();
  }

  const auto t_ac = timestamp - data_ahead_.timestamp;
  const double denom = std::chrono::duration<double>(t_ab).count();
  if (std::abs(denom) <= std::numeric_limits<double>::epsilon()) {
    return data_behind_.q.normalized();
  }

  const double ratio = std::clamp(
    std::chrono::duration<double>(t_ac).count() / denom, 0.0, 1.0);

  const Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  const Eigen::Quaterniond q_b = data_behind_.q.normalized();
  return q_a.slerp(ratio, q_b).normalized();
}

Eigen::Quaterniond DmImu::latest_quaternion() const
{
  return data_behind_.q.normalized();
}

std::array<double, 3> DmImu::latest_accel() const
{
  return {static_cast<double>(data_.accx),
          static_cast<double>(data_.accy),
          static_cast<double>(data_.accz)};
}

std::array<double, 3> DmImu::latest_gyro() const
{
  return {static_cast<double>(data_.gyrox),
          static_cast<double>(data_.gyroy),
          static_cast<double>(data_.gyroz)};
}

// 新增：端口+波特率构造
DmImu::DmImu(const std::string & port, int baud)
: imu_seial_baud_(baud),
  queue_(kQueueCapacity)
{
  // 直接用传入的串口与波特率，不读 YAML
  imu_serial_port_ = port;

  data_ = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  pending_bytes_.reserve(256);

  init_imu_serial();

  enter_setting_mode();
  sleep_for_command();

  turn_on_accel();
  sleep_for_command();

  turn_on_gyro();
  sleep_for_command();

  turn_off_euler();
  sleep_for_command();

  turn_on_quat();
  sleep_for_command();

  // 目标约 300Hz，设置 3ms 输出间隔（≈333Hz）
  set_output_interval_ms(3);
  sleep_for_command();

  save_imu_para();
  sleep_for_command();

  exit_setting_mode();
  sleep_for_post_config();

  rec_thread_ = std::thread(&DmImu::get_imu_data_thread, this);

  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);

  RCLCPP_INFO(kLogger, "[DmImu] initialized (port=%s baud=%d)",
              imu_serial_port_.c_str(), imu_seial_baud_);
}
}  // namespace io

