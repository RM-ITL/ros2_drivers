#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include "serial/serial.h"
#include "thread_safe_queue.hpp"
#include <yaml-cpp/yaml.h>

namespace io
{

// IMU数据结构
struct ImuData
{
  float accx, accy, accz;    // 加速度计数据
  float gyrox, gyroy, gyroz; // 陀螺仪数据
  float roll, pitch, yaw;    // 欧拉角数据
};

class DmImu 
{
public:
  DmImu(const std::string & config_path);
  ~DmImu();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  Eigen::Quaterniond latest_quaternion() const;
  std::array<double, 3> latest_accel() const;
  std::array<double, 3> latest_gyro() const;

  explicit DmImu(const std::string & port, int baud);

private:
  // 初始化函数
  void init_imu_serial();
  
  // IMU配置函数
  void enter_setting_mode();
  void turn_on_accel();
  void turn_on_gyro();
  void turn_off_euler();
  void turn_on_quat();
  void set_output_interval_ms(uint16_t interval_ms);
  void save_imu_para();
  void exit_setting_mode();
  void restart_imu();
  
  // 数据处理函数
  void get_imu_data_thread();
  
private:
  struct QuaterniondData
  {
    Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
    std::chrono::steady_clock::time_point timestamp{};
  };

  // 串口相关
  serial::Serial serial_;
  std::string imu_serial_port_;
  int imu_seial_baud_;
  utils::ThreadSafeQueue<QuaterniondData> queue_;
  std::vector<uint8_t> pending_bytes_;
  
  // 线程相关
  std::thread rec_thread_;
  std::atomic<bool> stop_thread_{false};
  
  // 消息和数据
  // sensor_msgs::msg::Imu imu_msgs_;
  ImuData data_;
  QuaterniondData data_ahead_{}, data_behind_{};
};

}

#endif // IMU_DRIVER_H
