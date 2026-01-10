#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "imu_driver.h"

class ImuDriverNode : public rclcpp::Node
{
public:
  ImuDriverNode()
  : Node("imu_driver_node")
  {
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/dm_imu/data_raw", rclcpp::SensorDataQoS());

    // 从 ROS2 参数读取配置
    std::string port = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    int baud = this->declare_parameter<int>("baud", 921600);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
    double publish_rate_hz = this->declare_parameter<double>("publish_rate", 300.0);
    if (publish_rate_hz <= 0.0) publish_rate_hz = 300.0;

    // 初始化 IMU 驱动
    imu_driver_ = std::make_unique<io::DmImu>(port, baud);

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = this->create_wall_timer(period, std::bind(&ImuDriverNode::publish_imu_data, this));

    RCLCPP_INFO(this->get_logger(), "IMU driver node started. port=%s baud=%d rate=%.1f Hz frame=%s",
                port.c_str(), baud, publish_rate_hz, frame_id_.c_str());
  }

private:
  void publish_imu_data()
  {
    auto q    = imu_driver_->latest_quaternion();
    auto acc  = imu_driver_->latest_accel();
    auto gyro = imu_driver_->latest_gyro();

    // 发布原始 IMU 数据
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = frame_id_;

    imu_msg.orientation.w = q.w();
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();

    imu_msg.linear_acceleration.x = acc[0];
    imu_msg.linear_acceleration.y = acc[1];
    imu_msg.linear_acceleration.z = acc[2];

    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];

    imu_msg.orientation_covariance[0]        = -1.0;
    imu_msg.angular_velocity_covariance[0]   = -1.0;
    imu_msg.linear_acceleration_covariance[0]= -1.0;

    imu_pub_->publish(imu_msg);
  }

  std::unique_ptr<io::DmImu> imu_driver_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string frame_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImuDriverNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "[imu_driver_node] initialization failed: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
