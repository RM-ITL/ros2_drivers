#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
extern "C" {
#include "MvCameraControl.h"
}

class MvsCameraNode : public rclcpp::Node {
public:
  MvsCameraNode() : Node("mvs_camera") {
    serial_ = this->declare_parameter<std::string>("serial", "");
    width_  = this->declare_parameter<int>("width", 1280);
    height_ = this->declare_parameter<int>("height", 1024);
    fps_    = this->declare_parameter<double>("fps", 60.0);
    exposure_us_ = this->declare_parameter<double>("exposure_us", 8000.0);
    gain_   = this->declare_parameter<double>("gain", 6.0);
    auto_exposure_ = this->declare_parameter<bool>("auto_exposure", false);
    auto_wb_       = this->declare_parameter<bool>("auto_white_balance", true);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera");

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "camera_image", rclcpp::SensorDataQoS());

    if (!openDevice()) {
      RCLCPP_FATAL(this->get_logger(), "Open camera failed");
      rclcpp::shutdown();
      return;
    }
    grab_thread_ = std::thread([this]{ this->grabLoop(); });
  }

  ~MvsCameraNode() override {
    running_ = false;
    if (grab_thread_.joinable()) grab_thread_.join();
    if (handle_) {
      MV_CC_StopGrabbing(handle_);
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      handle_ = nullptr;
    }
  }

private:
  bool openDevice() {
    MV_CC_DEVICE_INFO_LIST dev_list{};
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
    if (ret != MV_OK || dev_list.nDeviceNum == 0) {
      RCLCPP_ERROR(this->get_logger(), "No camera found");
      return false;
    }
    unsigned index = 0;
    if (!serial_.empty()) {
      for (unsigned i=0; i<dev_list.nDeviceNum; ++i) {
        auto *info = dev_list.pDeviceInfo[i];
        // USB3 序列号字段
        std::string sn;
        if (info->nTLayerType == MV_USB_DEVICE) {
          sn = reinterpret_cast<char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
        } else if (info->nTLayerType == MV_GIGE_DEVICE) {
          sn = reinterpret_cast<char*>(info->SpecialInfo.stGigEInfo.chSerialNumber);
        }
        if (sn == serial_) { index = i; break; }
      }
    }
    if (MV_OK != MV_CC_CreateHandle(&handle_, dev_list.pDeviceInfo[index])) return false;
    if (MV_OK != MV_CC_OpenDevice(handle_)) return false;

    // 连续采集
    MV_CC_SetEnumValue(handle_, "TriggerMode", 0);

    // 分辨率（若相机不支持会返回错误，可忽略）
    MV_CC_SetIntValue(handle_, "Width",  width_);
    MV_CC_SetIntValue(handle_, "Height", height_);

    // 帧率
    MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", fps_);

    // 像素格式：尽量输出 Bayer8，统一转 BGR8
    MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);

    // 曝光/增益/白平衡
    MV_CC_SetEnumValue(handle_, "ExposureAuto", auto_exposure_ ? 2 : 0); // 2:Continuous, 0:Off
    if (!auto_exposure_) MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_us_);
    MV_CC_SetFloatValue(handle_, "Gain", gain_);
    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", auto_wb_ ? 2 : 0);

    return MV_OK == MV_CC_StartGrabbing(handle_);
  }

  void grabLoop() {
    running_ = true;
    while (running_ && rclcpp::ok()) {
      MV_FRAME_OUT out{};
      int ret = MV_CC_GetImageBuffer(handle_, &out, 1000);
      if (ret != MV_OK) continue;

      // 转 BGR8
      size_t dst_size = out.stFrameInfo.nWidth * out.stFrameInfo.nHeight * 3;
      if (bgr_.size() < dst_size) bgr_.resize(dst_size);
      MV_CC_PIXEL_CONVERT_PARAM c{};
      c.nWidth = out.stFrameInfo.nWidth;
      c.nHeight = out.stFrameInfo.nHeight;
      c.pSrcData = out.pBufAddr;
      c.nSrcDataLen = out.stFrameInfo.nFrameLen;
      c.enSrcPixelType = out.stFrameInfo.enPixelType;
      c.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
      c.pDstBuffer = bgr_.data();
      c.nDstBufferSize = static_cast<unsigned int>(bgr_.size());
      MV_CC_ConvertPixelType(handle_, &c);

      cv::Mat img(out.stFrameInfo.nHeight, out.stFrameInfo.nWidth, CV_8UC3, bgr_.data());
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
      msg->header.stamp = this->get_clock()->now();
      msg->header.frame_id = frame_id_;
      pub_->publish(*msg);

      MV_CC_FreeImageBuffer(handle_, &out);
    }
  }

  // 成员
  std::string serial_, frame_id_;
  int width_, height_;
  double fps_, exposure_us_, gain_;
  bool auto_exposure_, auto_wb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  void* handle_ = nullptr;
  std::thread grab_thread_;
  std::atomic<bool> running_{false};
  std::vector<uint8_t> bgr_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MvsCameraNode>());
  rclcpp::shutdown();
  return 0;
}
