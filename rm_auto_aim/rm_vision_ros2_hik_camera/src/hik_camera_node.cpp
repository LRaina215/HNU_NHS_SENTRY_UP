#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options)
  : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");
    MV_CC_DEVICE_INFO_LIST DeviceList;

    //枚举设备
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &DeviceList);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", DeviceList.nDeviceNum);
    while (DeviceList.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &DeviceList);
    }

    //创建相机句柄，打开设备
    MV_CC_CreateHandle(&camera_handle_, DeviceList.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle_);

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);

    //设置相机分辨率

    //获取最相机内置分辨率参数
    // MV_CC_GetWidth(camera_handle_,&camera_width);
    // MV_CC_GetHeight(camera_handle_,&camera_height);

//            //设置相机分辨率，要求为nInc（8）的倍数
//            setheight = (unsigned int)camera_height.nInc * 80;
//            setwidth = (unsigned int)camera_width.nInc * 80;

    // set param from camera_params.yaml  --Hooray
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 640);

    this->get_parameter("image_width", setwidth);
    this->get_parameter("image_height", setheight);

    setwidth = (unsigned int)(setwidth);
    setheight = (unsigned int)(setheight);

    AOIX = (unsigned int)((int)((1624 - setwidth) / 16) * 8);
    AOIY = (unsigned int)((int)((1240 - setheight) / 16) * 8);

    nRet = MV_CC_SetHeight(camera_handle_, setheight);
    nRet = MV_CC_SetWidth(camera_handle_, setwidth);
//            std::cout << "camera_height.nInc:" << camera_height.nInc << std::endl;
//            std::cout << "camera_width.nInc:" << camera_width.nInc << std::endl;

    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set Height and Width successfully!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Set Height and Width failed!");
    }

    //设置相机  AOI
    nRet = MV_CC_SetAOIoffsetX(camera_handle_, AOIX);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set AOIoffsetX successfully!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Set AOIoffsetX failed!");
    }
    nRet = MV_CC_SetAOIoffsetY(camera_handle_, AOIY);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set AOIoffsetY successfully!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Set AOIoffsetY failed!");
    }


    // Print camera info
    RCLCPP_INFO(this->get_logger(), "Camera info:");
    RCLCPP_INFO(this->get_logger(), "nWidthMax: %d", img_info_.nWidthMax);
    RCLCPP_INFO(this->get_logger(), "nHeightMax: %d", img_info_.nHeightMax);
    // RCLCPP_INFO(this->get_logger(), "nWidth: %d", img_info_.nWidthValue);
    // RCLCPP_INFO(this->get_logger(), "nHeight: %d", img_info_.nHeightValue);
    RCLCPP_INFO(this->get_logger(), "setwidth: %d", setwidth);
    RCLCPP_INFO(this->get_logger(), "setheight: %d", setheight);
    image_msg_.data.reserve(setheight * setwidth * 3);

    // Init convert param
    ConvertParam_.nWidth = setwidth;
    ConvertParam_.nHeight = setheight;
    ConvertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();
    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
    // Load camera params  --Hooray
    auto camera_params_url =
      this->declare_parameter(
      "camera_params_url",
      "package://hik_camera/config/camera_params.yaml");
    if (camera_info_manager_->validateURL(camera_params_url)) {
      RCLCPP_INFO(this->get_logger(), "Load params success!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera params URL: %s", camera_params_url.c_str());
    }


    //  Init Rotate param  --Hooray
    RotateParam_.enPixelType = PixelType_Gvsp_RGB8_Packed;
    RotateParam_.nWidth = setwidth;
    RotateParam_.nHeight = setheight;
    RotateParam_.enRotationAngle = MV_IMAGE_ROTATE_180;


    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
        MV_FRAME_OUT OutFrame;

        RCLCPP_INFO(this->get_logger(), "Publishing image!");

        image_msg_.header.frame_id = "camera_optical_frame";
        image_msg_.encoding = "rgb8";

        // 🚨 升级版看门狗：记录所有类型的连续失败次数
        int consecutive_fail_count = 0; 
        const int MAX_FAILS = 20;       

        while (rclcpp::ok()) {
          nRet = MV_CC_GetImageBuffer(camera_handle_, &OutFrame, 90);
          
          if (MV_OK == nRet) {
            // 💡 关键：只要成功拿到有效图像，所有失败计数立刻清零！
            consecutive_fail_count = 0; 

            ConvertParam_.pDstBuffer = image_msg_.data.data();
            ConvertParam_.nDstBufferSize = image_msg_.data.size();
            ConvertParam_.pSrcData = OutFrame.pBufAddr;
            ConvertParam_.nSrcDataLen = OutFrame.stFrameInfo.nFrameLen;

            RotateParam_.pSrcData = OutFrame.pBufAddr;
            RotateParam_.nSrcDataLen = OutFrame.stFrameInfo.nFrameLen;
            RotateParam_.pDstBuf = image_msg_.data.data();
            RotateParam_.nDstBufSize = image_msg_.data.size();

            ConvertParam_.enSrcPixelType = OutFrame.stFrameInfo.enPixelType;

            MV_CC_ConvertPixelType(camera_handle_, &ConvertParam_);

            camera_info_msg_.header.stamp = image_msg_.header.stamp = this->now();
            image_msg_.height = OutFrame.stFrameInfo.nHeight;
            image_msg_.width = OutFrame.stFrameInfo.nWidth;
            image_msg_.step = OutFrame.stFrameInfo.nWidth * 3;
            image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);
            camera_pub_.publish(image_msg_, camera_info_msg_);

            MV_CC_FreeImageBuffer(camera_handle_, &OutFrame);
            
          } else {
            // ==========================================
            // 🚨 异常处理与硬退出机制 (全类型报错拦截)
            // ==========================================
            consecutive_fail_count++; // 不管什么错，先记上一笔
            
            // 区分一下日志打印，方便你以后排查是超时还是别的玄学报错
            if (nRet == (int)MV_E_NODATA) {
                RCLCPP_WARN(this->get_logger(), "获取图像帧超时，跳过... (连续失败: %d/%d)", consecutive_fail_count, MAX_FAILS);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Get buffer failed! nRet: [%x] (连续失败: %d/%d)", nRet, consecutive_fail_count, MAX_FAILS);
            }
            
            // 💀 触发看门狗死刑判决
            if (consecutive_fail_count >= MAX_FAILS) {
                RCLCPP_ERROR(this->get_logger(), "连续 %d 次获取图像失败！执行强制清理并退出...", MAX_FAILS);
                
                // 1. 必须先释放海康底层的句柄！否则外部脚本瞬间拉起时会报“设备被占用”
                if (camera_handle_) {
                    MV_CC_StopGrabbing(camera_handle_);
                    MV_CC_CloseDevice(camera_handle_);
                    MV_CC_DestroyHandle(&camera_handle_);
                    camera_handle_ = nullptr;
                }

                // 2. 优雅切断 ROS 2 通讯
                rclcpp::shutdown(); 
                
                // 3. 强制以错误码(1)退出，呼叫外部 sh 脚本进行重启
                exit(1);            
            }
            
            continue; // 还没到阈值，继续尝试下一帧
          }
        }
      }};
  }

  ~HikCameraNode()
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  void declareParameters()
  {
    // set all params from yaml  --Hooray refactored
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE fValue;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &fValue);
    param_desc.integer_range[0].from_value = 1000;
    param_desc.integer_range[0].to_value = 60000;
    this->declare_parameter<double>("exposure_time", 10000.0, param_desc);
    this->get_parameter("exposure_time", exposure_time);
    // nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    nRet = MV_CC_SetExposureTime(camera_handle_, exposure_time);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set ExposureTime successfully!");
      RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);
    } else {
      RCLCPP_INFO(this->get_logger(), "Set ExposureTime failed!");
    }
    //MV_CC_SetExposureAutoMode(camera_handle_, 2);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &fValue);
    param_desc.integer_range[0].from_value = fValue.fMin;
    param_desc.integer_range[0].to_value = fValue.fMax;
    this->declare_parameter<double>("gain", 20.0, param_desc);
    this->get_parameter("gain", gain);
    nRet = MV_CC_SetGain(camera_handle_, gain);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set Gain successfully!");
      RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
    } else {
      RCLCPP_INFO(this->get_logger(), "Set gain failed!");
    }

    this->declare_parameter<int>("gain_mode", 1, param_desc);
    this->get_parameter("gain_mode", gain_mode);
    nRet = MV_CC_SetGainMode(camera_handle_, gain_mode);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set GainMode successfully!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Set GainMode failed!");
    }
    //MV_CC_SetAcquisitionMode(camera_handle_, 2);

    nRet = MV_CC_SetBalanceWhiteAuto(camera_handle_, 0);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set BalanceWhiteAuto successfully!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Set BalanceWhiteAuto failed!");
    }


    // PixelFormat 数据格式
    nRet = MV_CC_SetPixelFormat(camera_handle_, PixelType_Gvsp_RGB8_Packed);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "set PixelFormat successfully!");


    } else {
      RCLCPP_INFO(this->get_logger(), "set PixelFormat failed!");
    }

    //FrameRate 帧率
    this->declare_parameter<int>("framerate", 120);
    this->get_parameter("framerate", framerate);

    MV_CC_SetFrameRate(camera_handle_, framerate);
    if (nRet == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "Set FrameRate successfully!");
      RCLCPP_INFO(this->get_logger(), "Framerate:%d", framerate);
    } else {
      RCLCPP_INFO(this->get_logger(), "Set FrameRate failed!");
    }


    // if rotation image
//            this->declare_parameter<bool>("ifrotate", 0);
//            this->get_parameter("ifrotate", ifrotate);
//            if(ifrotate == 0)
//            {
//              RCLCPP_INFO(this->get_logger(), "Rotate Image OFF!");
//            }
//            else
//            {
//              RCLCPP_INFO(this->get_logger(), "Rotate Image On!");
//            }


  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;
  MVCC_FLOATVALUE timeframe;

  image_transport::CameraPublisher camera_pub_;

  int nRet = MV_OK;
  void * camera_handle_;

  MVCC_INTVALUE camera_width, camera_height;


  unsigned int setwidth, setheight, AOIX, AOIY;

  int framerate;

  double exposure_time;

  int gain_mode;

  float gain;

  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM ConvertParam_;

//        bool ifrotate;


  MV_CC_ROTATE_IMAGE_PARAM RotateParam_;

  MV_IMG_ROTATION_ANGLE rotation_angle;


  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
