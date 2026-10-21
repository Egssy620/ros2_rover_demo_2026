#include "nedo_capture/nedo_cap_node.hpp"

namespace nedo_capture
{

    NedoCapNode::NedoCapNode(const rclcpp::NodeOptions &options)
        : Node("nedo_cap_node", options)
    {
        device = this->declare_parameter("device", "/dev/video4");
        // device = this->declare_parameter("device", "/dev/front_camera");
        frame_id = this->declare_parameter("frame_id", "camera_frame");
        std::string pipeline_param = this->declare_parameter("pipeline", "");
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);

        // auto qos = rclcpp::SystemDefaultsQoS();
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("capture", qos);
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(20),
            std::bind(&NedoCapNode::timer_callback, this));

        // パイプラインの設定: パラメータで指定されていればそれを使用、なければdeviceから構築
        std::string pipeline;
        if (!pipeline_param.empty())
        {
            pipeline = pipeline_param;
            RCLCPP_INFO(this->get_logger(), "Using pipeline from parameter: %s", pipeline.c_str());
        }
        else
        {
            // GStreamerパイプラインの定義 (MJPG, 4K, 30fps)
            pipeline = "v4l2src device=" + device +
                       " ! image/jpeg, width=3840, height=2160, framerate=30/1 ! jpegdec ! videoconvert ! appsink";
            RCLCPP_INFO(this->get_logger(), "Using default pipeline for device %s: %s", device.c_str(), pipeline.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Attempting to open camera with pipeline: %s", pipeline.c_str());
        cap.open(pipeline, cv::CAP_GSTREAMER);
        if (!cap.isOpened())
        {
            // フォールバック: より基本的な設定を試行
            RCLCPP_WARN(this->get_logger(), "Primary pipeline failed, trying basic V4L2 format for device %s...", device.c_str());
            pipeline = "v4l2src device=" + device +
                       " ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink";
            RCLCPP_INFO(this->get_logger(), "Trying fallback pipeline: %s", pipeline.c_str());
            cap.open(pipeline, cv::CAP_GSTREAMER);

            if (!cap.isOpened())
            {
                // 最終フォールバック: 指定されたデバイスを直接開く
                RCLCPP_WARN(this->get_logger(), "GStreamer failed, trying to open device directly: %s", device.c_str());
                // deviceから数値を抽出（/dev/video4 -> 4）
                std::string device_num = device.substr(device.find_last_of("/") + 1);
                if (device_num.find("video") != std::string::npos)
                {
                    device_num = device_num.substr(5); // "video"を除去
                }
                try {
                    int device_id = std::stoi(device_num);
                    RCLCPP_INFO(this->get_logger(), "Opening device ID: %d (from %s)", device_id, device.c_str());
                    cap.open(device_id);
                    if (cap.isOpened())
                    {
                        RCLCPP_INFO(this->get_logger(), "Successfully opened device ID %d (device: %s)", device_id, device.c_str());
                    }
                } catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse device number from %s", device.c_str());
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Successfully opened camera with fallback pipeline for device %s", device.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Successfully opened camera with primary pipeline for device %s", device.c_str());
        }

        if (!cap.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera after all attempts.");
            RCLCPP_ERROR(this->get_logger(), "Device parameter: %s", device.c_str());
            RCLCPP_ERROR(this->get_logger(), "Last pipeline used: %s", pipeline.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Camera opened successfully - Device: %s, Pipeline: %s", device.c_str(), pipeline.c_str());
    }
    void NedoCapNode::timer_callback()
    {
        if (cap.isOpened())
        {
            cv::Mat frame;
            cap >> frame;
            if (!frame.empty())
            {
                std_msgs::msg::Header header = std_msgs::msg::Header();
                header.frame_id = frame_id;
                header.stamp = this->now();
                auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                publisher_->publish(*msg);
            }
        }
    }

    NedoCapNode::~NedoCapNode()
    {
        if (cap.isOpened())
        {
            cap.release();
        }
    }

} // namespace nedo_capture

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nedo_capture::NedoCapNode)