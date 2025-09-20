#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CompressedImagePublisher : public rclcpp::Node
{
public:
  CompressedImagePublisher()
  : Node("compressed_image_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/image/compressed", 10);

    cap_.open(0);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara");
      rclcpp::shutdown();
    }

    timer_ = this->create_wall_timer(
      100ms, std::bind(&CompressedImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) return;

    // Comprimir a JPEG
    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80}; // calidad 0-100
    cv::imencode(".jpg", frame, buf, params);

    // Llenar el mensaje
    auto msg = sensor_msgs::msg::CompressedImage();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "camera_frame";
    msg.format = "jpeg";
    msg.data = std::move(buf);

    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressedImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
