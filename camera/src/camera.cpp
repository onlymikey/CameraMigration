#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("image_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    cv::VideoCapture cap(0);
    cap >> image_;
    cap.release();

    if (image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "¡Fallo al capturar la imagen de la cámara!");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Imagen capturada exitosamente. Publicando...");

    timer_ = this->create_wall_timer(1s, std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto timestamp = this->get_clock()->now();

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
    msg->header.stamp = timestamp;
    msg->header.frame_id = "camera_frame";

    publisher_->publish(*msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::Mat image_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
