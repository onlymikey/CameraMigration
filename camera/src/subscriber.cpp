#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>

class CompressedImageSubscriber : public rclcpp::Node
{
public:
  CompressedImageSubscriber()
  : Node("compressed_image_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "camera/image/compressed", 10,
      std::bind(&CompressedImageSubscriber::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // Decodificar la imagen JPEG a cv::Mat
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    if (!frame.empty()) {
      cv::imshow("Subscriber", frame);
      cv::waitKey(1);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressedImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
