
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// 1. Incluir el tipo de mensaje de Imagen
#include "sensor_msgs/msg/image.hpp" 

// 2. Incluir OpenCV
#include <opencv2/opencv.hpp> 

// 3. Incluir cv_bridge para la conversión
#include "cv_bridge/cv_bridge.hpp" 

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher()
  : Node("image_publisher")
  {
    // Crear el publicador para el tópico "camera/image_raw"
    // El tipo de mensaje ahora es sensor_msgs::msg::Image
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // Ruta a la imagen que quieres publicar
    // CAMBIA ESTO a la ruta de tu propia imagen
    std::string image_path = "ruta/a/tu/imagen.jpg"; 
    
    // Cargar la imagen usando OpenCV
        cv::VideoCapture cap(0);

    cap >> image_; // or cap.read(frame);
  // Open the default camera (usually the first camera device)
      cap.release();


    // Verificar si la imagen se cargó correctamente
    if (image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "¡Fallo al cargar la imagen! Verifica la ruta: %s", image_path.c_str());
      // Detener la ejecución si no se puede cargar la imagen
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Imagen cargada exitosamente. Publicando...");

    // Crear un temporizador para llamar al callback cada segundo (1 Hz)
    timer_ = this->create_wall_timer(
      1s, std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // Crear un mensaje de imagen de ROS
    auto msg = std::make_unique<sensor_msgs::msg::Image>();

    // Obtener el tiempo actual para la cabecera del mensaje
    auto timestamp = this->get_clock()->now();

    // Convertir la imagen de cv::Mat a un mensaje de ROS usando cv_bridge
    // cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg(*msg);
    // La forma moderna y más completa es:
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_)
              .toImageMsg(*msg);
    
    // Rellenar la cabecera del mensaje
    msg->header.stamp = timestamp;
    msg->header.frame_id = "camera_frame"; // Un nombre de frame de ejemplo

    // Publicar el mensaje
    publisher_->publish(*msg);
  }

  // Declaración de miembros
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  cv::Mat image_; // Objeto de OpenCV para almacenar la imagen
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}