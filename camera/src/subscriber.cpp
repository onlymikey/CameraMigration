#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/highgui.hpp> // Necesario para cv::imshow y cv::waitKey

// Usamos std::placeholders para el callback de la subscripción
using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    // Crear la subscripción al tópico "camera/image_raw"
    // El '10' es la profundidad de la cola (QoS)
    // El tercer argumento es el callback que se ejecutará al recibir un mensaje
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10, std::bind(&ImageSubscriber::topic_callback, this, _1));
  }

private:
  // Esta función se ejecuta cada vez que llega un mensaje de imagen
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "¡Imagen recibida!");

    try
    {
      // Convertir el mensaje de ROS (msg) a una imagen de OpenCV (cv::Mat)
      // Usamos toCvShare para evitar una copia si es posible, pero toCvCopy es más seguro
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

      // Mostrar la imagen en una ventana llamada "Image Viewer"
      cv::imshow("Image Viewer", cv_ptr->image);

      // Esperar 1 milisegundo. Esto es VITAL para que OpenCV pueda procesar
      // los eventos de la GUI y dibujar la imagen.
      cv::waitKey(1); 
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error de cv_bridge: %s", e.what());
    }
  }

  // Declaración del miembro de la subscripción
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Se crea y se pone a "girar" el nodo subscriptor
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}