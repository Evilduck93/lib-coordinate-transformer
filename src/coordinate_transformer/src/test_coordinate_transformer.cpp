#include <rclcpp/rclcpp.hpp>
#include "coordinate_transformer/coordinate_transformer.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("coordinate_transformer_example");

  // Создаем объект трансформера, передавая ноду
  auto transformer = std::make_shared<CoordinateTransformer>(node);

  // Загружаем конфигурацию (например, трансформации и границы) из YAML-файла
  transformer->loadConfig("config/transforms.yaml");

  // Задаем границы для системы "robot_base"
  geometry_msgs::msg::Point min_point, max_point;
  min_point.x = -1.0;  min_point.y = -1.0;  min_point.z = 0.0;
  max_point.x =  1.0;  max_point.y =  1.0;  max_point.z = 1.5;
  transformer->setBounds("robot_base", min_point, max_point);

  // Пример преобразования координаты
  geometry_msgs::msg::PoseStamped input, output;
  input.header.frame_id = "robot_base";
  input.pose.position.x = 0.5;
  input.pose.position.y = 0.2;
  input.pose.position.z = 0.3;
  input.pose.orientation.w = 1.0;
  input.pose.orientation.x = input.pose.orientation.y = input.pose.orientation.z = 0.0;

  auto status = transformer->convert(input, output, "world");

  if (status == ResultStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Преобразование выполнено успешно");
    // Здесь можно обработать результат преобразования (например, вывести координаты)
  } else {
    RCLCPP_ERROR(node->get_logger(), "Ошибка преобразования");
  }

  rclcpp::shutdown();
  return 0;
}
