#include <rclcpp/rclcpp.hpp>
#include "coordinate_transformer/coordinate_transformer.hpp"
#include "coordinate_transformer/gcode_parser.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("coordinate_transformer_example");

  auto transformer = std::make_shared<CoordinateTransformer>(node);
  transformer->loadConfig("config/transforms.yaml");

  geometry_msgs::msg::Point min_point, max_point;
  min_point.x = -1.0;  min_point.y = -1.0;  min_point.z = 0.0;
  max_point.x =  1.0;  max_point.y =  1.0;  max_point.z = 1.5;
  transformer->setBounds("robot_base", min_point, max_point);

  // Загрузка G-code файла
  GCodeParser parser("robot_base");
  if (!parser.loadFile("config/sample.gcode")) {
    RCLCPP_ERROR(node->get_logger(), "Не удалось загрузить G-code файл");
    rclcpp::shutdown();
    return 1;
  }

  geometry_msgs::msg::PoseStamped input_pose, transformed_pose;
  while (parser.getNextPose(input_pose)) {
    auto status = transformer->convert(input_pose, transformed_pose, "world");

    if (status == ResultStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Преобразовано: [%.2f, %.2f, %.2f]",
                  transformed_pose.pose.position.x,
                  transformed_pose.pose.position.y,
                  transformed_pose.pose.position.z);
    } else {
      RCLCPP_WARN(node->get_logger(), "Ошибка преобразования координаты");
    }
  }

  rclcpp::shutdown();
  return 0;
}
