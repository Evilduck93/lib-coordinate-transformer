/**
 * @file coordinate_transformer.cpp
 * @brief Реализация методов класса CoordinateTransformer.
 */

 #include "coordinate_transformer/coordinate_transformer.hpp"
 #include <chrono>
 #include <tf2/transform_datatypes.h>
 
 using namespace std::chrono_literals;
 
 CoordinateTransformer::CoordinateTransformer(std::shared_ptr<rclcpp::Node> node)
   : node_(node),
     tf_buffer_(node_->get_clock()),
     tf_listener_(tf_buffer_)
 {
 }
 
 ResultStatus CoordinateTransformer::convert(
   const geometry_msgs::msg::PoseStamped& input,
   geometry_msgs::msg::PoseStamped& output,
   const std::string& target_frame)
 {
   try {
     // Пытаемся получить трансформацию с таймаутом 1 секунда
     output = tf_buffer_.transform(input, target_frame, tf2::Duration(1s));
   } catch (const tf2::TransformException &ex) {
     RCLCPP_ERROR(node_->get_logger(), "Ошибка преобразования: %s", ex.what());
     return ResultStatus::TRANSFORM_NOT_FOUND;
   }
   // Если для целевой системы заданы границы, проверяем выход за них
   if (bounds_map_.find(target_frame) != bounds_map_.end()) {
     const auto &b = bounds_map_[target_frame];
     const auto &p = output.pose.position;
     if (p.x < b.min.x || p.y < b.min.y || p.z < b.min.z ||
         p.x > b.max.x || p.y > b.max.y || p.z > b.max.z)
     {
       return ResultStatus::OUT_OF_BOUNDS;
     }
   }
   return ResultStatus::SUCCESS;
 }
 
 ResultStatus CoordinateTransformer::inverseConvert(
   const geometry_msgs::msg::PoseStamped& input,
   geometry_msgs::msg::PoseStamped& output,
   const std::string& source_frame)
 {
   try {
     output = tf_buffer_.transform(input, source_frame, tf2::Duration(1s));
   } catch (const tf2::TransformException &ex) {
     RCLCPP_ERROR(node_->get_logger(), "Ошибка обратного преобразования: %s", ex.what());
     return ResultStatus::TRANSFORM_NOT_FOUND;
   }
   if (bounds_map_.find(source_frame) != bounds_map_.end()) {
     const auto &b = bounds_map_[source_frame];
     const auto &p = output.pose.position;
     if (p.x < b.min.x || p.y < b.min.y || p.z < b.min.z ||
         p.x > b.max.x || p.y > b.max.y || p.z > b.max.z)
     {
       return ResultStatus::OUT_OF_BOUNDS;
     }
   }
   return ResultStatus::SUCCESS;
 }
 
 void CoordinateTransformer::addTransform(const geometry_msgs::msg::TransformStamped& transform)
{
  try {
    tf_buffer_.setTransform(transform, "test", true);  // Используем tf_buffer_ вместо buffer_
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to add transform: %s", ex.what());
  }
}

 
 void CoordinateTransformer::setBounds(
   const std::string& frame_id,
   const geometry_msgs::msg::Point& min,
   const geometry_msgs::msg::Point& max)
 {
   Bounds b;
   b.min = min;
   b.max = max;
   bounds_map_[frame_id] = b;
 }
 
 void CoordinateTransformer::loadConfig(const std::string& file_path)
 {
   try {
     YAML::Node config = YAML::LoadFile(file_path);
     if (config["transforms"]) {
       for (const auto &t : config["transforms"]) {
         geometry_msgs::msg::TransformStamped transform;
         transform.header.frame_id = t["parent_frame"].as<std::string>();
         transform.child_frame_id = t["child_frame"].as<std::string>();
         auto translation = t["translation"];
         transform.transform.translation.x = translation[0].as<double>();
         transform.transform.translation.y = translation[1].as<double>();
         transform.transform.translation.z = translation[2].as<double>();
         auto rotation = t["rotation"];
         transform.transform.rotation.x = rotation[0].as<double>();
         transform.transform.rotation.y = rotation[1].as<double>();
         transform.transform.rotation.z = rotation[2].as<double>();
         transform.transform.rotation.w = rotation[3].as<double>();
         addTransform(transform);
       }
     }
     if (config["bounds"]) {
       for (const auto &b : config["bounds"]) {
         std::string frame = b.first.as<std::string>();
         auto min = b.second["min"];
         auto max = b.second["max"];
         geometry_msgs::msg::Point min_point, max_point;
         min_point.x = min[0].as<double>();
         min_point.y = min[1].as<double>();
         min_point.z = min[2].as<double>();
         max_point.x = max[0].as<double>();
         max_point.y = max[1].as<double>();
         max_point.z = max[2].as<double>();
         setBounds(frame, min_point, max_point);
       }
     }
   } catch (const std::exception& ex) {
     RCLCPP_ERROR(node_->get_logger(), "Ошибка загрузки конфигурации: %s", ex.what());
   }
 }
 
 bool CoordinateTransformer::isWithinBounds(const geometry_msgs::msg::Point& point, const Bounds& bounds)
 {
   return (point.x >= bounds.min.x && point.x <= bounds.max.x &&
           point.y >= bounds.min.y && point.y <= bounds.max.y &&
           point.z >= bounds.min.z && point.z <= bounds.max.z);
 }
 