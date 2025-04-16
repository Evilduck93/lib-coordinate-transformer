/**
 * @file coordinate_transformer.hpp
 * @brief Интерфейс библиотеки для преобразования координат в ROS2.
 *
 * Данная библиотека реализует преобразование координат между системами отсчёта с использованием tf2,
 * загрузку трансформаций и проверку границ. Интерфейс реализован через классы и методы.
 */

 #ifndef COORDINATE_TRANSFORMER__COORDINATE_TRANSFORMER_HPP_
 #define COORDINATE_TRANSFORMER__COORDINATE_TRANSFORMER_HPP_
 
 #include <rclcpp/rclcpp.hpp>
 #include <geometry_msgs/msg/pose_stamped.hpp>
 #include <geometry_msgs/msg/transform_stamped.hpp>
 #include <geometry_msgs/msg/point.hpp>
 #include <tf2_ros/buffer.h>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 #include <yaml-cpp/yaml.h>
 #include <unordered_map>
 #include <string>
 
 /**
  * @brief Перечисление статусов результата преобразования.
  */
 enum class ResultStatus {
   SUCCESS,            ///< Преобразование выполнено успешно.
   OUT_OF_BOUNDS,      ///< Результат выходит за установленные границы.
   TRANSFORM_NOT_FOUND,///< Не найдена требуемая трансформация.
   INVALID_INPUT       ///< Некорректные входные данные.
 };
 
 /**
  * @brief Класс для преобразования координат между системами отсчёта.
  *
  * Данный класс предоставляет методы:
  * - Прямого преобразования (convert)
  * - Обратного преобразования (inverseConvert)
  * - Загрузки трансформаций из YAML (loadConfig)
  * - Ручного добавления трансформаций (addTransform)
  * - Задания и проверки границ (setBounds)
  *
  * Использует tf2 для получения трансформаций и rclcpp::Node для доступа к параметрам ROS2.
  */
 class CoordinateTransformer {
 public:
   /**
    * @brief Конструктор.
    * @param node Shared pointer на rclcpp::Node для работы с параметрами и логированием.
    */
   explicit CoordinateTransformer(std::shared_ptr<rclcpp::Node> node);
 
   /**
    * @brief Прямое преобразование (из исходной системы в целевую).
    * @param input Исходная поза.
    * @param output Преобразованная поза.
    * @param target_frame Целевая система отсчёта.
    * @return Статус преобразования.
    */
   ResultStatus convert(
     const geometry_msgs::msg::PoseStamped& input,
     geometry_msgs::msg::PoseStamped& output,
     const std::string& target_frame);
 
   /**
    * @brief Обратное преобразование (из целевой системы в исходную).
    * @param input Поза в целевой системе.
    * @param output Преобразованная поза в исходной системе.
    * @param source_frame Исходная система отсчёта.
    * @return Статус преобразования.
    */
   ResultStatus inverseConvert(
     const geometry_msgs::msg::PoseStamped& input,
     geometry_msgs::msg::PoseStamped& output,
     const std::string& source_frame);
 
   /**
    * @brief Ручное добавление трансформации.
    * @param transform Трансформация для добавления.
    */
   void addTransform(const geometry_msgs::msg::TransformStamped& transform);
 
   /**
    * @brief Установка прямоугольных границ для системы отсчёта.
    * @param frame_id Идентификатор системы.
    * @param min Минимальные координаты.
    * @param max Максимальные координаты.
    */
   void setBounds(
     const std::string& frame_id,
     const geometry_msgs::msg::Point& min,
     const geometry_msgs::msg::Point& max);
 
   /**
    * @brief Загрузка конфигурации из YAML-файла.
    * @param file_path Путь к YAML-файлу.
    */
   void loadConfig(const std::string& file_path);
 
 private:
   std::shared_ptr<rclcpp::Node> node_;
   tf2_ros::Buffer tf_buffer_;
   tf2_ros::TransformListener tf_listener_;
 
   struct Bounds {
     geometry_msgs::msg::Point min;
     geometry_msgs::msg::Point max;
   };
 
   std::unordered_map<std::string, Bounds> bounds_map_;
 
   /**
    * @brief Проверяет, находится ли точка внутри заданных границ.
    * @param point Точка для проверки.
    * @param bounds Границы.
    * @return true, если точка внутри границ, иначе false.
    */
   bool isWithinBounds(const geometry_msgs::msg::Point& point, const Bounds& bounds);
 };
 
 #endif  // COORDINATE_TRANSFORMER__COORDINATE_TRANSFORMER_HPP_
 