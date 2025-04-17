#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <geometry_msgs/msg/pose_stamped.hpp>

class GCodeParser
{
public:
  explicit GCodeParser(const std::string &frame_id = "tool0");

  // Загрузить G-code из файла
  bool loadFile(const std::string &filename);

  // Получить следующую позу, возвращает false если G-code закончился
  bool getNextPose(geometry_msgs::msg::PoseStamped &pose);

  // Сброс текущего состояния (начать сначала)
  void reset();

private:
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  size_t current_index_;
  std::string default_frame_;
};
