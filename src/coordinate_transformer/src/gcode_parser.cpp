#include "coordinate_transformer/gcode_parser.hpp"

GCodeParser::GCodeParser(const std::string &frame_id)
: current_index_(0), default_frame_(frame_id)
{
}

bool GCodeParser::loadFile(const std::string &filename)
{
  poses_.clear();
  current_index_ = 0;

  std::ifstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == ';') continue;  // Пропустить комментарии

    std::istringstream iss(line);
    std::string word;
    double x = 0.0, y = 0.0, z = 0.0;
    bool has_x = false, has_y = false, has_z = false;

    while (iss >> word) {
      if (word[0] == 'X') {
        x = std::stod(word.substr(1));
        has_x = true;
      } else if (word[0] == 'Y') {
        y = std::stod(word.substr(1));
        has_y = true;
      } else if (word[0] == 'Z') {
        z = std::stod(word.substr(1));
        has_z = true;
      }
    }

    if (has_x || has_y || has_z) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = default_frame_;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = z;
      pose.pose.orientation.w = 1.0;  // Единичная ориентация
      poses_.push_back(pose);
    }
  }

  return true;
}

bool GCodeParser::getNextPose(geometry_msgs::msg::PoseStamped &pose)
{
  if (current_index_ >= poses_.size()) {
    return false;
  }

  pose = poses_[current_index_++];
  return true;
}

void GCodeParser::reset()
{
  current_index_ = 0;
}
