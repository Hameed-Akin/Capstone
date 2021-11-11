#ifndef LOGGER_H
#define LOGGER_H
#include <fstream>
#include <ros/ros.h>

#include "openpose_parser/LoadHands.h"


class Logger
{
public:
  Logger();
  ~Logger();

  void write(std_msgs::Header header,geometry_msgs::PoseArray left,geometry_msgs::PoseArray right,float min_distance_left,float min_distance_right);

private:
  std::ofstream log_;


};

#endif // LOGGER_H

