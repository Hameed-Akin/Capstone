#include "logger.h"
#include <chrono>
#include <string>

//#include "time.h"

Logger::Logger()
{

  time_t t = time(0);   // get time now
  struct tm * now = localtime( & t );

  char buffer [80];
  strftime (buffer,80,"%Y-%m-%d-%H-%M-%S",now);

  std::string filename = "/tmp/handspose_" + std::string(buffer) + ".txt";

  log_.open (filename, std::ios::out | std::ios::trunc);

  log_ << "seq" << " " << "left_pts_total" << " " << "right_pts_total" <<  " " << "left_0_x  left_0_y  right_0_x  right_0_y  min_dist_left  min_dist_right" << std::endl;

}

Logger::~Logger()
{
  log_.close();
}

void Logger::write(std_msgs::Header header,geometry_msgs::PoseArray left,geometry_msgs::PoseArray right,float min_dist_left,float min_dist_right ){

  log_ << header.seq << " " << left.poses.size() << " " << right.poses.size() << " " ;
  for(auto pose : left.poses  ){
      log_ << pose.position.x << " " << pose.position.y << " " ;
  }

  for(auto pose : right.poses  ){
      log_ << pose.position.x << " " << pose.position.y << " " ;
  }

  log_ << min_dist_left << " " << min_dist_right << " ";
  log_ << std::endl;
}
