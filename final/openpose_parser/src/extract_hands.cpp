#include "ros/ros.h"
#include <string>
#include "openpose_parser/LoadHands.h"
#include <iostream>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>

using namespace std;

// stores the x and y values
struct coordinates
{
  double x;
  double y;
};

coordinates getrightcoordinates(coordinates coordinates_, int keypoint, const Json::Value keypoints)
{
  coordinates_.x = keypoints[0]["hand_right_keypoints_2d"][keypoint * 3].asDouble();
  coordinates_.y = keypoints[0]["hand_right_keypoints_2d"][(keypoint * 3) + 1].asDouble();

  return coordinates_;
}

coordinates getleftcoordinates(coordinates coordinates_, int keypoint, const Json::Value keypoints)
{
  coordinates_.x = keypoints[0]["hand_left_keypoints_2d"][keypoint * 3].asDouble();
  coordinates_.y = keypoints[0]["hand_left_keypoints_2d"][(keypoint * 3) + 1].asDouble();

  return coordinates_;
}

void printcoordinates(coordinates coordinates_)
{
  cout << "[x,y] = "
       << "[" << coordinates_.x << "," << coordinates_.y << "]" << endl;
}

coordinates right_;
coordinates left_;

class ExtractHands
{
private:
  ros::NodeHandle n_;
  ros::ServiceServer service_;
  std::string filename_;

public:
  ExtractHands()
  {

    // Few lines ot get a parameter that can be passed as string here `_example:=foo` for instance
    ros::NodeHandle pn("~");
    // std::string example;
    // std::string filename_;

    // filename_ ="/home/hameed/catkin_ws/src/human_skirting/Abdul/13689_keypoints.json";
    // filename_ = "/home/hameed/catkin_ws/src/human_skirting/Abdul/" + filename_ + "_keypoints.json";

    filename_ = "/home/hameed/catkin_ws/src/human_skirting/Abdul/" + to_string(13689) + "_keypoints.json";

    pn.param<std::string>("filename_", filename_);
    // pn.getParam("filename_", filename_);

    // pn.param<std::string>("filename_",filename_ );
    ROS_INFO_STREAM("file name " << filename_);

    // pn.getParam("filename_",filename_ )

    //Allowing an incoming service on /extract_hands
    service_ = n_.advertiseService("extract_hands", &ExtractHands::extractData, this);
  }

  bool extractData(openpose_parser::LoadHands::Request &request,
                   openpose_parser::LoadHands::Response &response)

  {
    //When an incoming call arrives, we can respond to it here
    ROS_INFO_STREAM("Got a call, should load file with id:" << request.header.seq);

    //Sending back for now a bogus location in image as only element of arrray
    // geometry_msgs::Pose right;
    // right.position.x = 600;
    // right.position.y = 360;

    // geometry_msgs::Pose left;
    // left.position.x = 680;
    // left.position.y = 360;

    // / home / hameed / Capstone / 20210831_dataset / wool_json / 7919.json

        // filename_ = "/home/hameed/catkin_ws/src/human_skirting/Abdul/" + to_string(request.header.seq) + ".json";

    filename_ = "/home/hameed/Capstone/20210831_dataset/wool_json/" + to_string(request.header.seq) + ".json";
    ifstream ifs(filename_);
    Json::Reader reader;
    Json::Value obj;
    reader.parse(ifs, obj);

    const Json::Value keypoints = obj["people"];

    right_ = getrightcoordinates(right_, 9, keypoints);
    left_ = getleftcoordinates(left_, 9, keypoints);

    geometry_msgs::Pose right;
    right.position.x = right_.x;
    right.position.y = right_.y;

    // printcoordinates(point5);

    geometry_msgs::Pose left;
    left.position.x = left_.x;
    left.position.y = left_.y;

    // std::string filename = "/tmp/handspose_" + filename_ + ".json";

    // cout<< filename;

    // printcoordinates(point17);

    response.success = true;
    response.right.header.seq = request.header.seq;
    response.right.poses.push_back(right);
    response.left.header.seq = request.header.seq;
    response.left.poses.push_back(left);

    return true; //We retrun true to indicate the service call sucsseded (your responce should indicate a value)
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "extract_hands");

  std::string json_filename;



  ExtractHands ExtractHands;

  ros::spin();

  return 0;
}
