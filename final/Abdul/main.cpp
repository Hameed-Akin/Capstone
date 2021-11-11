

#include <iostream>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>
#include <string>

// #include <nlohmann/json.hpp>

using namespace std;

// stores the x and y values
struct coordinates
{
  double x;
  double y;
};

coordinates point5;
coordinates point9;
coordinates point13;
coordinates point17;



coordinates getkeypointcoordinates(coordinates coordinates_, int keypoint, const Json::Value keypoints)
{
  coordinates_.x = keypoints[0]["hand_right_keypoints_2d"][keypoint * 3].asDouble();
  coordinates_.y = keypoints[0]["hand_right_keypoints_2d"][(keypoint * 3) + 1].asDouble();

  return coordinates_;
} 

void printcoordinates(coordinates coordinates_)
{
  cout<< "[x,y] = " << "[" << coordinates_.x << "," << coordinates_.y << "]" <<endl;
}



int main(int argc, char *argv[])
{
    std::string json_filename;

    //Expects filename as parameter
    if(argc>1){
        json_filename = argv[1];
    }
    else {
        std::cout << "Run with ./" << argv[0] << " location of json file " << std::endl;
        return -1;
    }

  ifstream ifs(json_filename); // insert name of the supposed josn file
  Json::Reader reader;
  Json::Value obj;
  reader.parse(ifs, obj);


  const Json::Value keypoints = obj["people"];

  // keypoints[0]["hand_right_keypoints_2d"][15].asDouble();

  point5 = getkeypointcoordinates(point5,5,keypoints);
  point9 = getkeypointcoordinates(point5,9,keypoints);
  point13 = getkeypointcoordinates(point5,13,keypoints);
  point17 = getkeypointcoordinates(point5,13,keypoints);

  printcoordinates(point17);
 

}





// g++ -o Main main.cpp -ljsoncpp


