#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <image_transport/image_transport.h> //includes all image headers
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sstream>

void processImage(const sensor_msgs::ImageConstPtr& msg, double rotAngle)
{
  std::cout << msg->header.stamp.sec << "." << msg->header.stamp.nsec/1000 << std::endl;
}


int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "USAGE: unbag XXXX.bag folder" << std::endl;
        return 1;
    }

    rosbag::Bag bag;

    try{
        bag.open(argv[1], rosbag::bagmode::Read);
//        bagR.open(argv[2], rosbag::bagmode::Read);


        std::vector<std::string> topics;
        topics.push_back(std::string("/camera/color/image_raw"));


        rosbag::View view(bag, rosbag::TopicQuery(topics));
        int count=0;

        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::ImageConstPtr msg = m.instantiate<sensor_msgs::Image>();
            if (msg != nullptr)
            {
              cv_bridge::CvImagePtr cv_ptr;
              try
              {
                  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

                  cv::imshow("OpenCV viewer RGB", cv_ptr->image);
                  cv::waitKey(3);

                  std::stringstream filename;
                  filename << argv[2] << "/" << msg->header.seq << ".png";
                  cv::imwrite(filename.str(),cv_ptr->image);

              }

              catch (cv_bridge::Exception& e)
              {
                  ROS_ERROR("cv_bridge exception: %s", e.what());
              }

            }

            ROS_INFO_STREAM("frame:" << count++ );

        }
        bag.close();
    }
    catch(rosbag::BagUnindexedException){
        std::cout << "Bag unindexed!" << std::endl;
    }
    catch(ros::Exception ex){
        ROS_ERROR("%s",ex.what());
        std::cout << ex.what() << std::endl;
    }

	return 0;
	
	
}
