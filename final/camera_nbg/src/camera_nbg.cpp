/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Camera BG filter based off of RTABMAP_ROS/RGBD_Odometry
#include "camera_nbg.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h> //to convert pcl data to ros data
#include <pcl/io/ply_io.h>                   // to load ply of the table
#include <pcl/common/distances.h>            //to use euclidian distance between points

using namespace pcl_conversions;
using namespace pcl::io;

Camera_Nbg::Camera_Nbg(int argc, char *argv[]) : sync_(0), logger_(), pointCloudMessage_(new sensor_msgs::PointCloud2)
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  if (!readParameters(pnh))
    ros::requestShutdown();
  pointCloudPublisher_ = nh.advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 1, true);
  initialize(nh);

  int queueSize = 5;
  pnh.param("queue_size", queueSize, queueSize);

  ros::NodeHandle rgb_nh(nh, "camera/color/");
  ros::NodeHandle depth_nh(nh, "camera/aligned_depth_to_color/");
  ros::NodeHandle rgb_pnh(pnh, "camera/color/");
  ros::NodeHandle depth_pnh(pnh, "camera/aligned_depth_to_color/");
  image_transport::ImageTransport rgb_it(rgb_nh);
  image_transport::ImageTransport depth_it(depth_nh);

  image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
  image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

  //publisher
  ros::NodeHandle p_rgb_nh(nh, "pb_rgb");
  ros::NodeHandle p_depth_nh(nh, "pb_depth");
  image_transport::ImageTransport p_rgb_it(p_rgb_nh);
  //  image_transport::ImageTransport p_depth_it(p_depth_nh);
  std::string image_topic = "/camera_nbg/rgb/image_rect_color";
  //  std::string depth_topic = "/camera_nbg/depth_registered/image_raw";
  pub_image = p_rgb_it.advertise(image_topic, 2); //** changed 1 to 2
  //  pub_depth = p_depth_it.advertise(depth_topic, 1);
  pub_pc = depth_nh.advertise<sensor_msgs::PointCloud2>("points", 1, true);
  //Publishing markers ... just to show how
  viz_pub_ = depth_nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 3, false);

  //subscribe, we need the hint of image_raw for the synchornizer
  image_mono_sub_.subscribe(rgb_it, rgb_nh.resolveName("image_raw"), 1, hintsRgb);
  image_depth_sub_.subscribe(depth_it, depth_nh.resolveName("image_raw"), 1, hintsDepth);
  info_sub_.subscribe(rgb_nh, "camera_info", 1);

  ROS_INFO("\n%s subscribed to:\n   %s,\n   %s,\n   %s",
           ros::this_node::getName().c_str(),
           image_mono_sub_.getTopic().c_str(),
           image_depth_sub_.getTopic().c_str(),
           info_sub_.getTopic().c_str());

  ROS_INFO("\n%s publishing to: \n   %s",
           ros::this_node::getName().c_str(),
           image_topic.c_str());

  //Inserting calling service
  client_ = nh.serviceClient<openpose_parser::LoadHands>("extract_hands");

  sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queueSize),
                                                          image_mono_sub_, image_depth_sub_, info_sub_);
  sync_->registerCallback(boost::bind(&Camera_Nbg::callback, this, _1, _2, _3));
}

Camera_Nbg::~Camera_Nbg()
{
  delete sync_;
}

void Camera_Nbg::callback(const sensor_msgs::ImageConstPtr &image,
                          const sensor_msgs::ImageConstPtr &depth,
                          const sensor_msgs::CameraInfoConstPtr &cameraInfo)
{

  if (!(image->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
        image->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
        image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
        image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) ||
      !(depth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0 ||
        depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0 ||
        depth->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0))
  {
    ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 (mono8 recommended) and image_depth=16UC1,32FC1,mono16");
    return;
  }
  else if (depth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0)
  {
    static bool warned = false;
    if (!warned)
    {
      ROS_WARN("Input depth type is 32FC1, please use type 16UC1 or mono16 for depth. The depth images "
               "will be processed anyway but with a conversion. This warning is only be printed once...");
      warned = true;
    }
  }

  if (image->data.size() && depth->data.size() && cameraInfo->K[4] != 0)
  {
    cv_bridge::CvImagePtr ptrImage;
    //cv_bridge::CvImagePtr ptrDepth;
    cv_bridge::CvImageConstPtr ptrDepth;

    //Convert depth
    try
    {
      ptrDepth = cv_bridge::toCvShare(depth, "32FC1");
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("Failed to transform depth image. cv_bridge exception: %s", e.what());
    }
    // Convert rgb
    try
    {
      ptrImage = cv_bridge::toCvCopy(image); //, "mono8");   //Note: Why is this mono...
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //! Point cloud message to publish.
    if (pub_pc.getNumSubscribers() > 0)
    {

      pclPoint::Ptr cloud(new pclPoint());
      std::cout << "going to convert depth" << std::endl;

      convertDepthToPointCloud(ptrDepth, cloud, cameraInfo);

      std::cout << "converted depth" << std::endl;

      sensor_msgs::PointCloud2::Ptr pointCloudMessage(new sensor_msgs::PointCloud2());
      toROSMsg(*cloud, *pointCloudMessage);

      std::cout << "converted message" << std::endl;

      //I don't think there is a way to copy the header (all the fields), so here goers each one
      pointCloudMessage->header.frame_id = depth->header.frame_id;
      pointCloudMessage->header.stamp.sec = depth->header.stamp.sec;
      pointCloudMessage->header.stamp.nsec = depth->header.stamp.nsec;
      pointCloudMessage->header.seq = depth->header.seq;

      pub_pc.publish(pointCloudMessage);
      ROS_INFO_STREAM("Point cloud published:" << pointCloudMessage->data.size());
    }

    //Will now call detect client
    openpose_parser::LoadHands loadHands;
    loadHands.request.header.seq = depth->header.seq;
    loadHands.request.header.frame_id = depth->header.frame_id;
    loadHands.request.header.stamp.sec = depth->header.stamp.sec;
    loadHands.request.header.stamp.nsec = depth->header.stamp.nsec;

    int x_right = 0;
    int y_right = 0;

    int x = 0;
    int y = 0;

    if (client_.call(loadHands))
    {
      ROS_INFO("Received responce from openpose_parser, example of using left");

      //Example of using left hand (and using only one pose for now
      // 1. Let's check sequence number for sanity
      // 2. Also, shoudl check we have some poses returned for left hand
      if (loadHands.response.left.header.seq == loadHands.request.header.seq)
      {
        ROS_INFO_STREAM("Size of data left:" << loadHands.response.left.poses.size() << " right:" << loadHands.response.right.poses.size());
        if (loadHands.response.left.poses.size() > 0 || loadHands.response.right.poses.size() > 0)
        {
          geometry_msgs::Pose right;
          right = loadHands.response.right.poses.at(0);
          x_right = static_cast<int>(right.position.x);
          y_right = static_cast<int>(right.position.y);
          ROS_INFO_STREAM("recieved for right  : [" << x_right << "," << y_right << "]");

          geometry_msgs::Pose left;
          left = loadHands.response.left.poses.at(0);
          x = static_cast<int>(left.position.x);
          y = static_cast<int>(left.position.y);
          // ROS_INFO_STREAM("recieved :" << x <<"," << y);
          ROS_INFO_STREAM("recieved for left  : [" << x << "," << y << "]");
        }
        else
        {
          ROS_WARN("Did not recive any point on left hand");
        }

        //Let's write the info to file
        // logger_.write(loadHands.request.header, loadHands.response.left, loadHands.response.right);
      }
      else
      {
        ROS_WARN("Sequence number invalid in responce, ignoring");
      }
    }
    else
    {
      ROS_WARN("No responce from openpose_parser, loading middle of image as marker");

      //An example of getting a single poitn in image
      //image ----->  x (col) - width
      //      |
      //      v
      //    y (row) - height
      x = ptrDepth->image.cols / 2;
      y = ptrDepth->image.rows / 2;
    }

    pcl::PointXYZ pt_right = pixelLocationTo3d(ptrDepth, x_right, y_right, cameraInfo);

    pcl::PointXYZ pt = pixelLocationTo3d(ptrDepth, x, y, cameraInfo);

    float min_dist_right = 1000; // large number to start from
    float min_dist = 1000;

    //closest distance for RIGHT hand to the table
    pcl::PointXYZRGB closestPt_right;
    //Let's find closest point here from the table to pt

    for (unsigned int i = 0; i < pointCloudTable_.size(); i++)
    {
      float d = pcl::squaredEuclideanDistance(pt_right, pointCloudTable_.at(i));
      if (d < min_dist_right)
      {
        min_dist_right = d;
        closestPt_right = pointCloudTable_.at(i);
      }
    }
    ROS_INFO_STREAM("closest D right :" << min_dist_right << "pt [x,y,z]=[" << closestPt_right.x << "," << closestPt_right.y << "," << closestPt_right.z << "]");

    //closest distance for LEFT hand to the table
    pcl::PointXYZRGB closestPt;
    //Let's find closest point here from the table to pt
    for (unsigned int i = 0; i < pointCloudTable_.size(); i++)
    {
      float d = pcl::squaredEuclideanDistance(pt, pointCloudTable_.at(i));
      if (d < min_dist)
      {
        min_dist = d;
        closestPt = pointCloudTable_.at(i);
      }
    }

    ROS_INFO_STREAM("closest D left :" << min_dist << "pt [x,y,z]=[" << closestPt.x << "," << closestPt.y << "," << closestPt.z << "]");

    //Let's write the info to file
    logger_.write(loadHands.request.header, loadHands.response.left, loadHands.response.right,min_dist, min_dist_right);

    //And publishing it as a marker
    pointToMarker(pt_right, depth->header);
    pointToMarker(pt, depth->header);

    //Going to show same point on RGB colored
    //cv::Vec3b & color = ptrImage->image.at<cv::Vec3b>(row,col);
    // cv::circle(ptrImage->image, cv::Point(x_right, y_right), 10, cv::Scalar(255, 177, 0), 2);
    cv::circle(ptrImage->image, cv::Point(x, y), 10, cv::Scalar(255, 177, 0), 2);
    cv::circle(ptrImage->image, cv::Point(x_right, y_right), 10, cv::Scalar(255, 177, 0), 2);

    sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(ptrImage->header, ptrImage->encoding, ptrImage->image).toImageMsg();

    pub_image.publish(msg_img);
  }
}

void Camera_Nbg::convertDepthToPointCloud(cv_bridge::CvImageConstPtr cvImgPtr, pclPoint::Ptr &cloud,
                                          const sensor_msgs::CameraInfoConstPtr &cameraInfo)
{
  const float *Di;

  float fx = cameraInfo->K.at(0);
  float fy = cameraInfo->K.at(4);
  float cx = cameraInfo->K.at(2);
  float cy = cameraInfo->K.at(5);

  cloud->clear();
  cloud->reserve(static_cast<unsigned long int>(cvImgPtr->image.rows * cvImgPtr->image.cols));
  unsigned int ct = 0;

  for (int r = 0; r < cvImgPtr->image.rows; r++)
  { //row
    Di = cvImgPtr->image.ptr<float>(r);

    for (int c = 0; c < cvImgPtr->image.cols; c++)
    { //column
      pcl::PointXYZ pt;
      float d = Di[c] * 1e-3; //need to convert to [m] here
      pt.x = ((c - cx) * d) / fx;
      pt.y = ((r - cy) * d) / fy;
      pt.z = d;

      if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z))
      {
        cloud->points.push_back(pt);
      }
    }
  }

  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = false;
}

pcl::PointXYZ
Camera_Nbg::pixelLocationTo3d(cv_bridge::CvImageConstPtr cvImgPtr, int x, int y,
                              const sensor_msgs::CameraInfoConstPtr &cameraInfo)
{

  float fx = cameraInfo->K.at(0);
  float fy = cameraInfo->K.at(4);
  float cx = cameraInfo->K.at(2);
  float cy = cameraInfo->K.at(5);

  //to get a single pixel value
  float d = cvImgPtr->image.at<float>(y, x);
  d *= 1e-3;
  pcl::PointXYZ pt;
  pt.x = ((x - cx) * d) / fx;
  pt.y = ((y - cy) * d) / fy;
  pt.z = d;

  const float *Di;
  Di = cvImgPtr->image.ptr<float>(y);
  float di = Di[x] * 1e-3; //need to convert to [m] here
  std::cout << "pt [x,y]=[" << x << "," << y << "] d=" << d << " di=" << di << std::endl;

  return pt;
}

void Camera_Nbg::pointToMarker(pcl::PointXYZ pt, std_msgs::Header header)
{

  //! Here is an example of publishing a marker (in a marker array)
  //!
  //!
  int marker_counter = 0;
  visualization_msgs::Marker marker;

  //We need to set the frame
  // Set the frame ID and time stamp.
  marker.header.frame_id = header.frame_id;
  marker.header.seq = header.seq;
  marker.header.stamp.sec = header.stamp.sec;
  marker.header.stamp.nsec = header.stamp.nsec;

  //We set lifetime (it will dissapear in this many seconds)
  marker.lifetime = ros::Duration(2.0);
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "pose";
  marker.id = marker_counter++;

  // The marker type, we use a cylinder in this example
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
  marker.action = visualization_msgs::Marker::ADD;

  //As an example, we are setting it
  marker.pose.position.x = pt.x;
  marker.pose.position.y = pt.y;
  marker.pose.position.z = pt.z;

  //Orientation, can we orientate it?
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  //Alpha is stransparency (50% transparent)
  marker.color.a = 0.5f;

  //Colour is r,g,b where each channel of colour is 0-1. Bellow will make it orange
  marker.color.r = 1.0;
  marker.color.g = static_cast<float>(177.0 / 255.0);
  marker.color.b = 0.0;

  //We push the marker back on our array of markers
  marker_array_.markers.push_back(marker);

  //We publish the marker array
  viz_pub_.publish(marker_array_);

  //Clear the markers for the next iteration of the code
  marker_array_.markers.clear();
}

bool Camera_Nbg::readFile(const std::string &filePath, const std::string &pointCloudFrameId)
{
  if (filePath.find(".ply") != std::string::npos)
  {
    // Load .ply file.
    //    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    if (loadPLYFile(filePath, pointCloudTable_) != 0)
      return false;

    // Define PointCloud2 message.
    toROSMsg(pointCloudTable_, *pointCloudMessage_);
  }
  else
  {
    ROS_ERROR_STREAM("Data format not supported.");
    return false;
  }

  pointCloudMessage_->header.frame_id = pointCloudFrameId;

  ROS_INFO_STREAM("Loaded point cloud with " << pointCloudMessage_->width << " points.");
  return true;
}

bool Camera_Nbg::readParameters(ros::NodeHandle nh)
{

  bool allParametersRead = true;
  //Ply file information
  if (!nh.getParam("file_path", filePath_))
    allParametersRead = false;
  if (!nh.getParam("topic", pointCloudTopic_))
    allParametersRead = false;
  if (!nh.getParam("frame", pointCloudFrameId_))
    allParametersRead = false;

  double updateRate;
  nh.param("rate", updateRate, 0.0);
  if (updateRate == 0.0)
  {
    isContinousPublishing_ = false;
  }
  else
  {
    isContinousPublishing_ = true;
    updateDuration_.fromSec(1.0 / updateRate);
  }

  if (!allParametersRead)
  {
    ROS_WARN("Could not read all parameters. Typical command-line usage:\n rosrun ply_publisher ply_publisher"
             " _file_path:=path_to_your_point_cloud_file _topic:=/your_topic _frame:=point_cloud_frame"
             " (optional) _rate:=publishing_rate");
    return false;
  }

  return true;
}

void Camera_Nbg::timerCallback(const ros::TimerEvent &timerEvent)
{
  if (!publish())
    ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
}

void Camera_Nbg::initialize(ros::NodeHandle nh)
{
  if (!readFile(filePath_, pointCloudFrameId_))
    ros::requestShutdown();

  if (isContinousPublishing_)
  {
    timer_ = nh.createTimer(updateDuration_, &Camera_Nbg::timerCallback, this);
  }
  else
  {
    ros::Duration(1.0).sleep(); // Need this to get things ready before publishing.
    if (!publish())
      ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
    ros::requestShutdown();
  }
}

bool Camera_Nbg::publish()
{
  pointCloudMessage_->header.stamp = ros::Time::now();
  if (pointCloudPublisher_.getNumSubscribers() > 0u)
  {
    pointCloudPublisher_.publish(pointCloudMessage_);
    ROS_INFO_STREAM("Point cloud published in topic \"" << pointCloudTopic_ << "\".");
  }
  return true;
}
