#ifndef CAMERA_NBG_NODE_H
#define CAMERA_NBG_NODE_H

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

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

//#include <geometry_msgs/Transform.h>
//#include <geometry_msgs/Pose.h>

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "openpose_parser/LoadHands.h"

#include "logger.h"
#include <sensor_msgs/PointCloud2.h>

static const float FOREGROUND_DIST =3.0;    // Distance in m.
typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<Point> pclPoint;

class Camera_Nbg
{
public:

    Camera_Nbg(int argc, char *argv[]);

    ~Camera_Nbg();

    void callback(const sensor_msgs::ImageConstPtr& image,
            const sensor_msgs::ImageConstPtr& depth,
            const sensor_msgs::CameraInfoConstPtr& cameraInfo);

private:
    void
    convertDepthToPointCloud(cv_bridge::CvImageConstPtr cvImgPtr, pclPoint::Ptr &cloud,
                                    const sensor_msgs::CameraInfoConstPtr& cameraInfo);

    void
    pointToMarker(pcl::PointXYZ pt, std_msgs::Header header);

    pcl::PointXYZ
    pixelLocationTo3d(cv_bridge::CvImageConstPtr cvImgPtr, int row, int col,
                                    const sensor_msgs::CameraInfoConstPtr& cameraInfo );

    /*!
     * Read the point cloud from a .ply file.
     * @param filePath the path to the .ply file.
     * @param pointCloudFrameId the id of the frame of the point cloud data.
     * @return true if successful.
     */
    bool readFile(const std::string& filePath, const std::string& pointCloudFrameId);

    /*!
     * Publish the point cloud as a PointCloud2.
     * @return true if successful.
     */
    bool publish();

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters(ros::NodeHandle nh);

    /*!
     * Initializes node.
     */
    void initialize(ros::NodeHandle nh);

    /*!
     * Timer callback function.
     * @param timerEvent the timer event.
     */
    void timerCallback(const ros::TimerEvent& timerEvent);


private:

  image_transport::SubscriberFilter image_mono_sub_;
  image_transport::SubscriberFilter image_depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> * sync_;

  image_transport::Publisher pub_image;
  image_transport::Publisher pub_depth;
  ros::Publisher pub_pc;

  ros::Publisher viz_pub_;//!< Visulisation publisher
  visualization_msgs::MarkerArray marker_array_;//!< Storage for marker array

  ros::ServiceClient client_;//!< Client that we can use to call detect hands

  Logger logger_;


  //! Point cloud message to publish.
  sensor_msgs::PointCloud2::Ptr pointCloudMessage_;

  //! Point cloud publisher.
  ros::Publisher pointCloudPublisher_;

  //! Timer for publishing the point cloud.
  ros::Timer timer_;

  //! Path to the point cloud file.
  std::string filePath_;

  //! Point cloud topic to be published at.
  std::string pointCloudTopic_;

  //! Point cloud frame id.
  std::string pointCloudFrameId_;

  //! If true, continous publishing is used.
  //! If false, point cloud is only published once.
  bool isContinousPublishing_;

  //! Duration between publishing steps.
  ros::Duration updateDuration_;

  ros::NodeHandle nh_;

  pcl::PointCloud<pcl::PointXYZRGB> pointCloudTable_;

};




#endif // CAMERA_NBG_NODE_H
