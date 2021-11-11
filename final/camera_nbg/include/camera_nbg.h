#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tf/tf.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

class camera_nbg
{

public:
		camera_nbg(ros::NodeHandle &n);

		virtual ~camera_nbg();

		void callback(const sensor_msgs::ImageConstPtr& image,
                const sensor_msgs::ImageConstPtr& depth,
                const sensor_msgs::CameraInfoConstPtr& cameraInfo);

private:
    sensor_msgs::ImagePtr msg_img;
    sensor_msgs::ImagePtr msg_depth;
};
