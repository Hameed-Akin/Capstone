#include "ros/ros.h"
#include "camera_nbg.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_nbg");

    Camera_Nbg cam(argc, argv);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
