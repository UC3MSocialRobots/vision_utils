#include"lightintensitymeasurer.h"
#include "ros/ros.h"
#include "stdio.h"

int main(int argc, char *argv[])
{
    std::cout << "Start" << std::endl;
    ros::init(argc,argv,"light_intensity_measurer");
    std::cout << "ROS initialized" << std::endl;

    LightIntensityMeasurer measurer;

    ros::spin();
    std::cout << "\nClean program exit" << std::endl;

}


//rostopic info /alz/rgb/compressed
//sensor_msgs/CompressedImage
