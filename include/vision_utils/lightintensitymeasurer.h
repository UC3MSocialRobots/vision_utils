#ifndef LIGHTINTENSITYMEASURER_H
#define LIGHTINTENSITYMEASURER_H

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include"ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "screens_msgs/ScreensContrast.h"
#include "screens_msgs/ScreensStatus.h"


using namespace cv;
using namespace std;

class LightIntensityMeasurer
{
public:
    LightIntensityMeasurer();
    static int getMeanIntensity(Mat & image);
    int getContrastLevel(int meanLuminance);
    void publishContrastLevel(int level);
    void checkParams();
    bool checkContrastVariation(int currentContrast);

    void callbackKinectFrame(const sensor_msgs::CompressedImage& frame);
    void callbackScreenStatus(const screens_msgs::ScreensStatus& scr_status);


private:
    ros::NodeHandle nh_;
    ros::Publisher contrast_level_pub_;
    ros::Subscriber kinect_frame_sub_;
    ros::Subscriber screens_status_sub_;

    int max_luminance_;
    int min_luminance_;
    int luminance_levels_;
    int detection_window_;
    int min_detections_;
    int last_contrast_;
    int last_contrast_published_;
    int zero_contrast_threshold_;


};

#endif // LIGHTINTENSITYMEASURER_H
