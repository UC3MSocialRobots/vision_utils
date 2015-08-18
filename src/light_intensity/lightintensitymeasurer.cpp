#include "vision_utils/lightintensitymeasurer.h"



LightIntensityMeasurer::LightIntensityMeasurer() :
    nh_(),
    kinect_frame_sub_(nh_.subscribe("/alz/rgb/compressed",20, &LightIntensityMeasurer::callbackKinectFrame, this)),
    screens_status_sub_(nh_.subscribe("screens_status",20, &LightIntensityMeasurer::callbackScreenStatus, this)),
    contrast_level_pub_(nh_.advertise<screens_msgs::ScreensContrast>("screens_contrast",10))
{
    checkParams();
    detection_window_ = 0;
    last_contrast_ = -1;
    last_contrast_published_ = -1;
}

int LightIntensityMeasurer::getMeanIntensity(Mat & image)
{
    Mat hlsFrame;
    vector<Mat> hlsChannels;

    cvtColor(image, hlsFrame,CV_BGR2HLS);
    split(hlsFrame, hlsChannels);
    Mat lum = hlsChannels.at(1);
    Scalar mean_intensity, std_dev;
    cv::meanStdDev(lum,mean_intensity,std_dev);
    //cout << "Mean " << mean_intensity[0] << " --- StdDev " << std_dev[0] << endl;

    return mean_intensity[0];
}

void LightIntensityMeasurer::callbackKinectFrame(const sensor_msgs::CompressedImage& frame)
{
    Mat decodedFrame = imdecode(frame.data,CV_LOAD_IMAGE_ANYCOLOR);
    int meanLuminance = getMeanIntensity(decodedFrame);
    int contrastLevel = getContrastLevel(meanLuminance);
    // Publish contrast level
   cout << "Mean " << meanLuminance << " --- Contrast Level " << contrastLevel << endl;

    bool publish = checkContrastVariation(contrastLevel);
    if ((publish && contrastLevel != last_contrast_published_)||last_contrast_published_ == -1)
    {
        publishContrastLevel(contrastLevel);
        cout << "Publish contrast change " << last_contrast_published_<< " --> " << contrastLevel << endl;
        last_contrast_published_ = contrastLevel;
    }
}

bool LightIntensityMeasurer::checkContrastVariation(int currentContrast)
{

    if(currentContrast == last_contrast_)
    {
        detection_window_ ++;
        if(detection_window_ == min_detections_)
            return true; // Publish current contrast
    }
    else
    {
        detection_window_ = 0;
        last_contrast_ = currentContrast;
    }
    return false;
}

void LightIntensityMeasurer::publishContrastLevel(int level)
{
    screens_msgs::ScreensContrast scr_contrast;
    scr_contrast.contrast = level;

    contrast_level_pub_.publish(scr_contrast);

}

int LightIntensityMeasurer::getContrastLevel(int meanLuminance)
{
    int contrastLevel = 0;
    if(meanLuminance >= max_luminance_)
        return 15;
    if(meanLuminance <= zero_contrast_threshold_)
        return 0;
    if(meanLuminance <= min_luminance_)
        return 4;

    contrastLevel = (meanLuminance-min_luminance_)/((max_luminance_-min_luminance_)/luminance_levels_);

    if(contrastLevel < 4) contrastLevel = 4;

    return contrastLevel;

}

void LightIntensityMeasurer::callbackScreenStatus(const screens_msgs::ScreensStatus& scr_status)
{
    cout << "Screen status received " << (int)scr_status.status << endl;
    if(scr_status.status == scr_status.UP)
    {
        cout << "Republishing current contrast " << last_contrast_published_ << endl;

        publishContrastLevel(last_contrast_published_);
    }

}

void LightIntensityMeasurer::checkParams()
{
    cout << "--- Reading parameters--- " << endl;
    int luminanceTemp = -1;
    if(!ros::param::getCached("/light_intensity_measurer/max_luminance",luminanceTemp))
    {
        luminanceTemp = 180;
        ros::param::set("/light_intensity_measurer/max_luminance",luminanceTemp);
    }
    max_luminance_ = luminanceTemp;
    cout << "    Maximum luminance: " << max_luminance_ << endl;

    if(!ros::param::getCached("/light_intensity_measurer/min_luminance",luminanceTemp))
    {
        luminanceTemp = 40;
        ros::param::set("/light_intensity_measurer/min_luminance",luminanceTemp);
    }
    min_luminance_ = luminanceTemp;
    cout << "    Minimum luminance: " << min_luminance_ << endl;

    if(!ros::param::getCached("/light_intensity_measurer/levels",luminanceTemp))
    {
        luminanceTemp = 15;
        ros::param::set("/light_intensity_measurer/levels",luminanceTemp);
    }
    luminance_levels_ = luminanceTemp;
    cout << "    Luminance levels: " << luminance_levels_ << endl;


    if(!ros::param::getCached("/light_intensity_measurer/min_detections",luminanceTemp))
    {
        luminanceTemp = 10;
        ros::param::set("/light_intensity_measurer/min_detections",luminanceTemp);
    }
    min_detections_ = luminanceTemp;
    cout << "    Minimum number of detections to publish a contrast change: " << min_detections_ << endl;

        if(!ros::param::getCached("/light_intensity_measurer/zero_contrast_threshold",luminanceTemp))
    {
        luminanceTemp = 10;
        ros::param::set("/light_intensity_measurer/zero_contrast_threshold",luminanceTemp);
    }
    zero_contrast_threshold_ = luminanceTemp;
    cout << "    Minimum intensity to set contrast as zero: " << zero_contrast_threshold_ << endl;
}
