/*!
 * compressed_rounded_image_transport is an image_transport plugin for float images.
 * \author Arnaud Ramey ( arnaud.a.ramey@gmail.com )
            -- Robotics Lab, University Carlos III of Madrid
 * \date Jan. 2012
This is an executable to subscribe to a depth topic
  and show it in a grayscale version.

\section Parameters
 - \b "~input_topic"
      [string] (default:"depth")
      to set the input topic
 - \b "~window_title"
      [string] (default:{input_topic})
      to set the window title
 - \b "~color_mode"
      [int] (default:vision_utils::FULL_RGB_SCALED)
      to set the color mode
 - \b "~display_images"
      [int] (default:0)
      to display the images
 - \b "~save_images"
      [int] (default:0)
      to save the images

\section Subscriptions
  - \b {input_topic}
        [sensor_msgs/Image]
        The input image.
        Must be a depth image.
  */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
// compressed_rounded
#include "vision_utils/timer.h"
#include "vision_utils/depth_image_to_vizualisation_color_image.h"

//! the uchar image that is displayed
//cv::Mat1b uchar_greyscale_img;
//! the color image that is displayed
cv::Mat3b uchar_rgb_img;
//! the sensor_msgs::Image -> cv::Mat converter
cv_bridge::CvImageConstPtr bridge_img_ptr;
//! the name of the window
std::string window_title;
//! the way to represent the depth in color
vision_utils::DepthViewerColorMode _color_mode;
//! true for saving images
bool display_images = true, save_images = false;
float min_value = 0, max_value = 10;
int img_counter = 1, mouse_x = -1, mouse_y = -1;
vision_utils::Timer last_mouse_move;

////////////////////////////////////////////////////////////////////////////////

//! the callback when a depth image is received: convert it to uchar and show it
void depth_image_callback(const sensor_msgs::ImageConstPtr& received_float_img_msg) {
  // ROS_INFO("depth_image_callback()");
  // conversion to cv::Mat
  // cf
  // http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
  try {
    bridge_img_ptr = cv_bridge::toCvShare(received_float_img_msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  const cv::Mat & received_float_img_ref = bridge_img_ptr->image;

  // vision_utils::Timer timer;
  vision_utils::depth_image_to_vizualisation_color_image
      (received_float_img_ref, uchar_rgb_img, _color_mode, min_value, max_value);
  // timer.printTime("depth_image_to_vizualisation_color_image()");

  if (last_mouse_move.getTimeSeconds() < 2
      && mouse_x >= 0 && mouse_x <= uchar_rgb_img.cols
      && mouse_y >= 0 && mouse_y <= uchar_rgb_img.rows) {
    float dist = received_float_img_ref.at<float>(mouse_y, mouse_x);
    std::ostringstream text; text << std::setprecision(3) << dist << " m";
    cv::rectangle(uchar_rgb_img, cv::Point(mouse_x-2, mouse_y-2),
                  cv::Point(mouse_x+2, mouse_y+2), CV_RGB(255,255,255), -1);
    cv::putText(uchar_rgb_img, text.str(), cv::Point(mouse_x + 10, mouse_y),
                CV_FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 255, 255), 2);
  }

  if (save_images) {
    std::ostringstream filename;
    filename << "depth" << std::setw(6) << std::setfill('0') << img_counter++ << ".jpg";
    cv::imwrite(filename.str(), uchar_rgb_img);
    printf("Saved depth image '%s'\n", filename.str().c_str());
  } // end if (save_images)

  if (display_images) {
    cv::imshow(window_title, uchar_rgb_img);
    int key = (char) cv::waitKey(1);
    if (key == ' ')
      _color_mode = (vision_utils::DepthViewerColorMode)
          ((_color_mode + 1) % vision_utils::DEPTH_VIEWER_COLOR_NMODES);
    else if (key == 27) {// esc
      cv::destroyWindow(window_title);
      cv::waitKey(5);
      display_images = false;
    }
    else if (key == 'q')
      ros::shutdown();
  } // end if (display_images)
} // end depth_image_callback();

////////////////////////////////////////////////////////////////////////////////

void mouse_cb(int, int x, int y, int, void*) {
  mouse_x = x;
  mouse_y = y;
  last_mouse_move.reset();;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_to_rgb_viewer");
  ros::NodeHandle nh_public, nh_private("~");

  // get the input topic
  std::string input_topic;
  nh_private.param<std::string>("input_topic", input_topic, "depth");
  ROS_INFO("input_topic:'%s'", input_topic.c_str());

  // subscribe to it
  image_transport::ImageTransport it(nh_public);
  image_transport::Subscriber sub = it.subscribe(input_topic, 1,
                                                 depth_image_callback);

  // create the window
  int _color_mode_int;
  nh_private.param<std::string>("window_title", window_title,
                                sub.getTopic());
  nh_private.param<bool>("display_images", display_images, true);
  if (display_images) {
    cv::namedWindow(window_title);
    cv::setMouseCallback(window_title, mouse_cb);
  }
  nh_private.param<bool>("save_images", save_images, false);
  save_images = (save_images > 0);

  // determine if a color version is needed
  nh_private.param<int>("color_mode", _color_mode_int, vision_utils::FULL_RGB_SCALED);
  _color_mode = (vision_utils::DepthViewerColorMode)
      (_color_mode_int % vision_utils::DEPTH_VIEWER_COLOR_NMODES);
  double min_value_double = min_value, max_value_double = max_value;
  nh_private.param<double>("min_value", min_value_double, min_value_double);
  nh_private.param<double>("max_value", max_value_double, max_value_double);
  min_value = min_value_double; max_value = max_value_double;

  ROS_INFO("depth_to_rgb_viewer:input topic:'%s', display_images:%i, save_images:%i,"
           "min_value:%g, max_value:%g",
           sub.getTopic().c_str(), display_images, save_images,
           min_value, max_value);
  ros::spin();
  return 0;
} // end main()
