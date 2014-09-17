/*!
  \file        rgb_and_depth_to_yaml.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/29

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

A simple node that enables the simultaneous writing to file
of a depth and a rgb images.
Before saving them, shows them in a window.
Press 'q' not to save these images.

Output: in the current folder, a file "rgb_depth_{timestamp}.yaml"
containing both depth and rgb images.

\section Parameters
  - \b "~rgb_topic"
        [string] (default: "/kinect_only/rgb")
        Where we will get the rgb images.

  - \b "~depth_topic"
        [string] (default: "/kinect_only/depth")
        Where we will get the depth images.

  - \b "~png_output"
        [bool] (default: false)
        If false, save images as YAML files.
        If true, save them as PNG files: smaller files,
        RGB lossless but depth compressed (float->uchar).

  - \b "~filename_prefix"
        [std::string] (default: "")
        The prefix for the written files.
        If "", the timestamp is used.

\section Subscriptions
  - \b  {rgb_topic}, {depth_topic}
        [sensor_msgs::Image]
        The input topics for rgb and depth images.
        Note both topics are synchronized with a
        message_filters::sync_policies::ApproximateTime.

\section Publications: None.

\example
rosrun vision rgb_and_depth_to_yaml _rgb_topic:=/color_img  _depth_topic:=/depth_img  _png_output:=true  _filename_prefix:=foo
 */

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// AD
#include <src/string/StringUtils.h>
#include <vision_utils/image_utils/io.h>

#define HELP "Press ' ' or 'y' to save the showed images and exit."

bool png_output = false;
std::string _filename_prefix = "";
cv::Mat3b depth_illus;

////////////////////////////////////////////////////////////////////////////////

//! this function is called each time an image is received
void image_callback(const sensor_msgs::ImageConstPtr& rgb_msg,
                    const sensor_msgs::ImageConstPtr& depth_msg) {
  printf("image_callback()\n");
  // get the OpenCV images
  cv_bridge::CvImageConstPtr bridge_rgb_img_ptr, bridge_depth_img_ptr;
  try {
    bridge_rgb_img_ptr = cv_bridge::toCvShare
        (rgb_msg, sensor_msgs::image_encodings::BGR8);
    bridge_depth_img_ptr = cv_bridge::toCvShare(depth_msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  const cv::Mat & rgb_img = bridge_rgb_img_ptr->image;
  const cv::Mat & depth_img = bridge_depth_img_ptr->image;
  image_utils::depth_image_to_vizualisation_color_image
      (depth_img, depth_illus, image_utils::FULL_RGB_STRETCHED);
  // show image
  cv::imshow("rgb_img", rgb_img);
  cv::imshow("depth_illus", depth_illus);
  char c = cv::waitKey(0);
  if ((int) c == 27) {
    printf("Exiting without writing files...\n");
    exit(-1);
  }
  else if (c == 'y' || c == ' ') {
    std::string curr_filename_prefix =
        (_filename_prefix == "" ? StringUtils::timestamp() : _filename_prefix);
    //  printf("_filename_prefix:'%s', curr_filename_prefix:'%s'\n",
    //         _filename_prefix.c_str(), curr_filename_prefix.c_str());
    if (png_output)
      image_utils::write_rgb_and_depth_image_to_image_file
          (curr_filename_prefix, &rgb_img, &depth_img);
    else
      image_utils::write_rgb_and_depth_to_yaml_file
          (curr_filename_prefix, rgb_img, depth_img);
    ros::shutdown();
    return;
  }
  else {
    printf("Images discarded. " HELP "\n");
  }
} // end image_callback();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "rgb_and_depth_to_yaml");
  ros::NodeHandle nh_public, nh_private("~");
  // get params
  std::string depth_topic = "/kinect_only/depth";
  std::string rgb_topic = "/kinect_only/rgb";
  nh_private.param("depth_topic", depth_topic, depth_topic);
  nh_private.param("rgb_topic", rgb_topic, rgb_topic);
  nh_private.param("png_output", png_output, png_output);
  nh_private.param("filename_prefix", _filename_prefix, _filename_prefix);

  // subscribers
  message_filters::Subscriber<sensor_msgs::Image> _rgb_image_subscriber
      (nh_public, rgb_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> _depth_image_subscriber
      (nh_public, depth_topic, 1);
  printf("Subscribing to RGB:'%s', depth:'%s', filename_prefix:'%s'. " HELP "\n",
         _rgb_image_subscriber.getTopic().c_str(),
         _depth_image_subscriber.getTopic().c_str(),
         _filename_prefix.c_str());

  // synchronize depth and rgb callbacks
  // ApproximateTime synchronizer
  typedef message_filters::sync_policies::ApproximateTime
      <sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  //policy.setMaxIntervalDuration (ros::Duration(30.f / 1000)); // max package of 30ms
  message_filters::Synchronizer<MySyncPolicy> sync
      (MySyncPolicy(5), _rgb_image_subscriber, _depth_image_subscriber);
  sync.registerCallback(boost::bind(&image_callback, _1, _2));

  ros::spin();
  return 0;
}
