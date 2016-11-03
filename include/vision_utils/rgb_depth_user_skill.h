/*!
  \file        rgb_depth_user_skill.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/11/17

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

A template skill to subscribe to a synchronized stream of
RGB, depth and user images.

\section Parameters
  - \b "~rgb_topic", "~depth_topic", "~user_topic"
        [std::string, default "rgb", "depth", "user"]
        The name of the topic where the RGB image, depth image,
        user image streams can be obtained.

  - \b "~display"
      [bool, default: false]
      True to display the acquired RGB, depth, user images

\section Subscriptions
  - \b {start_topic}, {stop_topic}
        [std_msgs::Int16]
        \see NanoSkill.

  - \b {rgb_topic}, {depth_topic}, {user_topic}
        [sensor_msgs::Image]
        The RGB image, depth image, user image streams
 */

#ifndef RGB_DEPTH_USER_SKILL_H
#define RGB_DEPTH_USER_SKILL_H

// ROS
#include "vision_utils/nano_skill.h"
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>


//#define USE_EXACT_TIME // comment to use approx time
#ifdef USE_EXACT_TIME
#include <message_filters/sync_policies/exact_time.h>
#else // not USE_EXACT_TIME
#include <message_filters/sync_policies/approximate_time.h>
#endif
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   //printf_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

namespace vision_utils {

class RgbDepthUserSkill : public NanoSkill {
public:

  typedef sensor_msgs::Image Image;
  // use of message filters - inspiration available at
  // http://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/
  typedef image_transport::SubscriberFilter ImgSub;
  static const unsigned int QUEUE_SIZE = 3;

  RgbDepthUserSkill(const std::string & start_topic,
                    const std::string & stop_topic)
    : NanoSkill(start_topic, stop_topic),
      _display(false),
      _it(_nh_public)
  {
    // get image topics
    std::string _rgb_topic = "rgb";
    _nh_private.param("rgb_topic", _rgb_topic, _rgb_topic);
    _resolved_rgb_topic = _nh_public.resolveName(_rgb_topic);
    std::string _depth_topic = "depth";
    _nh_private.param("depth_topic", _depth_topic, _depth_topic);
    _resolved_depth_topic = _nh_public.resolveName(_depth_topic);
    std::string _user_topic = "user";
    _nh_private.param("user_topic", _user_topic, _user_topic);
    _resolved_user_topic = _nh_public.resolveName(_user_topic);
    // get camera model
    read_camera_model_files
        (DEFAULT_KINECT_SERIAL(), _default_depth_camera_model, _default_rgb_camera_model);
  }

  //////////////////////////////////////////////////////////////////////////////

  ~RgbDepthUserSkill() {
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void start() {
    if (_status == RUNNING) {
      printf("RgbDepthUserSkill already running!\n");
      return;
    }
    _status = RUNNING;
    // check display param
    _nh_private.param("display", _display, _display);

    // subscribe to the image message
    _rgb_sub = new ImgSub(_it, _resolved_rgb_topic, 1);
    _depth_sub = new ImgSub(_it, _resolved_depth_topic, 1);
    _user_sub = new ImgSub(_it, _resolved_user_topic, 1);

    // ApproximateTime synchronizer
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    //policy.setMaxIntervalDuration (ros::Duration(30.f / 1000)); // max package of 30ms
    _sync =  new message_filters::Synchronizer<MySyncPolicy>
             (MySyncPolicy(QUEUE_SIZE), *_rgb_sub, *_depth_sub, *_user_sub);
    _sync->registerCallback(boost::bind(&RgbDepthUserSkill::rgb_depth_user_cb, this, _1, _2, _3));

    create_subscribers_and_publishers();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void stop() {
    if (_status == STOPPED) {
      printf("RgbDepthUserSkill already stopped!\n");
      return;
    }
    _status = STOPPED;
    _rgb_sub->unsubscribe();
    _depth_sub->unsubscribe();
    delete _rgb_sub;
    delete _depth_sub;
    // delete _sync; // also delete the subscribers it includes
    shutdown_subscribers_and_publishers();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! inherited from NanoSkill
  // virtual void create_subscribers_and_publishers() = 0;
  // virtual void shutdown_subscribers_and_publishers() = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! should be extended for any kind of display using cv::imshow(), etc.
      Is called if the param _display:=true .
      Should contain a cv::waitKey(). */
  virtual void display(const cv::Mat3b & rgb,
                       const cv::Mat1f & depth,
                       const cv::Mat1b & user) {
    cv::imshow("rgb", rgb);
    cv::waitKey(5);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline std::string get_rgb_topic()   const { return _resolved_rgb_topic; }
  inline std::string get_depth_topic() const { return _resolved_depth_topic; }

  //! conversion functions between image pixels and 3D points
  inline cv::Point3f pixel2world_depth(const cv::Point & p2d) const {
    return pixel2world_depth<cv::Point3d>
        (p2d, _default_depth_camera_model, _depth_bridge->image);
  }
  inline cv::Point3f pixel2world_rgb(const cv::Point & p2d) const {
    float ratio_cols = _depth_bridge->image.cols / _rgb_bridge->image.cols;
    float ratio_rows = _depth_bridge->image.rows / _rgb_bridge->image.rows;
    return pixel2world_depth(cv::Point(p2d.x * ratio_cols,
                                       p2d.y * ratio_rows));
  }

  inline cv::Point2d world2pixel_depth(const cv::Point3d & p3d) const {
    return world2pixel<cv::Point2d>
        (p3d, _default_depth_camera_model);
  }
  inline cv::Point2d world2pixel_rgb(const cv::Point3d & p3d) const {
    cv::Point2d p2d = world2pixel_depth(p3d);
    float ratio_cols = _rgb_bridge->image.cols / _depth_bridge->image.cols;
    float ratio_rows = _rgb_bridge->image.rows / _depth_bridge->image.rows;
    return cv::Point(p2d.x * ratio_cols, p2d.y * ratio_rows);
  }

protected:

  //! where work is done - you should extend this function
  virtual void process_rgb_depth_user(const cv::Mat3b & rgb,
                                      const cv::Mat1f & depth,
                                      const cv::Mat1b & user) = 0;

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std_msgs::Header _images_header; //!< the header of the last received frame
  //! for reprojection
  image_geometry::PinholeCameraModel _default_depth_camera_model,
  _default_rgb_camera_model;

private:

  //////////////////////////////////////////////////////////////////////////////

  void rgb_depth_user_cb(const Image::ConstPtr& rgb_msg,
                         const Image::ConstPtr& depth_msg,
                         const Image::ConstPtr& user_msg) {
    // DEBUG_PRINT("rgb_depth_user_cb()\n");
    try {
      _rgb_bridge = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
      _depth_bridge = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      _user_bridge = cv_bridge::toCvShare(user_msg, sensor_msgs::image_encodings::TYPE_8UC1);
      _images_header = rgb_msg->header;
    } catch (cv_bridge::Exception& e) {
      printf("cv_bridge exception: %s\n", e.what());
      return;
    }
    process_rgb_depth_user(_rgb_bridge->image,
                           _depth_bridge->image,
                           _user_bridge->image);
    if (_display)
      display(_rgb_bridge->image, _depth_bridge->image, _user_bridge->image);
  } // end rgb_depth_user_cb();

  ////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  bool _display; //!< true to call display

  // //// images stuff
#ifdef USE_EXACT_TIME
  typedef message_filters::sync_policies::ExactTime<Image, Image, Image> MySyncPolicy;
#else // not USE_EXACT_TIME
  typedef message_filters::sync_policies::ApproximateTime<Image, Image, Image> MySyncPolicy;
#endif
  image_transport::ImageTransport _it;
  ImgSub *_rgb_sub, *_depth_sub, *_user_sub;
  message_filters::Synchronizer<MySyncPolicy>* _sync;

  //! where the camera frames are obtained
  std::string _resolved_rgb_topic, _resolved_depth_topic, _resolved_user_topic;
  //! bridges to uncompress the images
  cv_bridge::CvImageConstPtr _rgb_bridge, _depth_bridge, _user_bridge;
}; // end class RgbDepthUserSkill

} // end namespace vision_utils

#endif // RGB_DEPTH_USER_SKILL_H
