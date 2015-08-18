/*!
  \file        nite_subscriber_template.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/20

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

\todo Description of the file

\section Parameters
  - \b "~rgb_topic", "~depth_topic", "~user_topic", "~skeleton_topic"
        [std::string, default "rgb", "depth", "user", "skeletons"]
        The name of the topic where the RGB image, depth image,
        user image, skeleton message streams can be obtained.

\section Subscriptions
  - \b {start_topic}, {stop_topic}
        [std_msgs::Int16]
        \see NanoSkill.

  - \b {rgb_topic}, {depth_topic}, {user_topic}, {skeleton_topic}
        [sensor_msgs::Image]
        The RGB image, depth image, user image, skeleton message streams

\section Publications
  None.
 */
#ifndef NITE_RECEIVER_TEMPLATE_H
#define NITE_RECEIVER_TEMPLATE_H

#define QUEUE_SIZE 5
#define USE_EXACT_TIME // comment to use approx time

// ROS
#ifdef USE_EXACT_TIME
#include <message_filters/sync_policies/exact_time.h>
#else // not USE_EXACT_TIME
#include <message_filters/sync_policies/approximate_time.h>
#endif
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
// AD
#include <kinect/NiteSkeletonList.h>
#include "kinect/kinect_openni_utils.h"
#include "time/timer.h"

class NiteSubscriberTemplate {
public:
  typedef image_transport::SubscriberFilter ImageFilterSub;
  typedef message_filters::Subscriber<kinect::NiteSkeletonList> SkeletonListSub;

  //////////////////////////////////////////////////////////////////////////////

  NiteSubscriberTemplate()
    : _nh_private("~"),
      _it(_nh_public),
      _color_image_sub(NULL),
      _depth_image_sub(NULL),
      _user_image_sub(NULL),
      _skeleton_list_sub(NULL),
      _sync(NULL){
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void init() {
    ROS_INFO("NiteSubscriberTemplate::int()");
    // cf http://www.ros.org/wiki/message_filters#ApproximateTime_Policy
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
    std::string _skeleton_topic = "skeletons";
    _nh_private.param("skeleton_topic", _skeleton_topic, _skeleton_topic);
    _resolved_skeleton_topic = _nh_public.resolveName(_skeleton_topic);

    _color_image_sub = new ImageFilterSub(_it, _resolved_rgb_topic, 1);
    _depth_image_sub = new ImageFilterSub(_it, _resolved_depth_topic, 1);
    _user_image_sub =  new ImageFilterSub(_it, _resolved_user_topic, 1);
    _skeleton_list_sub = new SkeletonListSub(_nh_public, _resolved_skeleton_topic, 1);
    ROS_INFO("_color_image_sub:'%s', "
             "_depth_image_sub:'%s' "
             "_user_image_sub:'%s' "
             "_skeleton_sub:'%s'",
             _color_image_sub->getTopic().c_str(),
             _depth_image_sub->getTopic().c_str(),
             _user_image_sub->getTopic().c_str(),
             _skeleton_list_sub->getTopic().c_str());

    // ApproximateTime synchronizer
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    //policy.setMaxIntervalDuration (ros::Duration(30.f / 1000)); // max package of 30ms
    _sync = new message_filters::Synchronizer<MySyncPolicy>
            (MySyncPolicy(QUEUE_SIZE),
             * _color_image_sub,
             *_depth_image_sub,
             *_user_image_sub,
             *_skeleton_list_sub);

    _sync->registerCallback
        ( boost::bind(&NiteSubscriberTemplate::image_callback, this, _1, _2, _3, _4));
  } // end init();

  //////////////////////////////////////////////////////////////////////////////

  virtual void unsubscribe() {
    if (_color_image_sub == NULL)
      return;
    // delete subs
    _color_image_sub->unsubscribe();
    _depth_image_sub->unsubscribe();
    _user_image_sub->unsubscribe();
    _skeleton_list_sub->unsubscribe();
    delete _color_image_sub;
    delete _depth_image_sub;
    delete _user_image_sub;
    delete _skeleton_list_sub;
    delete _sync;
    _color_image_sub = NULL;
    _depth_image_sub = NULL;
    _user_image_sub = NULL;
    _skeleton_list_sub = NULL;
    _sync = NULL;
  } // end unsubscribe();

  //////////////////////////////////////////////////////////////////////////////

  //! dtor
  ~NiteSubscriberTemplate() {
    unsubscribe();
  } // end dtor

  //////////////////////////////////////////////////////////////////////////////

  virtual void fn(const cv::Mat3b & color,
                  const cv::Mat1f & depth,
                  const cv::Mat1b & user,
                  const kinect::NiteSkeletonList & skeleton_list) = 0;

protected:

  //////////////////////////////////////////////////////////////////////////////

  void image_callback(const sensor_msgs::ImageConstPtr& color_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg,
                      const sensor_msgs::ImageConstPtr& user_msg,
                      const kinect::NiteSkeletonListConstPtr& skeleton_list) {
    //ROS_INFO_THROTTLE(1, "NiteSubscriberTemplate::image_callback()");
    // printf("NiteSubscriberTemplate::image_callback()\n");

    _last_header = color_msg->header;
    kinect_openni_utils::get_kinect_serial_number_and_read_camera_model_files_if_needed
        (_nh_public, rgb_camera_model, depth_camera_model);

    try {
      //_bridge_img_color = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
      _bridge_img_color = cv_bridge::toCvShare(color_msg, sensor_msgs::image_encodings::BGR8);
      //    if (_bridge_img_color->encoding == sensor_msgs::image_encodings::RGB8)
      //      cv::cvtColor(_bridge_img_color->image, _bridge_img_color->image, CV_RGB2BGR);
      _bridge_img_depth = cv_bridge::toCvShare(depth_msg
                                               , sensor_msgs::image_encodings::TYPE_32FC1
                                               );
      _bridge_img_user = cv_bridge::toCvShare(user_msg
                                              // , sensor_msgs::image_encodings::MONO8
                                              );
      //  ROS_INFO_THROTTLE(1, "color_msg->encoding:'%s' -> _bridge_img_color->encoding:'%s'",
      //                        color_msg->encoding.c_str(), _bridge_img_color->encoding.c_str());
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Timer timer;
    fn(_bridge_img_color->image,
       _bridge_img_depth->image,
       _bridge_img_user->image,
       *skeleton_list);
    ROS_INFO_THROTTLE(1, "NiteSubscriberTemplate:time for effect fn: %g ms",
                      timer.getTimeMilliseconds());
    // printf("end of NiteSubscriberTemplate::image_callback()\n");
  } // end image_callback();

  //////////////////////////////////////////////////////////////////////////////

  ros::NodeHandle _nh_public, _nh_private;
  cv_bridge::CvImageConstPtr _bridge_img_depth, _bridge_img_user, _bridge_img_color;

  // cf http://www.ros.org/wiki/message_filters#ApproximateTime_Policy
  //! where the camera frames are obtained
  std::string _resolved_rgb_topic, _resolved_depth_topic,
  _resolved_user_topic, _resolved_skeleton_topic;

  image_transport::ImageTransport _it;
  ImageFilterSub *_color_image_sub;
  ImageFilterSub *_depth_image_sub;
  ImageFilterSub *_user_image_sub;
  SkeletonListSub *_skeleton_list_sub;
  std_msgs::Header _last_header; //!< the header of the last received frame
#ifdef USE_EXACT_TIME
  typedef message_filters::sync_policies::ExactTime
  <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, kinect::NiteSkeletonList>
  MySyncPolicy;
#else // not USE_EXACT_TIME
  typedef message_filters::sync_policies::ApproximateTime
  <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, kinect::NiteSkeletonList>
  MySyncPolicy;
#endif
  message_filters::Synchronizer<MySyncPolicy>* _sync;

  image_geometry::PinholeCameraModel rgb_camera_model;
  image_geometry::PinholeCameraModel depth_camera_model;
}; // end class NiteSubscriberTemplate

#endif // NITE_RECEIVER_TEMPLATE_H
