/*!
  \file        rgb_depth_pplp_template.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/21

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

The extension of PPLPublisherTemplate
that subscribes to a stream of RGB and depth data.

\section Parameters
  - \b "~rgb_topic", "~depth_topic"
        [std::string, default "rgb", "depth"]
        The name of the topic where the RGB image stream can be obtained.

\section Subscriptions
  - \b {start_topic}, {stop_topic}
        [std_msgs::Int16]
        \see PPLPublisherTemplate.

  - \b {rgb_topic}, {depth_topic}
        [sensor_msgs::Image]
        The depth and RGB streams.

\section Publications
  - \b "~ppl"
        [people_msgs::People]
        The detected users, \see PPLPublisherTemplate.
 */
#ifndef RGB_DEPTH_TEMPLATE2PPL_H
#define RGB_DEPTH_TEMPLATE2PPL_H

// utils
#include "vision_utils/timer.h"
// people_msgs
#include "vision_utils/pplp_template.h"
// ROS
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

namespace vision_utils {

class RgbDepthPPLPublisherTemplate : public PPLPublisherTemplate {
public:
  typedef sensor_msgs::Image Image;
  // use of message filters - inspiration available at
  // http://answers.ros.org/question/9705/synchronizer-and-image_transportsubscriber/
  typedef image_transport::SubscriberFilter ImgSub;
  static const unsigned int QUEUE_SIZE = 10;

  RgbDepthPPLPublisherTemplate(const std::string & start_topic,
                       const std::string & stop_topic)
    : PPLPublisherTemplate(start_topic, stop_topic),
      _it(_nh_public)
  {
    // get the image channel
    std::string _rgb_topic = "rgb";
    _nh_private.param("rgb_topic", _rgb_topic, _rgb_topic);
    _resolved_rgb_topic = _nh_public.resolveName(_rgb_topic);
    std::string _depth_topic = "depth";
    _nh_private.param("depth_topic", _depth_topic, _depth_topic);
    _resolved_depth_topic = _nh_public.resolveName(_depth_topic);
    _nh_private.param("display", _display, false);
  }

  //////////////////////////////////////////////////////////////////////////////

  ~RgbDepthPPLPublisherTemplate() {
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! should be extended for any kind of display using cv::imshow(), etc.
      Should not contain a cv::waitKey(). */
  virtual void display(const cv::Mat3b & rgb, const cv::Mat1f & depth) {}

  //////////////////////////////////////////////////////////////////////////////

  inline std::string get_rgb_topic()   const { return _resolved_rgb_topic; }
  inline std::string get_depth_topic() const { return _resolved_depth_topic; }

protected:

  /*! where work is done. Should end up with
      publish_PPL();
      if (_display) display(rgb, depth);
  */
  virtual void process_rgb_depth(const cv::Mat3b & rgb,
                                 const cv::Mat1f & depth) {}

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std_msgs::Header _images_header; //!< the header of the last received frame

private:

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    // subscribe to the image message
    _rgb_sub = new ImgSub(_it, _resolved_rgb_topic, 1);
    _depth_sub = new ImgSub(_it, _resolved_depth_topic, 1);

    // ApproximateTime synchronizer
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    //policy.setMaxIntervalDuration (ros::Duration(30.f / 1000)); // max package of 30ms
    _sync =  new message_filters::Synchronizer<MySyncPolicy>
             (MySyncPolicy(QUEUE_SIZE), *_rgb_sub, *_depth_sub);
    _sync->registerCallback(boost::bind(&RgbDepthPPLPublisherTemplate::rgb_depth_cb, this, _1, _2));

    // check publishers
    ::vision_utils::Timer timer;
    unsigned int npubs_rgb = 0, npubs_depth = 0;
    while(timer.getTimeSeconds() < 1 && (!npubs_rgb || !npubs_depth)) {
      npubs_rgb = _rgb_sub->getNumPublishers();
      npubs_depth = _depth_sub->getNumPublishers();
      usleep(100 * 1000);
    }
    if (npubs_rgb && npubs_depth)
      printf("RgbDepthPPLPublisherTemplate: found rgb and depth publishers :)\n");
    if (!npubs_rgb)
      printf("RgbDepthPPLPublisherTemplate: no rgb publisher at '%s'\n",
               get_rgb_topic().c_str());
    if (!npubs_depth)
      printf("RgbDepthPPLPublisherTemplate: no depth publisher at '%s'\n",
               get_depth_topic().c_str());
  } // end create_subscribers_and_publishers();

  //////////////////////////////////////////////////////////////////////////////

  void rgb_depth_cb(const Image::ConstPtr& rgb_msg,
                    const Image::ConstPtr& depth_msg) {
    //printf_ONCE("RgbDepthPPLPublisherTemplate::rgb_depth_cb()");
    // DEBUG_PRINT("rgb_depth_cb()\n");
    try {
      _depth_bridge = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
      _rgb_bridge = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
      _images_header = rgb_msg->header;
    } catch (cv_bridge::Exception& e) {
      printf("cv_bridge exception: %s\n", e.what());
      return;
    }
    process_rgb_depth(_rgb_bridge->image, _depth_bridge->image);
  } // end rgb_depth_cb();

  ////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    _rgb_sub->unsubscribe();
    _depth_sub->unsubscribe();
    delete _rgb_sub;
    delete _depth_sub;
    // delete _sync; // also delete the subscribers it includes
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

#ifdef USE_EXACT_TIME
  typedef message_filters::sync_policies::ExactTime<Image, Image> MySyncPolicy;
#else // not USE_EXACT_T*IME
  typedef message_filters::sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
#endif
  image_transport::ImageTransport _it;
  ImgSub *_rgb_sub, *_depth_sub;
  message_filters::Synchronizer<MySyncPolicy>* _sync;

  //! where the camera frames are obtained
  std::string _resolved_rgb_topic;
  std::string _resolved_depth_topic;

  cv_bridge::CvImageConstPtr _rgb_bridge, _depth_bridge;
protected:
  bool _display;
}; // end class RgdDepthPPLPublisherTemplate

} // end namespace vision_utils

#endif // RGB_DEPTH_TEMPLATE2PPL_H
