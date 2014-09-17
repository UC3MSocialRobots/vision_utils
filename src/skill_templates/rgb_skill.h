/*!
  \file        rgb_skill.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/15

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

A lightweight template for subscribing to a RGB stream.

\section Parameters
  - \b "~rgb_topic"
        [std::string, default "rgb"]
        The name of the topic where the RGB image stream can be obtained.

  - \b "~display"
        [bool, default false]
        True to call the display function upon receiving the images.

\section Subscriptions
  - \b {start_topic}, {stop_topic}
        [std_msgs::Int16]
        \see NanoSkill.

  - \b {rgb_topic}
        [sensor_msgs::Image]
        The RGB streams.

\section Publications
  None.
 */
#ifndef RGB_SKILL_H
#define RGB_SKILL_H

// ROS
#include <skill_templates/nano_skill.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <kinect_utils/kinect_openni_utils.h>
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   ROS_WARN(__VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

class RgbSkill : public NanoSkill {
public:
  typedef sensor_msgs::Image Image;
  static const unsigned int QUEUE_SIZE = 3;

  RgbSkill(const std::string & start_topic,
           const std::string & stop_topic)
    : NanoSkill(start_topic, stop_topic), _it(_nh_public)
  {
    // get the image channel
    std::string _rgb_topic = "rgb";
    _nh_private.param("rgb_topic", _rgb_topic, _rgb_topic);
    _resolved_rgb_topic = _nh_public.resolveName(_rgb_topic);
    _display = false;
  }

  //////////////////////////////////////////////////////////////////////////////

  ~RgbSkill() {
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void start() {
    if (_status == RUNNING) {
      printf("RgbSkill already running!\n");
      return;
    }
    _status = RUNNING;
    // check display param
    _nh_private.param("display", _display, _display);
    _nh_public.param("is_camera_tilted", _is_camera_tilted, false);

    // subscribe to the image message
    _rgb_sub = _it.subscribe(_resolved_rgb_topic, QUEUE_SIZE, &RgbSkill::rgb_cb, this);
    create_subscribers_and_publishers();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void stop() {
    if (_status == STOPPED) {
      printf("RgbSkill already stopped!\n");
      return;
    }
    _status = STOPPED;
    _rgb_sub.shutdown();
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
  virtual void display(const cv::Mat3b & rgb) {
    cv::imshow("rgb", rgb);
    cv::waitKey(5);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline std::string get_rgb_topic()   const { return _resolved_rgb_topic; }

protected:

  /*! where the real image processing work is done.
    children classes must extend this function */
  virtual void process_rgb(const cv::Mat3b & rgb) = 0;

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  bool _display; //!< true to call display
  std_msgs::Header _images_header; //!< the header of the last received frame
private:

  //////////////////////////////////////////////////////////////////////////////

  void rgb_cb(const Image::ConstPtr& rgb_msg) {
    // DEBUG_PRINT("rgb_cb()\n");
    try {
      _rgb_bridge = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
      _images_header = rgb_msg->header;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    const cv::Mat* rgb_ptr = &(_rgb_bridge->image);
    if (_is_camera_tilted) {
      cv::flip(_rgb_bridge->image, _rgb_flip, 0); // x axis
      cv::transpose( _rgb_flip, _rgb_rotated );
      rgb_ptr = &(_rgb_rotated);
    }
    process_rgb(*rgb_ptr);
    if (_display)
      display(*rgb_ptr);
  } // end rgb_cb();

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  // //// images stuff
  image_transport::ImageTransport _it;
  image_transport::Subscriber _rgb_sub;
  cv::Mat3b _rgb_flip, _rgb_rotated;
  //! where the camera frames are obtained
  std::string _resolved_rgb_topic;
  cv_bridge::CvImageConstPtr _rgb_bridge;
  bool _is_camera_tilted;
}; // end class RgbSkill

#endif // RGB_SKILL_H
