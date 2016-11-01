/*!
  \file        launcher_camera_blocker_skill.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/7/3

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
  - \b "~threshold_value"
        [int, grey value] (default: 128)
        The value of the binary threshold applied on the greyscale frame.
  - \b "~min_black_pixels_ratio"
        [double, percents of frame size] (default: .8)
        When the ratio of black pixels is greater, the camera is considered
        as blocked on this frame.
  - \b "~blocked_consecutive_time_needed"
        [double, seconds] (default: .5)
        If the camera is blocked during a longer time,
        it is considered is blocked and the events are emitted.
  - \b "~events_list"
        [std::string, CSV] (default: "CAMERA_BLOCKED,0,TOCADO,0")
        Comma-separated list of events and their parameters to send when blocked.
  - \b "~events_wait_time"
        [double, seconds] (default: 0)
        Time to wait between the block detection and the sending of the events.
        Setting it to a few seconds allows for instance the camera to focus again.

 */

// std
#include <std_msgs/String.h>
// opencv
#include <opencv2/imgproc/imgproc.hpp>
// vision
#include "vision_utils/rgb_skill.h"
#include "vision_utils/nano_etts_api.h"
#include "vision_utils/timer.h"
#include "vision_utils/string_split.h"

class CameraBlockerSkill : public RgbSkill {
public:
  CameraBlockerSkill() :
    RgbSkill("CAMERA_BLOCKER_SKILL_START", "CAMERA_BLOCKER_SKILL_STOP") {
  }

  //////////////////////////////////////////////////////////////////////////////

  void create_subscribers_and_publishers() {
    // ROS_WARN("create_subscribers_and_publishers()");
    _etts_api.advertise();
    _etts_api.say_text("|en:Block the camera hole with your hand to notify me!"
                       "|es:Tapa la camara con tu mano para notificar me!");

    // get params
    ros::NodeHandle nh_private("~");
    nh_private.param("events_wait_time", events_wait_time, 0.);
    nh_private.param("threshold_value", threshold_value, 128);
    nh_private.param("min_black_pixels_ratio", min_black_pixels_ratio, .8);
    nh_private.param("blocked_consecutive_time_needed",
                     blocked_consecutive_time_needed, .5);
    // get events list
    std::string events_list = "CAMERA_BLOCKED,0,TOCADO,0";
    nh_private.param("events_list", events_list, events_list);
    // split the event list
    std::vector<std::string> events_words_list;
    vision_utils::StringSplit(events_list, ",", &events_words_list);
    if (events_words_list.size() % 2 != 0) {
      ROS_ERROR("The event list '%s' does not respect the format "
                "[event_name,event_param]* ! Cannot use it", events_list.c_str());
      ros::shutdown();
    }
    // parse events list
    for (unsigned int event_idx = 0; event_idx < events_words_list.size() / 2;
         ++event_idx) {
      std::string topic = events_words_list[event_idx * 2];
      int param = vision_utils::cast_from_string<int>(events_words_list[event_idx * 2 + 1]);
      events_pubs.push_back(_nh_public.advertise<std_msgs::Int16>(topic, 1));
      std_msgs::Int16 msg; msg.data = param;
      events_msgs.push_back(msg);
      ROS_INFO("CameraBlockerSkill: adding event '%s':%i",
               events_pubs.back().getTopic().c_str(), param);
    } // end loop event_idx
    // advertise the events
    sleep(1);

    // reset counter of blocked frames
    _was_previous_frame_blocked = false;
    _was_current_block_notified_by_sound = false;

//  if (DISPLAY) {
//    cvNamedWindow("frame");
//    cvNamedWindow("frame_threshold");
//  }
  } // end create_subscribers_and_publishers();

  //////////////////////////////////////////////////////////////////////////////

  void shutdown_subscribers_and_publishers() {
    // ROS_WARN("shutdown_subscribers_and_publishers()");
//    if (DISPLAY) {
//      cvDestroyWindow("frame_threshold");
//      cvDestroyWindow("frame");
//    }
  } // end shutdown_subscribers_and_publishers();

  //////////////////////////////////////////////////////////////////////////////

  virtual void display(const cv::Mat3b & rgb) {
    cv::imshow("rgb", rgb);
    cv::imshow("frame_threshold", frame_bw_threshold);
    int key_code = (char) cv::waitKey(20);
    if (key_code == 27)
      exit(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  void process_rgb(const cv::Mat3b & rgb) {
    ROS_WARN("process_rgb(), time since last:%g ms", _timer.getTimeMilliseconds());
    _timer.reset();

    // threshold the image
    cv::cvtColor(rgb, frame_bw, cv::COLOR_BGR2GRAY);
    cv::threshold(frame_bw, frame_bw_threshold, threshold_value, 255, CV_THRESH_BINARY);

    // compute the ratio of zero pixels
    bool is_current_frame_blocked = false;
    double black_pixels_ratio = 1.f -
        1.f * cv::countNonZero(frame_bw_threshold) / (frame_bw.cols * frame_bw.rows);
    // ROS_WARN("black_pixels_ratio:%g", black_pixels_ratio);

    // determine if the frame is blocked
    if (black_pixels_ratio > min_black_pixels_ratio) { // frame blocked
      is_current_frame_blocked = true;
    }

    if (is_current_frame_blocked) {
      // ROS_WARN("frame blocked for %g seconds", blocked_consecutive_time.getTimeSeconds());

      // convert the threshold image to grey
      cv::threshold(frame_bw_threshold, frame_bw_threshold,
                    0, 128, CV_THRESH_BINARY_INV);
    }
    else {
      // ROS_WARN("frame not blocked");
    }

    // reset timer when needed
    // has to be done before the sound confirmation
    if (// reset if frame not blocked
        !is_current_frame_blocked
        // last frame not blocked = 1st blocked frame -> reset timer
        || (!_was_previous_frame_blocked && is_current_frame_blocked)
        ) {
      // ROS_WARN("Reseting blocked_consecutive_time");
      blocked_consecutive_time.reset();
    }

    // make sound if long enough
    if (!_was_current_block_notified_by_sound
        && is_current_frame_blocked
        && blocked_consecutive_time.getTimeMilliseconds() > blocked_consecutive_time_needed) {
      _etts_api.say_text("|en:NonVerbal::CONFIRMATION");
      _was_current_block_notified_by_sound = true;
    }

    // send events if back to non blocked and notified
    if (_was_previous_frame_blocked
        && !is_current_frame_blocked
        && _was_current_block_notified_by_sound)
    {
      advertize_camera_blocked();
      _was_current_block_notified_by_sound = false;
    }

    // store the block status for next frame
    _was_previous_frame_blocked = is_current_frame_blocked;
  } // end process_rgb()

  //////////////////////////////////////////////////////////////////////////////

private:

  void advertize_camera_blocked() {
    // ROS_WARN("advertize_camera_blocked()");

    // sleep needed time
    if (events_wait_time > 0) {
      // ROS_WARN("Sleeping %g sec before sending events", events_wait_time);
      usleep(events_wait_time * 1E6);
    }

    // send events
    for (unsigned int pub_idx = 0; pub_idx < events_pubs.size(); ++pub_idx)
      events_pubs[pub_idx].publish(events_msgs[pub_idx]);

  } // end advertize_camera_blocked();


  //////////////////////////////////////////////////////////////////////////////

  Timer _timer;
  cv::Mat1b frame_bw;
  //! the thresholded bw image
  cv::Mat1b frame_bw_threshold;
  int threshold_value;
  //! the ratio of black pixels to consider the frame as blocked
  double min_black_pixels_ratio;
  //! the list of event names to send when the camera is blocked
  std::vector<ros::Publisher> events_pubs;
  //! the list of event parameters to send when the camera is blocked
  std::vector<std_msgs::Int16> events_msgs;
  //! true when the previous frame was blocked
  bool _was_previous_frame_blocked;
  //! for how long the camera has been blocked
  Timer blocked_consecutive_time;
  //! the minimum time for a block to trigger the events to be sent
  double blocked_consecutive_time_needed;
  //! true if the current camera block triggered the events to be sent
  bool _was_current_block_notified_by_sound;
  double events_wait_time;
  NanoEttsApi _etts_api;
}; // end class CameraBlockerSkill

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "LAUNCHER_CAMERA_BLOCKER_SKILL");
  CameraBlockerSkill skill;
  ros::spin();
  return 0;
}
