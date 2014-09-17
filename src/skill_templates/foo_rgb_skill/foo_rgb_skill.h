/*!
  \file        foo_rgb_skill.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/13

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

A sample RgbSkill.
It subscribes to the image stream and inverts it.

\section Parameters
  - \b "~rgb_topic"
        [std::string, default "rgb"]
        The name of the topic where the RGB image stream can be obtained.

\section Subscriptions
  - \b "FOO_RGB_SKILL_START", "FOO_RGB_SKILL_STOP"
        [std_msgs::Int16]
        \see NanoSkill.

  - \b {rgb_topic}
        [sensor_msgs::Image]
        The RGB streams.

To run it,
T1: roscore
T2: rosrun usb_cam usb_cam_node _pixel_format:=yuyv
T3: rosrun vision_utils launcher_foo_rgb_skill _rgb_topic:=rosrun usb_cam usb_cam_node _pixel_format:=yuyv
T4: rostopic pub /FOO_RGB_SKILL_START std_msgs/Int16 {}

 */
#ifndef FOO_RGB_SKILL_H
#define FOO_RGB_SKILL_H

#include <skill_templates/rgb_skill.h>

class FooRgbSkill : public RgbSkill {
public:
  FooRgbSkill() : RgbSkill("FOO_RGB_SKILL_START", "FOO_RGB_SKILL_STOP"){
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! is called when the skill is activated.
      Should create any ROS publisher/subscriber that is needed.
      The subscribers needed for the activation/desactivation events
      are automatically handled.
  */
  virtual void create_subscribers_and_publishers() {
    printf("FooRgbSkill::create_subscribers_and_publishers()\n");
    // apart from the RGB stream, nothing to subscribe to
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! is called when the skill is desactivated.
      Should shutdown all subs/pubs created in create_subscribers_and_publishers().
      The subscribers needed for the activation/desactivation events
      are automatically handled.
 */
  virtual void shutdown_subscribers_and_publishers() {
    printf("FooRgbSkill::shutdown_subscribers_and_publishers()\n");
    // apart from the RGB stream, nothing to unsubscribe to
  }

  //////////////////////////////////////////////////////////////////////////////

  //! where the real image processing work is done.
  virtual void process_rgb(const cv::Mat3b & rgb) {
    printf("FooRgbSkill::process_rgb()\n");
    frame_out = CV_RGB(255,255,255) - rgb; // invert colors
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! should be extended for any kind of display using cv::imshow(), etc.
      Is called if the param _display:=true .
      Should contain a cv::waitKey(). */
  virtual void display(const cv::Mat3b & rgb) {
    printf("FooRgbSkill::display()\n");
    cv::imshow("rgb", rgb);
    cv::imshow("frame_out", frame_out);
    cv::waitKey(5);
  }

protected:
  cv::Mat3b frame_out;
}; // end class FooRgbSkill

#endif // FOO_RGB_SKILL_H
