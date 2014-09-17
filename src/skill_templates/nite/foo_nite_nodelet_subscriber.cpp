/*!
  \file        nite_effect_collection_nodelet.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/10/16

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

cf
https://code.ros.org/svn/ros-pkg/stacks/common_tutorials/trunk/nodelet_tutorial_math/
http://www.embeddedheaven.com/ros-nodelet.htm

\section Parameters
  - \b "resize_scale"
    [double] (default: 1)
    The scale of the interface.
    The original image is 320x240,
    the GUI size scales it by this parameter.

  - \b DISPLAY
    [bool] (default:1)
    if 1, display results in a GUI

\section Subscriptions

\section Publications

 */


#include "kinect_utils/skeleton_utils.h"
#include "image_utils/io.h"
#include "skill_templates/nite/nite_nodelet_subscriber_template.h"

namespace nite_nodelet_ns {

class FooNiteNodeletSubscriber : public NiteNodeletSubscriberTemplate {
public:

  virtual void onInit() {
    NiteNodeletSubscriberTemplate::onInit();
    cv::namedWindow("color");
    cv::namedWindow("depth");
    cv::namedWindow("user_illus");
    printf("Press 's' to save images\n");
  }

  void fn(const cv::Mat3b & color,
          const cv::Mat1f & depth,
          const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list) {
    NODELET_INFO_THROTTLE(1, "FooNiteNodeletSubscriber:fn()");
    image_utils::depth_image_to_vizualisation_color_image
        (depth, depth_illus, image_utils::FULL_RGB_SCALED);
    user_image_to_rgb(user, user_illus, 8);
    skeleton_utils::draw_skeleton_list(user_illus, skeleton_list);

    cv::imshow("color", color);
    cv::imshow("depth", depth);
    cv::imshow("user_illus", user_illus);
    char c = cv::waitKey(10);
    if (c == 's') {
      std::string stamp = StringUtils::timestamp();
      image_utils::write_rgb_depth_user_to_image_file
          (stamp, &color, &depth, &user, &user_illus);
    }
  } // end image_callback();

private:
  cv::Mat3b user_illus;
  cv::Mat3b depth_illus;
}; // end class FooNiteNodeletSubscriber

PLUGINLIB_DECLARE_CLASS(nite_nodelet_ns, FooNiteNodeletSubscriber, nite_nodelet_ns::FooNiteNodeletSubscriber, nodelet::Nodelet);
} // end namespace nite_nodelet_ns
