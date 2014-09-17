/*!
  \file        foo_nite_subscriber.cpp
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

A simple test file for \a NiteSubscriberTemplate.

 */

#include "skill_templates/nite/nite_subscriber_template.h"
#include "image_utils/io.h"
#include "kinect_utils/user_image_to_rgb.h"
#include "kinect_utils/skeleton_utils.h"

class FooNiteSubscriber : public NiteSubscriberTemplate {
public:

  virtual void init() {
    NiteSubscriberTemplate::init();
    cv::namedWindow("color");
    cv::namedWindow("depth_illus");
    cv::namedWindow("user_illus");
    printf("Press 's' to save images\n");
  }

  void fn(const cv::Mat3b & color,
          const cv::Mat1f & depth,
          const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list) {
    ROS_INFO_THROTTLE(1, "FooNiteSubscriber:fn()");
    image_utils::depth_image_to_vizualisation_color_image
        (depth, depth_illus, image_utils::FULL_RGB_SCALED);
    user_image_to_rgb(user, user_illus, 8);
    skeleton_utils::draw_skeleton_list(user_illus, skeleton_list);

    cv::imshow("color", color);
    //cv::imshow("depth", depth);
    cv::imshow("depth_illus", depth_illus);
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
}; // end class FooNiteSubscriber

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "foo_nite_reciever");
  FooNiteSubscriber receiver;
  receiver.init();
  ros::spin();
}
