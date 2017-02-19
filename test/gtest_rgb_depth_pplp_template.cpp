/*!
  \file        gtest_rgb_depth_pplp_template.cpp
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

Tests for PPLPublisherTemplate

 */
#include "vision_utils/rgb_depth_pplp_template.h"
#include "vision_utils/pplp_testing.h"
#include <vision_utils/rosmaster_alive.h>

class RgbDepthFoo2PPL : public RgbDepthPPLPublisherTemplate {
public:
  RgbDepthFoo2PPL() : RgbDepthPPLPublisherTemplate("FOO_START", "FOO_STOP") {
    DEBUG_PRINT("RgbDepthFoo2PPL: started with '%s' and stopped with '%s', "
               "subscribing to '%s', '%s', "
               "publish People results on '%s'\n",
               get_start_stopic().c_str(), get_stop_stopic().c_str(),
               get_rgb_topic().c_str(), get_depth_topic().c_str(),
               get_ppl_topic().c_str());
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  virtual void process_rgb_depth(const cv::Mat3b & rgb,
                                 const cv::Mat1f & depth) {
    // build PPL message
    people_msgs::People ppl;
    ppl.header = _images_header; // reuse the header of the last frame
    _rgb_copy = rgb;
    publish_PPL(ppl);
  }

  //////////////////////////////////////////////////////////////////////////////

  void display(const cv::Mat3b & rgb,
               const cv::Mat1f & depth) {
    vision_utils::depth_image_to_vizualisation_color_image(depth, _depth2viz);
    cv::imshow("rgb", rgb);
    cv::imshow("depth", _depth2viz);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! illus
  cv::Mat3b _rgb_copy, _depth2viz;
}; // end class RgbDepthFoo2PPL

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, create) {
  if (!vision_utils::rosmaster_alive()) return;
  RgbDepthFoo2PPL skill;
  ASSERT_FALSE(skill.is_running());
  ASSERT_TRUE(skill.get_ppl_published_nb() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, start_stop) {
  if (!vision_utils::rosmaster_alive()) return;
  RgbDepthFoo2PPL skill;
  vision_utils::start_stop(skill);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, speed_test) {
  if (!vision_utils::rosmaster_alive()) return;
  RgbDepthFoo2PPL skill;
  vision_utils::speed_test(skill, false, 30, .8);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest_RgbDepthPPLPublisherTemplate");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
