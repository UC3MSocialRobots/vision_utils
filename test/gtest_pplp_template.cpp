/*!
  \file        gtest_pplp_template.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/20

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
#include "vision_utils/pplp_testing.h"
#include <vision_utils/rosmaster_alive.h>

class Foo2PPL : public vision_utils::PPLPublisherTemplate {
public:
  Foo2PPL() : PPLPublisherTemplate("FOO_START", "FOO_STOP") {
  }
  void create_subscribers_and_publishers() {
    _rgb_sub = _nh_public.subscribe("rgb", 2, &Foo2PPL::rgb_cb, this);
  }

  void shutdown_subscribers_and_publishers() {
    _rgb_sub.shutdown();
  }

  inline void rgb_cb(const sensor_msgs::ImageConstPtr &) {
    printf("rgb_cb()\n");
    people_msgs::People ppl;
    ppl.header.stamp = ros::Time::now();
    ppl.people.clear();
    publish_PPL(ppl);
  }

private:
  ros::Subscriber _rgb_sub;
}; // end class Foo2PPL

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, create) {
  if (!vision_utils::rosmaster_alive()) return;
  Foo2PPL skill;
  ASSERT_FALSE(skill.is_running());
  ASSERT_TRUE(skill.get_ppl_published_nb() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, start_stop) {
  if (!vision_utils::rosmaster_alive()) return;
  Foo2PPL skill;
  vision_utils::start_stop(skill);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, speed_test) {
  if (!vision_utils::rosmaster_alive()) return;
  Foo2PPL skill;
  vision_utils::speed_test(skill, false, 10, .8);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest_pplp_template");
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
