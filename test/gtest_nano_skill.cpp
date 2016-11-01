/*!
  \file        gtest_nano_skill.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/12

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

Some simple tests for the NanoSkill skill template.
 */
#include <gtest/gtest.h>
#include "vision_utils/nano_skill.h"
#include "vision_utils/timer.h"
#include <vision_utils/rosmaster_alive.h>
#define ASSERT_TRUE_TIMEOUT(cond, timeout) { vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

class FooNanoSkill : public vision_utils::NanoSkill {
public:
  FooNanoSkill() : NanoSkill("FOO_NANO_SKILL_START", "FOO_NANO_SKILL_STOP"){
    _ntacto_received = 0;
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void create_subscribers_and_publishers() {
    _tacto_sub = _nh_public.subscribe("TACTO", 1, &FooNanoSkill::tacto_cb, this);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void shutdown_subscribers_and_publishers() {
    _tacto_sub.shutdown();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void tacto_cb(const std_msgs::Int16ConstPtr & /*msg*/) {
    ++_ntacto_received;
  }

  //////////////////////////////////////////////////////////////////////////////

  ros::Subscriber _tacto_sub;
  unsigned int _ntacto_received;
}; // end class FooNanoSkill

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, start_stop) {
  if (!vision_utils::rosmaster_alive()) return;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  // create skill
  FooNanoSkill skill;
  ASSERT_FALSE(skill.is_running());
  // create pubs
  ros::NodeHandle nh_public;
  ros::Publisher start_pub = nh_public.advertise<std_msgs::Int16>
                  (skill.get_start_stopic(), 1),
      stop_pub = nh_public.advertise<std_msgs::Int16>
                 (skill.get_stop_stopic(), 1),
      tacto_pub = nh_public.advertise<std_msgs::Int16>
                  ("TACTO", 1);

  for (unsigned int iter = 1; iter <= 2; ++iter) {
    printf("start_stop(): iter:%i\n", iter);
    // start skill
    start_pub.publish(std_msgs::Int16());
    ASSERT_TRUE_TIMEOUT(skill.is_running(), 1);
    ASSERT_TRUE_TIMEOUT(tacto_pub.getNumSubscribers(), 1);
    ASSERT_TRUE(skill._ntacto_received == iter-1);

    // trigger a tacto
    tacto_pub.publish(std_msgs::Int16());

    // check a new PPL was published
    ASSERT_TRUE(skill.is_running());
    ASSERT_TRUE_TIMEOUT(skill._ntacto_received == iter, 1);

    stop_pub.publish(std_msgs::Int16());
    ASSERT_TRUE_TIMEOUT(!skill.is_running(), 1);
  } // end for iter
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  ros::init(argc, argv, "gtest_FooNanoSkill");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
