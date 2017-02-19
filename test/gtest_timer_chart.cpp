/*!
  \file        test_timer_chart.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/14

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

Some tests for the TimerChart class.

 */
bool display = false;
// put the line "#define CHART_TIMER_ON" in your code to activate the TimerChart
#define CHART_TIMER_ON
#include "vision_utils/rand_gaussian.h"
#include "vision_utils/timer_chart.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <unistd.h>

inline void usleep_gauss(const double avg_us) {
  usleep(std::max(1., vision_utils::rand_gaussian() * 3E3 + avg_us));
}

TEST(TestSuite, test1) {
  TIMER_CREATE(timer);
  for (int i = 0; i < 20; ++i) {
    printf("i:%i\n", i);

    usleep_gauss(50E3);
    TIMER_PRINT_RESET(timer, "foo");
    usleep_gauss(10E3);
    TIMER_PRINT_RESET(timer, "bar");
    usleep_gauss(20E3);
    TIMER_PRINT_RESET(timer, "zim");
    usleep_gauss(50E3);
    TIMER_PRINT_RESET(timer, "zam");
if (display) {
    TIMER_DISPLAY_CHART(timer, 1);
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
    } // end if display
  } // end while (true)
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

