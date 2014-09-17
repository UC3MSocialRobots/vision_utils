/*!
  \file        gtest_mini_stage.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/24

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

Some tests for \b MiniStage class
and \b mini_stage_plugins namespace.

 */
// Bring in gtest
#include <gtest/gtest.h>
#include "visu_utils/mini_stage_plugins.h"

TEST(TestSuite, test_reproject_points) {
  MiniStage ms;
  cv::Point2f origin(0, 0);
  cv::Point2f origin_back_to_world = ms.pixel2world(ms.world2pixel(origin));
  double origin_dist = geometry_utils::distance_points(origin, origin_back_to_world);
  ASSERT_NEAR(fabs(origin_dist), 0, 0.1);

  for (unsigned int i = 0; i < 50; ++i) {
    // cv::Point2f test_pt(0, 0);
    cv::Point2f test_pt(drand48() * 5, drand48() * 5);
    cv::Point2f back_to_world = ms.pixel2world(ms.world2pixel(test_pt));
    double dist = geometry_utils::distance_points(test_pt, back_to_world);
    ASSERT_NEAR(fabs(dist), 0, 0.1)
        << "test_pt:" << geometry_utils::printP2(test_pt).c_str()
        << ", to pixel:" << geometry_utils::printP2(ms.world2pixel(test_pt)).c_str()
        << ", back_to_world:" << geometry_utils::printP2(back_to_world).c_str()
        << ", dist:" << dist;
  } // end loop i}
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
