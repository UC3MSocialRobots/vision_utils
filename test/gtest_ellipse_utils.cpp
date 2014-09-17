/*!
  \file        gtest_ellipse_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/18
  
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

 */
// Bring in gtest
#include <gtest/gtest.h>
#include "vision_utils/image_utils/ellipse_utils.h"

typedef cv::Point2i Pt2;

void test_three_pts2ellipse(Pt2 center, Pt2 end1, Pt2 end2,
                            double expected_w, double expected_h, double expected_theta_deg) {
  ellipse_utils::Ellipse e = ellipse_utils::three_pts2ellipse(center, end1, end2);
  ASSERT_NEAR(e.size.width, expected_w, 1E-2);
  ASSERT_NEAR(e.size.height, expected_h, 1E-2);
  ASSERT_NEAR(e.angle, expected_theta_deg, 1E-2);
}

TEST(TestSuite, three_pts2ellipse1) {
  test_three_pts2ellipse(Pt2(0, 0), Pt2(2, 0), Pt2(0, 1), 4, 2, 0);
}

TEST(TestSuite, three_pts2ellipse2) {
  test_three_pts2ellipse(Pt2(0, 0), Pt2(0, 1), Pt2(-2, 0), 2, 4, 90);
}

////////////////////////////////////////////////////////////////////////////////

void test_ellipse_axes(Pt2 center, double w, double h, double theta_deg,
                       Pt2 exp_long_axis_end1, Pt2 exp_long_axis_end2,
                       Pt2 exp_short_axis_end1, Pt2 exp_short_axis_end2) {
  ellipse_utils::Ellipse e(center, cv::Size2f(w, h), theta_deg);
  Pt2 long1, long2, short1, short2;
  ellipse_utils::ellipse_axes(e, long1, long2, short1, short2);
  ASSERT_TRUE((long1 == exp_long_axis_end1 && long2 == exp_long_axis_end2)
              || (long1 == exp_long_axis_end2 && long2 == exp_long_axis_end1))
      << "long1:" << long1 << ", long2:" << long2;
  ASSERT_TRUE((short1 == exp_short_axis_end1 && short2 == exp_short_axis_end2)
              || (short1 == exp_short_axis_end2 && short2 == exp_short_axis_end1))
      << "short1:" << short1 << ", short2:" << short2;
}

TEST(TestSuite, ellipse_axes1) {
  test_ellipse_axes(Pt2(0, 0), 4, 2, 0,  Pt2(2, 0), Pt2(-2, 0), Pt2(0, 1), Pt2(0, -1));
}

TEST(TestSuite, ellipse_axes2) {
  test_ellipse_axes(Pt2(0, 0), 4, 2, 90, Pt2(0, 2), Pt2(0, -2), Pt2(1, 0), Pt2(-1, 0));
}

TEST(TestSuite, ellipse_axes3) {
  test_ellipse_axes(Pt2(3, 4), 4, 2, 90, Pt2(3, 6), Pt2(3, 2), Pt2(4, 4), Pt2(2, 4));
}

TEST(TestSuite, ellipse_axes_inverted) {
  test_ellipse_axes(Pt2(0, 0), 2, 4, 0,  Pt2(0, 2), Pt2(0, -2), Pt2(1, 0), Pt2(-1, 0));
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
