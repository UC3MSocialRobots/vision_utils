/*!
  \file        gtest_atan3D.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/27

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
#include "vision_utils/atan3D.h"
#include <vision_utils/rosmaster_alive.h>

void test_convert3Dto2D(ros::NodeHandle & nh_private,
                        vision_utils::ReprojectionMode & mode,
                        const std::string & mode_str,
                        const double & x, const double & y, const double & z,
                        const double & exp_x2D,
                        const double & exp_y2D) {
  nh_private.setParam("reprojection_mode", mode_str);
  mode.check_param(true);
  double x2D, y2D;
  mode.convert3Dto2D(x, y, z, x2D, y2D);
  ASSERT_TRUE(x2D == exp_x2D && y2D == exp_y2D)
      << "mode:" << mode_str
      << ", x2D:" << x2D << " (expected:" << exp_x2D << ")"
      << ", y2D:" << y2D << " (expected:" << exp_y2D << ")";
}

TEST(TestSuite, convert3Dto2D) {
  if (!vision_utils::rosmaster_alive()) return;
  ros::NodeHandle nh_private("~");
  vision_utils::ReprojectionMode mode;
  double x = 1, y = 2, z = 3;
  test_convert3Dto2D(nh_private, mode, "xy", x, y, z, x, y);
  test_convert3Dto2D(nh_private, mode, "+xy", x, y, z, x, y);
  test_convert3Dto2D(nh_private, mode, "x+y", x, y, z, x, y);
  test_convert3Dto2D(nh_private, mode, "-x+y", x, y, z, -x, y);
  test_convert3Dto2D(nh_private, mode, "-x-y", x, y, z, -x, -y);

  test_convert3Dto2D(nh_private, mode, "yx", x, y, z, y, x);
  test_convert3Dto2D(nh_private, mode, "-y+x", x, y, z, -y, +x);
  test_convert3Dto2D(nh_private, mode, "-y-x", x, y, z, -y, -x);

  test_convert3Dto2D(nh_private, mode, "xz", x, y, z, x, z);
  test_convert3Dto2D(nh_private, mode, "-z+x", x, y, z, -z, x);
}

////////////////////////////////////////////////////////////////////////////////

void test_convert2Dto3D(ros::NodeHandle & nh_private,
                        vision_utils::ReprojectionMode & mode,
                        const std::string & mode_str,
                        const double & x2D, const double & y2D, const double & z2D,
                        const double & exp_x,
                        const double & exp_y,
                        const double & exp_z) {
  nh_private.setParam("reprojection_mode", mode_str);
  mode.check_param(true);
  double x, y, z;
  mode.convert2Dto3D(x2D, y2D, z2D, x, y, z);
  ASSERT_TRUE(x == exp_x && y == exp_y && z == exp_z)
      << "mode:" << mode_str
      << ", x:" << x << " (expected:" << exp_x << ")"
      << ", y:" << y << " (expected:" << exp_y << ")"
      << ", z:" << z << " (expected:" << exp_z << ")";
}

TEST(TestSuite, convert2Dto3D) {
  if (!vision_utils::rosmaster_alive()) return;
  ros::NodeHandle nh_private("~");
  vision_utils::ReprojectionMode mode;
  double x2D = 1, y2D = 2, z2D = 3;
  test_convert2Dto3D(nh_private, mode, "xy", x2D, y2D, z2D, x2D, y2D, z2D);
  test_convert2Dto3D(nh_private, mode, "+xy", x2D, y2D, z2D, x2D, y2D, z2D);
  test_convert2Dto3D(nh_private, mode, "x+y", x2D, y2D, z2D, x2D, y2D, z2D);
  test_convert2Dto3D(nh_private, mode, "-x+y", x2D, y2D, z2D, -x2D, y2D, z2D);
  test_convert2Dto3D(nh_private, mode, "-x-y", x2D, y2D, z2D, -x2D, -y2D, z2D);

  test_convert2Dto3D(nh_private, mode, "yx", x2D, y2D, z2D, y2D, x2D, z2D);
  test_convert2Dto3D(nh_private, mode, "-y+x", x2D, y2D, z2D, +y2D, -x2D, z2D);
  test_convert2Dto3D(nh_private, mode, "-y-x", x2D, y2D, z2D, -y2D, -x2D, z2D);

  test_convert2Dto3D(nh_private, mode, "xz", x2D, y2D, z2D, x2D, z2D, y2D);
  test_convert2Dto3D(nh_private, mode, "-x+z", x2D, y2D, z2D, -x2D, z2D, y2D);

  test_convert2Dto3D(nh_private, mode, "yz", x2D, y2D, z2D, z2D, x2D, y2D);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "gtest_atan3D");
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
