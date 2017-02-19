/*!
  \file        gtest_ground_plane_finder.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/11

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
#include <ros/ros.h>
#include "vision_utils/timer.h"
#include "vision_utils/depth_image_to_vizualisation_color_image.h"
#include "vision_utils/ground_plane_finder.h"
#include "vision_utils/read_rgb_and_depth_image_from_image_file.h"

#include <vision_utils/img_path.h>

bool display = false;

TEST(TestSuite, empty) {
  vision_utils::GroundPlaneFinder finder;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_img) {
  vision_utils::GroundPlaneFinder finder;
  cv::Mat1f depth;
  bool ok = finder.compute_plane(depth);
  ASSERT_TRUE(!ok);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, nan_img) {
  vision_utils::GroundPlaneFinder finder;
  cv::Mat1f depth(50, 50, std::numeric_limits<float>::quiet_NaN());
  bool ok = finder.compute_plane(depth);
  ASSERT_TRUE(!ok);
}

////////////////////////////////////////////////////////////////////////////////

void test_prefix(const std::string & filename_prefix) {
  vision_utils::GroundPlaneFinder finder;
  cv::Mat3b rgb;
  cv::Mat1f depth;
  ASSERT_TRUE(vision_utils::read_rgb_and_depth_image_from_image_file
              (filename_prefix, &rgb, &depth));
  vision_utils::Timer timer;
  ASSERT_TRUE(finder.compute_plane(depth));
  timer.printTime("find");
  cv::Mat1b mask;
  ASSERT_TRUE(finder.to_img(depth, mask));
if (display) {
  cv::imshow("rgb", rgb);
  cv::imshow("depth", vision_utils::depth2viz(depth));
  cv::imshow("mask", mask);
  cv::waitKey(0);
    } // end if display
  //vision_utils::imwrite_debug(vision_utils::timestamp()+"_mask.png", mask, vision_utils::MONOCHROME);
  ASSERT_TRUE(mask.size() == depth.size());
  double ratio = 1.*cv::countNonZero(mask)/(mask.cols*mask.rows);
  ASSERT_TRUE(ratio > .05) << "ratio:" << ratio; // 5%
}

TEST(TestSuite, juggling1) { test_prefix(vision_utils::IMG_DIR() + "depth/juggling1"); }
TEST(TestSuite, juggling2) { test_prefix(vision_utils::IMG_DIR() + "depth/juggling2"); }
TEST(TestSuite, juggling3) { test_prefix(vision_utils::IMG_DIR() + "depth/juggling3"); }
TEST(TestSuite, alberto1) { test_prefix(vision_utils::IMG_DIR() + "depth/alberto1"); }
TEST(TestSuite, alberto2) { test_prefix(vision_utils::IMG_DIR() + "depth/alberto2"); }
TEST(TestSuite, alvaro1) { test_prefix(vision_utils::IMG_DIR() + "depth/alvaro1"); }
TEST(TestSuite, alvaro2) { test_prefix(vision_utils::IMG_DIR() + "depth/alvaro2"); }
TEST(TestSuite, david_arnaud1) { test_prefix(vision_utils::IMG_DIR() + "depth/david_arnaud1"); }
TEST(TestSuite, david_arnaud2) { test_prefix(vision_utils::IMG_DIR() + "depth/david_arnaud2"); }
TEST(TestSuite, david_arnaud3) { test_prefix(vision_utils::IMG_DIR() + "depth/david_arnaud3"); }
TEST(TestSuite, ainara1) { test_prefix(vision_utils::IMG_DIR() + "breast/2013-10-05_15-46-03-286"); }


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
