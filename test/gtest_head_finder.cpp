/*!
  \file        gtest_head_finder.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/17

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

Some tests for HeadFinder
 */
// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "vision_utils/timer.h"
#include <vision_utils/img_path.h>
#include "vision_utils/head_finder.h"
#include <vision_utils/distance_points.h>

bool display = false;

void test_find(const cv::Mat1b & user_mask, bool expect_success = true,
               cv::Point expected_head_pos = cv::Point(), bool has_answer_in_white = false) {
  vision_utils::HeadFinder finder;
  cv::Point head_pos;
  cv::Mat1b user_mask_255 = (user_mask > 0);
  vision_utils::Timer timer;
  bool retval = finder.find(user_mask_255, head_pos);
  ASSERT_TRUE(retval == expect_success);
  timer.printTime("find()");
  if (!expect_success)
    return;

  // now try to find the correct value
  if (display) {
    finder.illus(user_mask_255);
  }
  if (has_answer_in_white) {
    cv::Mat1b user_mask_white = (user_mask == 255);
    std::vector<cv::Point> pts;
    vision_utils::nonNulPoints(user_mask_white, pts);
    if (pts.size() == 1)
      expected_head_pos = pts.front();
  }
  double dist = vision_utils::distance_points(head_pos, expected_head_pos);
  printf("dist:%g\n", dist);
  ASSERT_TRUE(dist < 50)
      << "head_pos:" << head_pos
      << ", expected_head_pos:" << expected_head_pos
      << ", dist:" << dist;
}

void test_find(const std::string & filename, bool expect_success = true,
               cv::Point expected_head_pos = cv::Point(), bool has_answer_in_white = false) {
  cv::Mat1b user_mask = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  test_find(user_mask, expect_success, expected_head_pos, has_answer_in_white);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, ctor) {
  vision_utils::HeadFinder finder;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_mask) {
  cv::Point head_pos;
  cv::Mat1b user_mask;
  vision_utils::HeadFinder finder;
  bool retval = finder.find(user_mask, head_pos);
  ASSERT_TRUE(!retval);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, black_mask) {
  test_find(cv::Mat1b(50, 50, (uchar) 0), false);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, hard_mask1) {
  test_find(vision_utils::IMG_DIR() + "skeletons/heads/hard_mask1.png", true, cv::Point(313, 194), true);
}
TEST(TestSuite, hard_mask2) {
  test_find(vision_utils::IMG_DIR() + "skeletons/heads/hard_mask2.png", true, cv::Point(325, 181), true);
}
TEST(TestSuite, hard_mask3) {
  test_find(vision_utils::IMG_DIR() + "skeletons/heads/hard_mask3.png", true, cv::Point(325, 181), true);
}

TEST(TestSuite, heads01) {
  test_find(vision_utils::IMG_DIR() + "skeletons/heads/01.png", true, cv::Point(), true);
}
TEST(TestSuite, heads02) {
  test_find(vision_utils::IMG_DIR() + "skeletons/heads/02.png", true, cv::Point(), true);
}
TEST(TestSuite, heads03) {
  test_find(vision_utils::IMG_DIR() + "skeletons/heads/03.png", true, cv::Point(), true);
}
TEST(TestSuite, heads04) {
  test_find(vision_utils::IMG_DIR() + "skeletons/heads/04.png", true, cv::Point(), true);
}
TEST(TestSuite, heads05) {
  test_find(vision_utils::IMG_DIR() + "skeletons/heads/05.png", true, cv::Point(), true);
}
TEST(TestSuite, alberto1_user_mask) {
  test_find(vision_utils::IMG_DIR() + "depth/alberto1_user_mask.png", true, cv::Point(364, 74));
}
TEST(TestSuite, juggling1_user_mask) {
  test_find(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png", true, cv::Point(371, 67));
}
TEST(TestSuite, juggling2_user_mask) {
  test_find(vision_utils::IMG_DIR() + "depth/juggling2_user_mask.png", true, cv::Point(402, 72));
}
TEST(TestSuite, ainara1) {
  test_find(vision_utils::IMG_DIR() + "breast/2013-10-05_15-46-13-769_user_mask.png", true, cv::Point(268, 92));
}
TEST(TestSuite, ainara2) {
  test_find(vision_utils::IMG_DIR() + "breast/2013-10-05_15-46-03-861_user_mask.png", true, cv::Point(135, 102));
}
TEST(TestSuite, ref_skel) {
  test_find(cv::imread(vision_utils::IMG_DIR() + "skeletons/ref_skel.png", CV_LOAD_IMAGE_GRAYSCALE) == 255,
            true, cv::Point(129, 41));
}

TEST(TestSuite, david_arnaud1) {
  test_find(cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud1_user_mask.png", CV_LOAD_IMAGE_GRAYSCALE) == 1,
            true, cv::Point(340, 102));
}

TEST(TestSuite, david_arnaud2) {
  test_find(cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud2_user_mask.png", CV_LOAD_IMAGE_GRAYSCALE) == 1,
            true, cv::Point(266, 86));
}

TEST(TestSuite, david_arnaud3) {
  test_find(cv::imread(vision_utils::IMG_DIR() + "depth/david_arnaud3_user_mask.png", CV_LOAD_IMAGE_GRAYSCALE) == 1,
            true, cv::Point(338, 119));
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
