/*!
  \file        gtest_mask_acceleration.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/5/10

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

Some tests for MaskAcceleration
*/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "vision_utils/mask_acceleration.h"
#include <vision_utils/distance_points.h>

bool display = false;

TEST(TestSuite, empty_img) {
  vision_utils::MaskAcceleration acc;
  ASSERT_FALSE(acc.from_mask(cv::Mat1b()));
}

TEST(TestSuite, black_img) {
  vision_utils::MaskAcceleration acc;
  ASSERT_FALSE(acc.from_mask(cv::Mat1b(100, 100, (uchar)0)));
}

TEST(TestSuite, white_img) {
  vision_utils::MaskAcceleration acc;
  ASSERT_TRUE(acc.from_mask(cv::Mat1b(100, 100, (uchar)255)));
}

TEST(TestSuite, still_circle) {
  vision_utils::MaskAcceleration acc;
  cv::Mat1b mask(100, 100, (uchar)0);
  cv::circle(mask, cv::Point(50, 50), 25, CV_RGB(255, 255, 255), -1);
  ASSERT_TRUE(acc.from_mask(mask));
  std::vector<vision_utils::MaskAcceleration::PixelAcceleration> accs
      = acc.get_pixel_accelerations();
  ASSERT_TRUE(accs.empty());

  // and again, to compute really accelerations
  ASSERT_TRUE(acc.from_mask(mask));
  ASSERT_TRUE(accs.empty()); // still
}

TEST(TestSuite, octogon_at_border) {
  vision_utils::MaskAcceleration acc;
  cv::Mat3b illus;
  unsigned int npasses = 5;
  for (  unsigned int pass = 0; pass < npasses; ++pass) {
    unsigned int w = 80 + 50 * pass, h = w;
    cv::Mat1b mask(h, w, (uchar)0);
    int off = 25 * (npasses - pass - 1);
    cv::Point offset(off, off);
    // draw octogon
    std::vector<cv::Point> pts;
    pts.push_back(cv::Point(w/3,   0));
    pts.push_back(cv::Point(2*w/3, 0));
    pts.push_back(cv::Point(w-1,   h/3));
    pts.push_back(cv::Point(w-1,   2*h/3));
    pts.push_back(cv::Point(2*w/3, h-1));
    pts.push_back(cv::Point(w/3,   h-1));
    pts.push_back(cv::Point(0,     2*h/3));
    pts.push_back(cv::Point(0,     h/3));
    cv::fillPoly(mask, std::vector< std::vector<cv::Point> > (1, pts),
                 cv::Scalar::all(255));
    ASSERT_TRUE(acc.from_mask(mask, offset));
    acc.draw_illus(illus);
if (display) {
    cv::imshow("mask", mask);
    cv::imshow("illus", illus);
    cv::waitKey(0);
    } // end if display
    std::vector<vision_utils::MaskAcceleration::PixelAcceleration> accs
        = acc.get_pixel_accelerations();
    if (pass == 0)
      ASSERT_TRUE(accs.empty()); // first pass -> no accel
    else
      ASSERT_FALSE(accs.empty()); // growing octogon
  } // end for pass
}

void test_growing_circle(const cv::Point & offset) {
  vision_utils::MaskAcceleration acc;
  unsigned int start_radius = 15, radius_step = 2;
  cv::Mat1b mask(100, 100);
  cv::Point circle_center(50, 50);
  for (unsigned int curr_radius = start_radius; curr_radius < 40; curr_radius+=radius_step) {
    // draw growing circle
    mask.setTo(0);
    cv::circle(mask, circle_center, curr_radius, CV_RGB(255, 255, 255), -1);
    ASSERT_TRUE(acc.from_mask(mask, offset, 0 )) << "radius:" << curr_radius;
    if (curr_radius == start_radius)
      continue;
    // check PixelAccelerations
    std::vector<vision_utils::MaskAcceleration::PixelAcceleration> accs
        = acc.get_pixel_accelerations();
    unsigned int naccs = accs.size();
    ASSERT_TRUE(naccs > 0);
    cv::Mat3b illus;
    acc.draw_illus(illus, 10);
if (display) {
    cv::imshow("mask", mask);
    cv::imshow("illus", illus);
    cv::waitKey(0);
    } // end if display
    for (unsigned int i = 0; i < naccs; ++i) {
      vision_utils::MaskAcceleration::PixelAcceleration acc = accs[i];
      ASSERT_TRUE(acc.norm > 0 && acc.norm <= 4 * radius_step)
          << "radius:" << curr_radius << ", acc:" << acc.to_string() << ", norm:" << acc.norm;
      // check origin on the curr circle
      double dist_origin2center = vision_utils::distance_points
                                  (acc.origin - offset, circle_center);
      ASSERT_TRUE(fabs(dist_origin2center - curr_radius) < 1)
          << "radius:" << curr_radius << ", acc:" << acc.to_string() << ", dist_origin2center:" << dist_origin2center;
      // check end on the prev circle
      cv::Point end = acc.origin - offset
                      - acc.norm * cv::Point(cos(acc.orien), sin(acc.orien));
      double dist_end2center = vision_utils::distance_points(end, circle_center);
      ASSERT_TRUE(fabs(dist_end2center - curr_radius + radius_step) <= 2 * radius_step)
          << "radius:" << curr_radius << ", acc:" << acc.to_string() << ", dist_end2center:" << dist_end2center;
    } // end for i
  } // end for curr_radius
} // end test_growing_circle();

TEST(TestSuite, growing_circle_no_offset) {
  test_growing_circle(cv::Point(0, 0));
}

TEST(TestSuite, growing_circle_offset) {
  test_growing_circle(cv::Point(10, 20));
}

TEST(TestSuite, growing_circle_neg_offset) {
  test_growing_circle(cv::Point(-10, -20));
}

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
