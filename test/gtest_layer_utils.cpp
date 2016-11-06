/*!
  \file        gtest_layer_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/29

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
#include <opencv2/highgui/highgui.hpp>
#include <vision_utils/img_path.h>
#include "vision_utils/matrix_testing.h"
#include "vision_utils/rgb2hue.h"
#include "vision_utils/rgb2value.h"
#include "vision_utils/rgb_saturate_saturation_value.h"
#include "vision_utils/rgb2saturation.h"
#include "vision_utils/saturate_saturation_value.h"
#include "vision_utils/timer.h"


TEST(TestSuite, rgb2hue_four_colors_img) {
  cv::Mat3b src_bgr(2, 2), temp_hsv;
  src_bgr.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 0, 0); // blue  => H360 = 240
  src_bgr.at<cv::Vec3b>(0, 1) = cv::Vec3b(0, 255, 0); // green => H360 = 120
  src_bgr.at<cv::Vec3b>(1, 0) = cv::Vec3b(0, 0, 255); // red   => H360 = 0
  src_bgr.at<cv::Vec3b>(1, 1) = cv::Vec3b(255, 255, 0); // cyan=> H360 = 180
  cv::Mat1b dest_hue;
  vision_utils::rgb2hue(src_bgr, temp_hsv, dest_hue);

  //std::cout << "dest_hue" << dest_hue << std::endl;
  ASSERT_TRUE(dest_hue.at<uchar>(0, 0) == 240/2) << "dest_hue" << dest_hue;
  ASSERT_TRUE(dest_hue.at<uchar>(0, 1) == 120/2) << "dest_hue" << dest_hue;
  ASSERT_TRUE(dest_hue.at<uchar>(1, 0) == 0/2)   << "dest_hue" << dest_hue;
  ASSERT_TRUE(dest_hue.at<uchar>(1, 1) == 180/2) << "dest_hue" << dest_hue;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, rgb_saturate_saturation_value_four_colors_img) {
  cv::Mat3b src_bgr(2, 2), img_hsv_viz;
  src_bgr.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 0, 0); // blue  => H360 = 240
  src_bgr.at<cv::Vec3b>(0, 1) = cv::Vec3b(0, 255, 0); // green => H360 = 120
  src_bgr.at<cv::Vec3b>(1, 0) = cv::Vec3b(0, 0, 255); // red   => H360 = 0
  src_bgr.at<cv::Vec3b>(1, 1) = cv::Vec3b(255, 255, 0); // cyan=> H360 = 180
  vision_utils::rgb_saturate_saturation_value(src_bgr, img_hsv_viz);

  //std::cout << "dest_hue" << dest_hue << std::endl;
  ASSERT_TRUE(vision_utils::matrices_equal(src_bgr, img_hsv_viz));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, rgb_saturate_saturation_value_bench) {
  cv::Mat3b src_bgr = cv::imread(vision_utils::IMG_DIR() + "balloon.png"), out_bgr, out_bgr2;
  unsigned int ntimes = 10;
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time)
    vision_utils::rgb_saturate_saturation_value_slow(src_bgr, out_bgr);
  timer.printTime_factor("rgb_saturate_saturation_value_slow()", ntimes);

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    vision_utils::rgb_saturate_saturation_value(src_bgr, out_bgr2);
  timer.printTime_factor("rgb_saturate_saturation_value()", ntimes);

  //cv::imshow("out_bgr", out_bgr); cv::imshow("out_bgr2", out_bgr2); cv::waitKey(0);
  ASSERT_TRUE(vision_utils::matrices_near(out_bgr, out_bgr2, 1));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, rgb_saturate_saturation_value_img) {
  cv::Mat3b src_bgr = cv::imread(vision_utils::IMG_DIR() + "balloon.png"), out_bgr;
  std::vector<cv::Mat> layers;
  vision_utils::saturate_saturation_value(src_bgr, layers, out_bgr);
  ASSERT_TRUE(src_bgr.size() == out_bgr.size());
  // cv::imshow("out", out_bgr); cv::waitKey(0);

  cv::Mat3b temp_hsv;
  cv::Mat1b src_hue, out_hue, out_saturation, out_value;
  // hue in src_bgr and out_bgr should match
  vision_utils::rgb2hue(src_bgr, temp_hsv, src_hue);
  vision_utils::rgb2hue(out_bgr, temp_hsv, out_hue);
  ASSERT_TRUE(vision_utils::matrices_equal(src_hue, out_hue));


  // out_saturation should be = 255
  vision_utils::rgb2saturation(out_bgr, temp_hsv, out_saturation);
  ASSERT_TRUE(cv::countNonZero(out_saturation - 255) == 0);

  // out_value should be = 255
  vision_utils::rgb2value(out_bgr, temp_hsv, out_value);
  ASSERT_TRUE(cv::countNonZero(out_value - 255) == 0);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



