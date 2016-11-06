/*!
  \file        gtest_voronoi.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/11

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

Some tests for Voronoi functions

 */
// Bring in gtest
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/timer.h"
#include <vision_utils/img_path.h>
#include "vision_utils/voronoi.h"

#define SKEL_DIR vision_utils::IMG_DIR() + "skeletons/"
#define MAX_RATIO_DIFF 2E-2

double compute_ratio_diff(const cv::Mat1b & A, const cv::Mat1b & B) {
  cv::Mat1b diff;
  cv::bitwise_xor(A > 0, B > 0, diff);
  return 1.f * cv::countNonZero(diff) / (A.cols * A.rows);
}

void test_thin_impl(const cv::Mat1b & query, const cv::Mat1b & correct_ans,
                    std::string implementation_name,
                    bool crop_img_before,
                    bool only_print = false) {
  printf("test_thin_impl('%s', crop:%i)\n",
         implementation_name.c_str(), crop_img_before);
  // make the effective thinning
  vision_utils::VoronoiThinner thinner;
  vision_utils::Timer timer;
  thinner.thin(query, implementation_name, crop_img_before);
  timer.printTime(implementation_name.c_str());

  //  cv::imshow("query", query);
  //  cv::imshow("skel.get_skeleton()", skel.get_skeleton());
  //  cv::imshow("correct_ans", correct_ans);
  //  cv::waitKey(0);

  if (only_print) {
    std::cout << "implementation_name:'" << implementation_name << "'"
              << ", crop:" << crop_img_before
              << ", query:" << vision_utils::ImageContour::to_string(query)
              << "skel.get_skeleton():" << vision_utils::ImageContour::to_string(thinner.get_skeleton())
              << std::endl;
    return;
  }

  // compare with result
  cv::Mat1b correct_ans_cropped;
  correct_ans(thinner.get_bbox()).copyTo(correct_ans_cropped);
  printf("correct_ans_cropped:(%ix%i), skel:(%ix%i)\n",
         correct_ans_cropped.cols, correct_ans_cropped.rows,
         thinner.get_skeleton().cols, thinner.get_skeleton().rows);
  double ratio_diff = compute_ratio_diff(correct_ans_cropped, thinner.get_skeleton());
  printf("ratio_diff:%g\n", ratio_diff);
  ASSERT_TRUE(ratio_diff < MAX_RATIO_DIFF)
      << "implementation_name:'" << implementation_name << "'"
      << ", crop:" << crop_img_before
      << ", ratio_diff:" << ratio_diff
      << ", query:" << vision_utils::ImageContour::to_string(query)
      << "correct_ans:" << vision_utils::ImageContour::to_string(correct_ans_cropped)
      << "skel.get_skeleton():" << vision_utils::ImageContour::to_string(thinner.get_skeleton());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_black_img) {
  cv::Mat1b query(50, 50, (uchar) 0),
      correct_ans = query.clone();
  test_thin_impl(query, correct_ans, IMPL_MORPH, false);
  test_thin_impl(query, correct_ans, IMPL_MORPH, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_white_img_no_border) {
  cv::Mat1b query(5, 10, (uchar) 255),
      correct_ans = query.clone();
  correct_ans.setTo(0);
  cv::line(correct_ans, cv::Point(2, 2), cv::Point(6, 2), cv::Scalar::all(255), 1);
  // will only work with crop
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, true);

  correct_ans.setTo(0);
  cv::line(correct_ans, cv::Point(2, 2), cv::Point(7, 2), cv::Scalar::all(255), 1);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_white_img_border) {
  cv::Mat1b query(5, 10, (uchar) 0), correct_ans = query.clone();
  cv::rectangle(query, cv::Rect(1, 1, query.cols - 2, query.rows - 2), cv::Scalar::all(255), -1);
  correct_ans.setTo(0);
  cv::line(correct_ans, cv::Point(2, 2), cv::Point(6, 2), cv::Scalar::all(255), 1);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, true);

  correct_ans.setTo(0);
  cv::line(correct_ans, cv::Point(2, 2), cv::Point(7, 2), cv::Scalar::all(255), 1);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_single_dot) {
  int cols = 50;
  cv::Mat1b query(cols, cols, (uchar) 0);
  query(cols / 2, cols / 2) = 255;
  cv::Mat1b correct_ans = query.clone();
  test_thin_impl(query, correct_ans, IMPL_MORPH, false);
  test_thin_impl(query, correct_ans, IMPL_MORPH, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, true);
}

TEST(TestSuite, test_circle) {
  cv::Mat1b query(7, 7, (uchar) 0);
  cv::Mat1b correct_ans = query.clone();
  correct_ans << 0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,
      0,0,1,1,0,0,0,
      0,1,1,1,1,0,0,
      0,1,1,1,1,0,0,
      0,0,1,1,0,0,0,
      0,0,0,0,0,0,0,
  test_thin_impl(query, correct_ans, IMPL_MORPH, false, true);
  test_thin_impl(query, correct_ans, IMPL_MORPH, true, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, false, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, true, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, false, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, true, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, false, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, true, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, false, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, true, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, false, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, true, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, false, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, true, true);
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_morph_opencv) {
  cv::Mat1b query = cv::imread(SKEL_DIR "opencv_src.png", CV_LOAD_IMAGE_GRAYSCALE),
      correct_ans = cv::imread(SKEL_DIR "opencv_morph.png", CV_LOAD_IMAGE_GRAYSCALE);
  test_thin_impl(query, correct_ans, IMPL_MORPH, false);
  test_thin_impl(query, correct_ans, IMPL_MORPH, true);
}


TEST(TestSuite, test_morph_O) {
  cv::Mat1b query = cv::imread(SKEL_DIR "single_O_src.png", CV_LOAD_IMAGE_GRAYSCALE),
      correct_ans = cv::imread(SKEL_DIR "single_O_morph.png", CV_LOAD_IMAGE_GRAYSCALE);
  test_thin_impl(query, correct_ans, IMPL_MORPH, false);
  test_thin_impl(query, correct_ans, IMPL_MORPH, true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, minitest_guo_hall) {
  cv::Mat1b query(9, 7, (uchar) 0); // rows, cols
  cv::rectangle(query, cv::Point(1, 1), cv::Point(3, 7), cv::Scalar::all(255), -1);
  cv::rectangle(query, cv::Point(1, 5), cv::Point(5, 7), cv::Scalar::all(255), -1);
  std::cout << "query:" << vision_utils::ImageContour::to_string(query) << std::endl;
  cv::Rect foo(1, 1, 5, 5);
  std::cout << "foo" << foo << ", br:" << foo.br() << std::endl;
  cv::Mat1b correct_ans = query.clone();
  correct_ans << 0,0,0,0,0,0,0,
      0,0,0,0,0,0,0,
      0,0,1,0,0,0,0,
      0,0,1,0,0,0,0,
      0,0,1,0,0,0,0,
      0,0,0,1,0,0,0,
      0,0,0,0,1,0,0,
      0,0,0,0,0,0,0,
      0,0,0,0,0,0,0;
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, true);
}

TEST(TestSuite, test_lorem_guo_hall) {
  cv::Mat1b query = cv::imread(SKEL_DIR "lorem_src.png", CV_LOAD_IMAGE_GRAYSCALE),
      correct_ans = cv::imread(SKEL_DIR "lorem_guo_hall.png", CV_LOAD_IMAGE_GRAYSCALE);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_GUO_HALL_FAST, true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_zhang_suen_japanese) {
  cv::Mat1b query = cv::imread(SKEL_DIR "japanese_src.png", CV_LOAD_IMAGE_GRAYSCALE),
      correct_ans = cv::imread(SKEL_DIR "japanese_zhang_suen.png", CV_LOAD_IMAGE_GRAYSCALE);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_ORIGINAL, true);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, false);
  test_thin_impl(query, correct_ans, IMPL_ZHANG_SUEN_FAST, true);
}

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
