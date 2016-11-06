/*!
  \file        gtest_image_contour.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/16

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

Some tests for class \b Imagecontour

 */
// Bring in gtest
#include <gtest/gtest.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/image_contour.h"
#include "vision_utils/timer.h"
#include <vision_utils/img_path.h>

void check_sizes(vision_utils::ImageContour & contour,
                 unsigned int expected_contour_size,
                 unsigned int expected_inner_size) {
  ASSERT_TRUE(contour.contour_size() == expected_contour_size)
      << "contour:" << contour.to_string();
  ASSERT_TRUE(contour.inside_size() == expected_inner_size)
      << "contour:" << contour.to_string();
}

void set_point_empty_C4(vision_utils::ImageContour & contour, int row, int col,
                        unsigned int /*expected_contour_size*/,
                        unsigned int /*expected_inner_size*/)
{
  contour.set_point_empty_C4(row, col); // up left
  ASSERT_TRUE(contour(row, col) == vision_utils::ImageContour::EMPTY) << "contour" << contour.to_string();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_contour) {
  vision_utils::ImageContour contour;
  ASSERT_TRUE(contour.cols == 0);
  ASSERT_TRUE(contour.rows == 0);
  check_sizes(contour, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_img) {
  vision_utils::ImageContour contour;
  cv::Mat1b img;
  contour.from_image_C4(img);
  ASSERT_TRUE(contour.cols == 0);
  ASSERT_TRUE(contour.rows == 0);
  check_sizes(contour, 0, 0);

  int cols = 50;
  img.create(cols, cols);
  img.setTo(0);
  contour.from_image_C4(img);
  ASSERT_TRUE(contour.cols == cols);
  ASSERT_TRUE(contour.rows == cols);
  check_sizes(contour, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, rect_img) {
  int cols = 10;
  cv::Mat1b img(cols, cols, (uchar) 0);
  vision_utils::ImageContour contour;

  for (int col = 0; col < cols; ++col) {
    for (int row = 0; row < cols; ++row) {
      img.setTo(0);
      img.at<uchar>(row, col) = 255;
      contour.from_image_C4(img);
      ASSERT_TRUE(contour.cols == cols);
      ASSERT_TRUE(contour.rows == cols);
      ASSERT_TRUE(contour.contour_size() == 1)
          << "(" << col << "," << row << "): contour:" << contour.to_string();
      ASSERT_TRUE(contour.inside_size() == 0)
          << "(" << col << "," << row << "): contour:" << contour.to_string();
      ASSERT_TRUE(contour(row, col) == vision_utils::ImageContour::CONTOUR)
          << "(" << col << "," << row << "): contour:" << contour.to_string();

      // make hole
      contour.set_point_empty_C4(row, col);
      ASSERT_TRUE(contour.contour_size() == 0)
          << "(" << col << "," << row << "): contour:" << contour.to_string();
      ASSERT_TRUE(contour.inside_size() == 0)
          << "(" << col << "," << row << "): contour:" << contour.to_string();
    } // end loop row
  } // end loop col
}

////////////////////////////////////////////////////////////////////////////////

void test_rect(bool C8 = false) {
  int cols = 100, rows = 80, ntimes = 100;
  vision_utils::ImageContour contour;
  cv::Mat1b img(rows, cols, (uchar) 255), correct_ans = img.clone();
  correct_ans.setTo(0);
  cv::rectangle(correct_ans, cv::Rect(0, 0, cols, rows), cv::Scalar::all(255), 1);
  for (int time = 0; time < ntimes; ++time) {
    if (C8)
      contour.from_image_C8(img);
    else
      contour.from_image_C4(img);
    ASSERT_TRUE(cv::countNonZero(contour.contour_image() - correct_ans) == 0)
        << "contour:" << contour.to_string()
        << "correct_ans:" << vision_utils::ImageContour::to_string(correct_ans);
  } // end loop time
}

TEST(TestSuite, test_rect_C4) {  test_rect(false); }
TEST(TestSuite, test_rect_C8) {  test_rect(true); }

////////////////////////////////////////////////////////////////////////////////

void test_circle(bool C8 = false) {
  int cols = 100, rows = 80, ntimes = 100;
  vision_utils::ImageContour contour;
  cv::Mat1b img(rows, cols, (uchar) 0), correct_ans = img.clone();
  for (int time = 0; time < ntimes; ++time) {
    img.setTo(0);
    correct_ans.setTo(0);
    cv::Point center(rand() % rows, rand() % cols);
    int radius = rand() % cols;
    cv::circle(img, center, radius, cv::Scalar::all(255), -1);
    cv::circle(correct_ans, center, radius, cv::Scalar::all(255), 1);
    // also add the border
    img.col(0     ).copyTo(correct_ans.col(0     ));
    img.col(cols-1).copyTo(correct_ans.col(cols-1));
    img.row(0     ).copyTo(correct_ans.row(0     ));
    img.row(rows-1).copyTo(correct_ans.row(rows-1));
    if (C8)
      contour.from_image_C8(img);
    else
      contour.from_image_C4(img);
    ASSERT_TRUE(cv::countNonZero(contour.contour_image() - correct_ans) == 0)
        << "contour:" << contour.to_string()
        << "correct_ans:" << vision_utils::ImageContour::to_string(correct_ans);
  } // end loop time
}

TEST(TestSuite, circleC4) {
  test_circle(false);
}

TEST(TestSuite, circleC8) {
  // test_circle(true);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, rectangle) {
  int cols = 100, ntimes = 100;
  vision_utils::ImageContour contour;
  cv::Mat1b img(cols, cols, (uchar) 0), correct_ans = img.clone();
  for (int time = 0; time < ntimes; ++time) {
    img.setTo(0);
    correct_ans.setTo(0);
    cv::Point begin(rand() % cols, rand() % cols), end(rand() % cols, rand() % cols);
    cv::rectangle(img, begin, end, cv::Scalar::all(255), -1);
    cv::rectangle(correct_ans, begin, end, cv::Scalar::all(255), 1);
    contour.from_image_C4(img);
    ASSERT_TRUE(cv::countNonZero(contour.contour_image() - correct_ans) == 0);
  } // end loop time
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, eat_rectangle) {
  cv::Mat1b img(4, 4, (uchar) 0);
  cv::rectangle(img, cv::Point(1, 1), cv::Point(3, 3), cv::Scalar::all(255), -1);
  vision_utils::ImageContour contour;
  contour.from_image_C4(img);
  //XXX
  //XOX
  //XXX
  check_sizes(contour, 8, 1);

  set_point_empty_C4(contour, 1, 1,  7, 1); // up left
  //-XX
  //XOX
  //XXX

  set_point_empty_C4(contour, 1, 3,  6, 1); // up right
  //-X-
  //XOX
  //XXX

  set_point_empty_C4(contour, 3, 3,  5, 1); // down right
  //-X-
  //XOX
  //XX-

  set_point_empty_C4(contour, 3, 1,  4, 1); // down left
  //-X-
  //XOX
  //-X-

  set_point_empty_C4(contour, 2, 2,  4, 0); // center
  //-X-
  //X-X
  //-X-

  set_point_empty_C4(contour, 2, 1,  3, 0); // left single pixel
  //-X-
  //--X
  //-X-

  // make hole in the middle
  contour.from_image_C4(img);
  //XXX
  //XOX
  //XXX
  check_sizes(contour, 8, 1);

  set_point_empty_C4(contour, 2, 2,  8, 0);
  //XXX
  //X-X
  //XXX
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, benchmark) {
  cv::Mat1b img = cv::imread(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png",
                             CV_LOAD_IMAGE_GRAYSCALE);
  unsigned int ntimes = 1000;
  vision_utils::ImageContour contour;
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time)
    contour.from_image_C4(img);
  timer.printTime_factor("from_image_C4()", ntimes);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

