/*!
  \file        test_array_to_color.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/12

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

Some tests for array_to_color()

 */
bool display = false;
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/cmatrix.h"
#include "vision_utils/array_to_color.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);

  // create some data
  unsigned int ncols = 8, nrows = 12;
  vision_utils::CMatrix<double> data(nrows, ncols);
  for (unsigned int row = 0; row < nrows; ++row) {
    for (unsigned int col = 0; col < ncols; ++col) {
      data[row][col] = std::min(1., fabs(1. * row - col) / 10.);
    } // end loop col
  } // end loop row
  printf("data:'%s'", data.to_string(15).c_str());

  cv::Mat3b out1, out2, caption1, caption2;
  // draw with edges and greyscale colormap
  vision_utils::array_to_color(data, nrows, ncols, out1, 50, 50);
  vision_utils::colormap_to_caption_image(caption1, 100, 300, vision_utils::ratio2grey_inv,
                      0., 1., .1, .2);
  // draw with no edges and red_green colormap
  array_to_color(data, nrows, ncols, out2, 60, 40, false, true,
                 vision_utils::ratio2red_green,
                 &vision_utils::int_to_lowercase_letter, &vision_utils::int_to_number);
  vision_utils::colormap_to_caption_image(caption2, 100, 300, vision_utils::ratio2red_green,
                      0., 1., .05, .2, false);

if (display) {
  cv::imshow("out1", out1);
  cv::imshow("out2", out2);
  cv::imshow("caption1", caption1);
  cv::imshow("caption2", caption2);
  cv::imwrite("out1.png", out1);
  cv::imwrite("out2.png", out2);
  cv::waitKey(0);
    } // end if display
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
