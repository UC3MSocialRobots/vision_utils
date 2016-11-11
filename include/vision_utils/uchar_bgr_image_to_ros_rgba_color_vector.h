/*!
  \file        uchar_bgr_image_to_ros_rgba_color_vector.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/2
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
 */

#ifndef UCHAR_BGR_IMAGE_TO_ROS_RGBA_COLOR_VECTOR_H
#define UCHAR_BGR_IMAGE_TO_ROS_RGBA_COLOR_VECTOR_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

void uchar_bgr_image_to_ros_rgba_color_vector
(const cv::Mat3b & bgr_image,
 std::vector<std_msgs::ColorRGBA> & out,
 const int data_step = 1,
 const cv::Mat1b & mask = cv::Mat(),
 bool skip_black_points = false)
{
  // dimensions check
  bool use_mask = (!mask.empty());
  if (use_mask && bgr_image.size() != mask.size()) {
    printf("uchar_bgr_image_to_ros_rgba_color_vector(): "
           "bgr_image size (%i, %i) != mask size (%i, %i) -> "
           "no using mask\n", bgr_image.cols, bgr_image.rows, mask.cols, mask.rows);
    use_mask = false;
  }

  int cols = bgr_image.cols, rows = bgr_image.rows;
  int n_pts = rows * cols / (data_step * data_step);
  out.clear();
  out.reserve(n_pts);

  const uchar* mask_ptr;
  for (int row = 0; row < rows; row += data_step) {
    // get the address of row
    const cv::Vec3b* bgr_data = bgr_image.ptr<cv::Vec3b>(row);
    if (use_mask)
      mask_ptr = mask.ptr<uchar>(row);
    for (int col = 0; col < cols; col += data_step) {
      if (skip_black_points &&
          bgr_data[col][0] == 0 &&
          bgr_data[col][1] == 0 &&
          bgr_data[col][2] == 0)
        continue;
      if (use_mask && mask_ptr[col] == 0)
        continue;
      // get color
      out.push_back
          (uchar_bgr_color_to_ros_rgba_color(
             bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]
          ));
    } // end loop col
  } // end loop row
} // end uchar_bgr_image_to_ros_rgba_color_vector();

} // end namespace vision_utils

#endif // UCHAR_BGR_IMAGE_TO_ROS_RGBA_COLOR_VECTOR_H
