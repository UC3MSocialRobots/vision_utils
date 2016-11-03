/*!
  \file        color_utils_ros.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/6

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

#ifndef COLOR_UTILS_ROS_H
#define COLOR_UTILS_ROS_H

#include <std_msgs/ColorRGBA.h>
#include <opencv2/core/core.hpp>

namespace vision_utils {
//cut:uchar_bgr_color_to_ros_rgba_color
/*!
 * Converts a 3 fields structure into a ROS std_msgs::ColorRGBA color.
 * \param b, g, r
 *   The blue, green, red fields, between 0 and 255
 * \return
 *   The corresponding color, with fields in 0..1.
 */
template<class Value255>
std_msgs::ColorRGBA uchar_bgr_color_to_ros_rgba_color
(const Value255& b, const Value255& g, const Value255& r) {
  std_msgs::ColorRGBA color;
  color.r = r / 255.f;
  color.g = g / 255.f;
  color.b = b / 255.f;
  color.a = 1;
  return color;
}

////////////////////////////////////////////////////////////////////////////////
//cut:uchar_bgr_image_to_ros_rgba_color_vector
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

#endif // COLOR_UTILS_ROS_H
