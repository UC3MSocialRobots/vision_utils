/*!
  \file        pixel2world_rgb.h
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

#ifndef PIXEL2WORLD_RGB_H
#define PIXEL2WORLD_RGB_H
// std includes
#include "vision_utils/pixel2world_depth.h"
#include "vision_utils/uchar_bgr_color_to_ros_rgba_color.h"
#include <std_msgs/ColorRGBA.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

/*!
 * Reproject a pixel of the RGB into 3D.
 * Version with a known depth value for the RGB pixel.
 * \param rgb_to_depth_scale_x, rgb_to_depth_scale_y
 *   Enables converting the RGB pixel into a depth pixel.
 */
inline cv::Point3d pixel2world_rgb(const cv::Point2d & pt_rgb,
                                   const image_geometry::PinholeCameraModel & depth_cam_model,
                                   const double & depth_value,
                                   const double rgb_to_depth_scale_x = 1,
                                   const double rgb_to_depth_scale_y = 1) {
  return pixel2world_depth<cv::Point3d>
      (cv::Point2d(rgb_to_depth_scale_x * pt_rgb.x,
                   rgb_to_depth_scale_y * pt_rgb.y),
       depth_cam_model, depth_value);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Reproject a pixel of the RGB into 3D.
 * Version reading the depth value from the depth image.
 * \param rgb_to_depth_scale_x, rgb_to_depth_scale_y
 *   Enables converting the RGB pixel into a depth pixel.
 */
inline cv::Point3d pixel2world_rgb(const cv::Point2d & pt_rgb,
                                   const image_geometry::PinholeCameraModel & depth_cam_model,
                                   const cv::Mat & depth_img,
                                   const double rgb_to_depth_scale_x = 1,
                                   const double rgb_to_depth_scale_y = 1) {
  cv::Point2d depth_pt(rgb_to_depth_scale_x * pt_rgb.x,
                       rgb_to_depth_scale_y * pt_rgb.y);
  if (depth_pt.x < 0 || depth_pt.x >= depth_img.cols
      || depth_pt.y < 0 || depth_pt.y >= depth_img.rows) {
    printf("Depth point (%g,%g) is outside the depth image (%ix%i)\n",
             depth_pt.x, depth_pt.y,
             depth_img.cols, depth_img.rows);
    return cv::Point3d(0, 0, 0);
  }
  IplImage depth_img_ipl = depth_img;
  double depth_value =
      //depth_img.at<float>(depth_pt.y, depth_pt.x);
      CV_IMAGE_ELEM(&depth_img_ipl, float,
                    (int) depth_pt.y, (int) depth_pt.x);
  return pixel2world_depth<cv::Point3d >(depth_pt, depth_cam_model, depth_value);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief pixel2world_rgb_color255
 * \param bgr_img
 *   The color image
 * \param depth_img
 *   The depth image
 * \param depth_cam_model
 *   The camera model of the depth image
 * \param depth_reprojected
 *   [OUT] the 3D points
 * \param colors
 *   [OUT] a vector, of, for instance, cv::Scalar or cv::Vec3b
 * \param data_step
 *   The increment in columns and rows for reprojecting the pixels.
 *   Use 1 to reproject all pixels, 2 to reproject 50% of them, etc.
 * \param mask_depth
 *   An optional mask to indicate what points should be kept
 *  (0=> discared pixel, anything else => reproject)
 * \param remove_nans
 *    If true, do not insert into the cloud points that correspond to
 *    an undefined depth (shiny surfaces, etc.)
 * \return
 *    true if success
 */
template<class Pt3, class Color255>
inline bool pixel2world_rgb_color255
(const cv::Mat & bgr_img,
 const cv::Mat & depth_img,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 std::vector<Pt3> & depth_reprojected,
 std::vector<Color255> & colors,
 const unsigned int data_step = 1,
 const cv::Mat1b & mask_depth = cv::Mat(),
 bool remove_nans = false)
{
  // clear answer
  int cols = depth_img.cols, rows = depth_img.rows;
  int n_pts = rows * cols / (data_step * data_step);
  depth_reprojected.clear();
  depth_reprojected.reserve(n_pts);
  colors.clear();
  colors.reserve(n_pts);

  // dimensions check
  if (depth_img.size() != bgr_img.size()) {
    printf("pixel2world_rgb(): depth size (%i, %i) != bgr_img size (%i, %i)\n",
           depth_img.cols, depth_img.rows, bgr_img.cols, bgr_img.rows);
    return false;
  }
  bool use_mask = (!mask_depth.empty());
  if (use_mask && depth_img.size() != mask_depth.size()) {
    printf("pixel2world_rgb(): depth size (%i, %i) != mask_depth size (%i, %i) -> "
           "no using mask\n", depth_img.cols, depth_img.rows,
           mask_depth.cols, mask_depth.rows);
    use_mask = false;
  }
  if (!depth_cam_model.initialized()) {
    printf("pixel2world_rgb(): depth_cam_model not initialized\n");
    return false;
  }
  if (depth_img.empty()) {
    printf("pixel2world_rgb(): empty images\n");
    return true;
  }

  // locates matrix header within a parent matrix
  // cf http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  cv::Size size;
  cv::Point offset;
  bgr_img.locateROI(size, offset);

  const uchar* mask_ptr;
  for (int row = 0; row < rows; row += data_step) {
    // get the address of row
    const float* depth_ptr = depth_img.ptr<float>(row);
    const cv::Vec3b* bgr_data = bgr_img.ptr<cv::Vec3b>(row);
    if (use_mask)
      mask_ptr = mask_depth.ptr<uchar>(row);
    for (int col = 0; col < cols; col += data_step) {
      if (use_mask && mask_ptr[col] == 0)
        continue;
      if (remove_nans && is_nan_depth(depth_ptr[col]))
        continue;
      depth_reprojected.push_back
          ( pixel2world_depth<Pt3>
            (cv::Point2d(col + offset.x, row + offset.y),
             depth_cam_model, depth_ptr[col]) );
      //colors.push_back(uchar_bgr_color_to_ros_rgba_color(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]));
      colors.push_back(Color255(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]));
    } // end loop col
  } // end loop row
  return true;
} // end pixel2world_rgb_color255()

////////////////////////////////////////////////////////////////////////////////

/*!
 * A version of pixel2world_rgb() using std_msgs::ColorRGBA
 * instead of cv::Scalar.
 * It calls the cv::Scalar version of pixel2world_rgb_color255(),
 * then converts the obtained colors.
 *
 * For all params, cf the cv::Scalar version of pixel2world_rgb_color255().
 */
template<class Pt3>
inline bool pixel2world_rgb_color_rgba
(const cv::Mat & bgr_img,
 const cv::Mat & depth_img,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 std::vector<Pt3> & depth_reprojected,
 std::vector<std_msgs::ColorRGBA> & colors,
 const unsigned int data_step = 1,
 const cv::Mat1b & mask_depth = cv::Mat(),
 bool remove_nans = false)
{
  // call pixel2world_rgb() with cv::Scalar.
  std::vector<cv::Scalar> colors_scalars;
  bool ret = pixel2world_rgb_color255
             (bgr_img, depth_img, depth_cam_model,
              depth_reprojected, colors_scalars, data_step, mask_depth, remove_nans);
  // convert cv::Scalar -> std_msgs::ColorRGBA
  colors.clear();
  colors.reserve(colors_scalars.size());
  for (unsigned int color_idx = 0; color_idx < colors_scalars.size(); ++color_idx)
    colors.push_back(uchar_bgr_color_to_ros_rgba_color
                     (colors_scalars[color_idx][0],
                     colors_scalars[color_idx][1],
        colors_scalars[color_idx][2]));
  return ret;
} // end pixel2world_rgb_color_rgba()

} // end namespace vision_utils

#endif // PIXEL2WORLD_RGB_H
