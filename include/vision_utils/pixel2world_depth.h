/*!
  \file        pixel2world_depth.h
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

#ifndef PIXEL2WORLD_DEPTH_H
#define PIXEL2WORLD_DEPTH_H
// std includes
#include <image_geometry/pinhole_camera_model.h>
#include "vision_utils/is_nan_depth.h"
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

/*!
 * Template version of pixel2world_depth().
 * It calls the non templated version, with cv::Point3d,
 * then converts it.
 */
template<class Pt3>
inline Pt3 pixel2world_depth
(const cv::Point2d & depth_pt,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const double & depth_value)
{
  cv::Point3d ans_cv =
      pixel2world_depth<cv::Point3d>(depth_pt, depth_cam_model, depth_value);
  Pt3 ans;
  ans.x = ans_cv.x;
  ans.y = ans_cv.y;
  ans.z = ans_cv.z;
  return ans;
} // end pixel2world_depth()


////////////////////////////////////////////////////////////////////////////////

/*!
 * The version of pixel2world_depth() needing the least data.
 * It first gets the depth value,
 * then calls the templated version of pixel2world_depth().
 */
template<class Pt3>
inline Pt3 pixel2world_depth
(const cv::Point2d & depth_pt,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const cv::Mat & depth_img) {
  IplImage depth_img_ipl = depth_img;
  if (depth_pt.x < 0 || depth_pt.x >= depth_img.cols
      || depth_pt.y < 0 || depth_pt.y >= depth_img.rows) {
    printf("Depth point (%g,%g) is outside the depth image (%ix%i).\n",
             depth_pt.x, depth_pt.y,
             depth_img.cols, depth_img.rows);
    return Pt3();
  }
  double depth_value = CV_IMAGE_ELEM(&depth_img_ipl, float,
                                     (int) depth_pt.y, (int) depth_pt.x);
  return pixel2world_depth<Pt3>(depth_pt, depth_cam_model, depth_value);
}

////////////////////////////////////////////////////////////////////////////////

static const double NAN_DOUBLE = 0;

/*!
 * The lowest level version, needing the most data.
 * \arg pt_rgb the point in the 2D RGB image, classic image frame
     +--------> x
     |
     |
   y V

   \return pt3d with such (z in direction of the scene):

<scene here>

   ^ z
    \
     +---> x
     |
     |
   y V

 */
template<>
inline cv::Point3d pixel2world_depth
(const cv::Point2d & depth_pt,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const double & depth_value)
{
  if (is_nan_depth(depth_value))
    return cv::Point3d(NAN_DOUBLE, NAN_DOUBLE, NAN_DOUBLE);
  cv::Point3d line_vec = depth_cam_model.projectPixelTo3dRay(depth_pt);
  // double line_vec_norm = norm(line_vec);
  //  if (fabs(line_vec_norm) < 1E-5)
  //    return cv::Point3d(0, 0, 0);
  //    //printf("depth_pt%s, line_vec:%s, line_vec_norm:%g",
  //             printP2(depth_pt).c_str(),
  //             printP(line_vec).c_str(),
  //             line_vec_norm);
  // return line_vec * (depth_value / line_vec_norm);

  if (fabs(line_vec.z) < 1E-5)
    return cv::Point3d(0, 0, 0);
  // return line_vec * (depth_value / line_vec.z);
  return cv::Point3d(line_vec.x * depth_value / line_vec.z,
                     line_vec.y * depth_value / line_vec.z,
                     depth_value);
} // end pixel2world_depth()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Reproject a depth image into a point cloud.
 * \param depth
 *    The depth image to be reprojected.
 * \param depth_cam_model
 *    The model of the camera that enables the reprojection.
 * \param depth_reprojected
 *    The point cloud after reprojection.
 * \param data_step
 *    Use 1 to reproject all pixels of \a depth,
 *    use 2 to reproject 50% of the pixels (every other pixel),
 *    use 3 for every third pixel, etc.
 *    Enables a good speedup while keeping most of the image.
 * \param mask
 *    A mask indicating what needs to be reprojected.
 *    Leave empty to reproject the whole image.
 * \param remove_nans
 *    If true, do not insert into the cloud points that correspond to
 *    an undefined depth (shiny surfaces, etc.)
 */
template<class Pt3>
inline bool pixel2world_depth
(const cv::Mat & depth_img,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 std::vector<Pt3> & depth_reprojected,
 const unsigned int data_step = 1,
 const cv::Mat1b & mask = cv::Mat(),
 bool remove_nans = false)
{
  int rows = depth_img.rows, cols = depth_img.cols;
  if (!depth_cam_model.initialized()) {
    printf("pixel2world_depth(): depth_cam_model not initialized\n");
    return false;
  }

  // dimensions check
  bool use_mask = (!mask.empty());
  if (use_mask && depth_img.size() != mask.size()) {
    printf("pixel2world_depth(): depth_img size (%i, %i) != mask size (%i, %i) -> "
           "no using mask\n", depth_img.cols, depth_img.rows, mask.cols, mask.rows);
    use_mask = false;
  }

  // clear answer
  depth_reprojected.clear();
  depth_reprojected.reserve(cols * rows);
  if (!cols && !rows) {
    printf("pixel2world_depth(): empty image!\n");
    return true;
  }

  // locates matrix header within a parent matrix
  // cf http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  cv::Size size;
  cv::Point offset;
  depth_img.locateROI(size, offset);

  const uchar* mask_ptr;
  for (int row = 0; row < rows; row += data_step) {
    // get the address of row
    const float* depth_ptr = depth_img.ptr<float>(row);
    if (use_mask)
      mask_ptr = mask.ptr<uchar>(row);
    for (int col = 0; col < cols; col += data_step) {
      if (use_mask && !mask_ptr[col])
        continue;
      if (remove_nans && is_nan_depth(depth_ptr[col]))
        continue;
      depth_reprojected.push_back
          ( pixel2world_depth<Pt3>
            (cv::Point2d(col + offset.x, row + offset.y),
             depth_cam_model, depth_ptr[col]) );
    } // end loop col
  } // end loop row
  return true;
} // end pixel2world_depth()

} // end namespace vision_utils

#endif // PIXEL2WORLD_DEPTH_H
