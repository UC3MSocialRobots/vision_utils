/*!
  \file        compute_pixel2meters_factor.h
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

#ifndef COMPUTE_PIXEL2METERS_FACTOR_H
#define COMPUTE_PIXEL2METERS_FACTOR_H
// std includes
#include "image_geometry/pinhole_camera_model.h"
#include <vision_utils/pixel2world_depth.h>
#include <vision_utils/dist3.h>
#include <vision_utils/read_camera_model_files.h>
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>

namespace vision_utils {

/*!
 * \return the factor to convert a distance in pixels into a distance in meters.
 * It is in meters / pixels.
 * Return "nan" if depth_value is nan. The returned value can be determined with isnan().
 * \param depth_size
 * \param depth_cam_model
 * \param depth_value
 */
inline double compute_pixel2meters_factor
(const cv::Size & depth_size,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const double & depth_value)
{
  int seg_size = depth_size.height / 2;
  cv::Point2d p1(depth_size.width / 2, depth_size.height / 2 - seg_size / 2);
  cv::Point2d p2(depth_size.width / 2, depth_size.height / 2 + seg_size / 2);
  cv::Point3d p1_world = pixel2world_depth<cv::Point3d>(p1, depth_cam_model, depth_value);
  cv::Point3d p2_world = pixel2world_depth<cv::Point3d>(p2, depth_cam_model, depth_value);
  return dist3(p1_world, p2_world) / seg_size;
}

////////////////////////////////////////////////////////////////////////////////

//! a short version for the other compute_pixel2meters_factor()
inline double compute_pixel2meters_factor
(const cv::Mat1f & depth_img,
 const image_geometry::PinholeCameraModel & depth_camera_model,
 const cv::Point & depth_pt)
{
  if (depth_pt.x < 0 || depth_pt.x >= depth_img.cols
      || depth_pt.y < 0 || depth_pt.y >= depth_img.rows) {
    printf("Depth point (%i,%i) is outside the depth image (%ix%i).\n",
             depth_pt.x, depth_pt.y,
             depth_img.cols, depth_img.rows);
    return NAN_DOUBLE;
  }
  IplImage depth_img_ipl = depth_img;
  const double depth_value = CV_IMAGE_ELEM(&depth_img_ipl, float,
                                           (int) depth_pt.y, (int) depth_pt.x);
  return compute_pixel2meters_factor
      (depth_img.size(), depth_camera_model, depth_value);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \param depth_img
 *    The depth image coming from a kinect.
 * \param kinect_serial_number
 *    Needed for loading our kinect camera model.
 * \param depth_pt
 *    The point where we want to evaluate the pixel2meters_factor.
 * \return
 *    pixel2meters_factor (in meters/pixels),
 *    or NAN_DOUBLE if error.
 */
inline double compute_pixel2meters_factor(const cv::Mat1f & depth_img,
                                          const std::string & kinect_serial_number,
                                          const cv::Point & depth_pt) {
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  bool ok = read_camera_model_files
            (kinect_serial_number, depth_camera_model, rgb_camera_model);
  if (!ok)
    return NAN_DOUBLE;
  // get pixel2meters_factor
  return compute_pixel2meters_factor
      (depth_img, depth_camera_model, depth_pt);
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline double compute_average_pixel2meters_factor
(const cv::Mat1f & depth_img,
 const _T & depth_camera_model,
 const cv::Mat1b & mask)
{
  if (mask.empty() || depth_img.size() != mask.size()) {
    printf("compute_average_pixel2meters_factor: dimensions of depth_img(%i, %i), "
           "mask(%i, %i) dont match!\n",
           depth_img.cols, depth_img.rows, mask.cols, mask.rows);
    return NAN_DOUBLE;
  }
  // compute median
  unsigned int nvalues = 50;
  std::vector<double> factors;
  factors.reserve(nvalues);
  unsigned int ntries = 0, maxtries = 100 * nvalues;
  while(factors.size() < nvalues && ++ntries < maxtries) {
    // take random point
    cv::Point depth_pt(rand() % depth_img.cols, rand() % depth_img.rows);
    // check it is in mask
    if (!mask(depth_pt))
      continue;
    // get pixel2meters_factor
    double curr_factor = compute_pixel2meters_factor
                         (depth_img, depth_camera_model, depth_pt);
    if (isnan(curr_factor))
      continue;
    factors.push_back(curr_factor);
  }
  if (ntries >= maxtries) {
    printf("compute_average_pixel2meters_factor(): reached the max number of tries, "
           "most probably the given depth image is empty\n");
    return NAN_DOUBLE;
  }

  // find median,
  // http://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation?lq=1
  size_t n = factors.size() / 2;
  std::nth_element(factors.begin(), factors.begin()+n, factors.end());
  return factors[n];
} // end compute_average_pixel2meters_factor()

} // end namespace vision_utils

#endif // COMPUTE_PIXEL2METERS_FACTOR_H
