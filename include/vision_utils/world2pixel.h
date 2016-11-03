/*!
  \file        world2pixel.h
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

#ifndef WORLD2PIXEL_H
#define WORLD2PIXEL_H
// std includes
#include <opencv2/core/core.hpp>

namespace vision_utils {

template<class Pt2>
inline Pt2 world2pixel
(const cv::Point3d & pt,
 const image_geometry::PinholeCameraModel & depth_or_rgb_cam_model)
{
  cv::Point2d ans_cv = world2pixel<cv::Point2d>(pt, depth_or_rgb_cam_model);
  Pt2 ans;
  ans.x = ans_cv.x;
  ans.y = ans_cv.y;
  return ans;
} // end world2pixel()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Converts a 2D point into a pixel.
 * The coordinates of this pixel will be in the RGB image
 * if the RGB cam model is given (same thing for depth).
 * \param pt
 *  The 3D point, with the following frame
 *  (/<robot>_camera_rgb_optical_frame = /<robot>_camera_depth_optical_frame).
 *  They correspond to the frame_id that comes with the 2D images topics.
 *
 *  ( Careful, the frames /<robot>_camera_rgb_frame = /<robot>_camera_depth_frame )
 *  ( are different. Needs a TF conversion for these.                             )

<scene here>

   ^ z
    \
     +---> x
     |
     |
   y V

 * \param depth_or_rgb_cam_model
 *    The camera model corresponding to the pixel coordinates we want
 * \return rgb the point in the 2D RGB image, classic image frame
     +--------> x
     |
     |
   y V

 */
template<>
inline cv::Point2d world2pixel(const cv::Point3d & pt,
                               const image_geometry::PinholeCameraModel & depth_or_rgb_cam_model) {
  return depth_or_rgb_cam_model.project3dToPixel(pt);
}

} // end namespace vision_utils

#endif // WORLD2PIXEL_H
