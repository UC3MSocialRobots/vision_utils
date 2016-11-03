/*!
  \file        rotate_image.h
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

#ifndef ROTATE_IMAGE_H
#define ROTATE_IMAGE_H
// std includes
#include <opencv2/imgproc/imgproc.hpp>

#define RAD2DEG     57.2957795130823208768  //!< to convert radians to degrees
#define DEG2RAD     0.01745329251994329577  //!< to convert degrees to radians

namespace vision_utils {

/*!
  Roate an image by a given angle.
 \param src
    The image to be rotated
 \param dst
    The resulting rotated image
 \param angle_rad
    The angle of rotation, in radians
 \param rotation_center
    The center of rotation. Must be in "src" bounds.
 \param flags, borderMode, borderValue
    \see cv::warpAffine()
    http://docs.opencv.org/modules/imgproc/doc/geometric_transformations.html?highlight=warpaffine#warpaffine
*/
inline void rotate_image(const cv::Mat & src, cv::Mat & dst,
                         const double & angle_rad, const cv::Point & rotation_center,
                         int flags=cv::INTER_LINEAR,
                         int borderMode=cv::BORDER_CONSTANT,
                         const cv::Scalar& borderValue=cv::Scalar()) {
  cv::Mat rot_mat = cv::getRotationMatrix2D
                    (rotation_center, angle_rad * RAD2DEG, 1.0);
  cv::warpAffine(src, dst, rot_mat, src.size(), flags, borderMode, borderValue);
} // end rotate_image();

} // end namespace vision_utils

#endif // ROTATE_IMAGE_H
