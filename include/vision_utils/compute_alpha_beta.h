/*!
  \file        compute_alpha_beta.h
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

#ifndef COMPUTE_ALPHA_BETA_H
#define COMPUTE_ALPHA_BETA_H

#include <vision_utils/nan_handling.h>

namespace vision_utils {

typedef double ScaleFactorType;

////////////////////////////////////////////////////////////////////////////////

/*! compute the transform :
   dest(i, j) = alpha * src(i, j) + beta
   find alpha, beta.
   NAN_VALUE = 0, so we want
   { min_cv_img_reshape -> 1
   { max_cv_img_reshape -> 255
   \arg alpha, beta
      computed values
*/
inline void compute_alpha_beta(const double & minVal, const double & maxVal,
                               ScaleFactorType & alpha, ScaleFactorType & beta) {
  if (maxVal == minVal)
    alpha = 1;
  else
    alpha = 254 / (maxVal - minVal);
  beta = 255 - alpha * maxVal;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param distance
    the value of the float image
 \param alpha_trans
    the scaling factor, obtained with convert_float_to_uchar()
 \param beta_trans
    the offset factor, obtained with convert_float_to_uchar()
 \return uchar
    the converted value, as an uchar
*/
inline uchar dist_to_image_val(const float & distance,
                               const ScaleFactorType & alpha_trans,
                               const ScaleFactorType & beta_trans) {
  return (is_nan_depth(distance) ?
            NAN_UCHAR
          : cv::saturate_cast<uchar>(alpha_trans * distance + beta_trans));
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param val
    the uchar value, as an uchar
 \param alpha_trans
    the scaling factor, obtained with convert_float_to_uchar()
 \param beta_trans
    the offset factor, obtained with convert_float_to_uchar()
 \return float
    the value of the float image
*/
inline float image_val_to_dist(const uchar & val,
                               const ScaleFactorType & alpha_trans,
                               const ScaleFactorType & beta_trans) {
  return (val == NAN_UCHAR ? NAN_DEPTH
                           : 1.f * (val - beta_trans) / alpha_trans);
}
} // end namespace vision_utils

#endif // COMPUTE_ALPHA_BETA_H
