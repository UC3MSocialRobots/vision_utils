/*!
  \file        convert_uchar_to_float.h
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

#ifndef CONVERT_UCHAR_TO_FLOAT_H
#define CONVERT_UCHAR_TO_FLOAT_H
// std includes
#include <opencv2/core/core.hpp>
#include <vector>
#include "vision_utils/is_nan_depth.h"
#include "vision_utils/compute_alpha_beta.h"

namespace vision_utils {

/*!
  Restores the compressed rounded image to the approximate float image
 \param src_uchar
    the compressed image
 \param dst_float (out)
    the approximate float image
 \param alpha_trans
    the scaling factor, obtained with convert_float_to_uchar()
 \param beta_trans
    the offset factor, obtained with convert_float_to_uchar()
 \param src_nan_indices
    should contain the indice of all the points that are equal to NaN.
    It should be sorted from smaller to bigger.
    It can be useful to restore the NaNs
    if the image was compressed in a lossy way afterwoards.
*/
inline void convert_uchar_to_float(const cv::Mat & src_uchar, cv::Mat & dst_float,
                                   const ScaleFactorType & alpha_trans,
                                   const ScaleFactorType & beta_trans,
                                   const std::vector<unsigned int>* src_nan_indices = NULL) {
  dst_float.create(src_uchar.size(), CV_32FC(src_uchar.channels()));
  // parameters for NaN restoring
  bool src_nan_indices_given = (src_nan_indices != NULL);
  unsigned int src_nan_indices_pos = 0, curr_pos_total = 0;
  const unsigned int* src_nan_current_indice =
      (src_nan_indices_given ? &(*src_nan_indices)[0] : NULL);
  //printf("src_nan_indices_given:%i", src_nan_indices_given);

  // make a lookup table for faster conversion
  float lookup_table[256];
  for (int col = 0; col <= 255; ++col)
    lookup_table[col] = image_val_to_dist(col, alpha_trans, beta_trans);

  // convert the image
  // int values_per_row = src_uchar.cols * src_uchar.channels();
  int rows = src_uchar.rows;
  int values_per_row = src_uchar.cols * src_uchar.channels();
  if (src_uchar.isContinuous() && dst_float.isContinuous()) {
    rows = 1;
    values_per_row =src_uchar.rows * src_uchar.cols * src_uchar.channels();
  }
  for (int row = 0; row < rows; ++row) {
    // get the address of row
    const uchar* src_ptr = src_uchar.ptr<uchar>(row);
    float* dst_ptr = dst_float.ptr<float>(row);
    // change each value
    for (int col = 0; col < values_per_row; ++col) {
      // restore NaN if needed
      if (src_nan_indices_given && curr_pos_total == *src_nan_current_indice) {
        *dst_ptr = NAN_DEPTH;
        if (src_nan_indices_pos < src_nan_indices->size() - 1)
          ++src_nan_current_indice;
      }
      else {
        //*dst_ptr = image_val_to_dist(*src_ptr, alpha_trans, beta_trans);
        *dst_ptr = lookup_table[*src_ptr];
      }
      ++src_ptr;
      ++dst_ptr;
      if (src_nan_indices_given)
        ++ curr_pos_total;
    } // end loop col
  } // end loop row
} // end convert_uchar_to_float();

} // end namespace vision_utils

#endif // CONVERT_UCHAR_TO_FLOAT_H
