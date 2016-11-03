/*!
  \file        convert_float_to_uchar.h
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

#ifndef CONVERT_FLOAT_TO_UCHAR_H
#define CONVERT_FLOAT_TO_UCHAR_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>
// vision_utils
#include <vision_utils/compute_alpha_beta.h>
#include <vision_utils/timer.h>

namespace vision_utils {

/*!
  Compresses a float matrix to a uchar one.
 \param src_float
    a float matrix, any number of channels <= 4
 \param dst_uchar (out)
    where the converted uchar matrix will be stored
 \param alpha_trans (out)
    the scaling factor
 \param beta_trans (out)
    the offset factor
 \param src_nan_indices (out)
    will contain the indice of all the points that are equal to NAN.
    It is sorted from smaller to bigger.
    It can be useful if the image is compressed in a lossy way afterwoards.
*/
inline void convert_float_to_uchar(const cv::Mat & src_float, cv::Mat & dst_uchar,
                                   cv::Mat & src_float_clean_buffer,
                                   ScaleFactorType & alpha_trans,
                                   ScaleFactorType & beta_trans,
                                   std::vector<unsigned int>* src_nan_indices = NULL) {
  // ::vision_utils::Timer timer;
  dst_uchar.create(src_float.size(), CV_8UC(src_float.channels()));
  // timer.printTime("create");

  bool store_indices = (src_nan_indices != NULL);
  if (store_indices) { // clear and reserve some big space
    src_nan_indices->clear();
    src_nan_indices->reserve((src_float.cols * src_float.rows * src_float.channels()) / 3);
  }
  cv::Mat* src_float_clean_ptr;

  // find the max value
  float minVal, maxVal;
#if 1 // find at the same time minVal, maxVal and clean NaNs
  // cv::Mat src_float_clean_buffer = src_float.clone();
  src_float.copyTo(src_float_clean_buffer);
  remove_nans_and_minmax<float>(src_float_clean_buffer, minVal, maxVal, NAN_DEPTH);
  src_float_clean_ptr = &src_float_clean_buffer;

#elif 1 // clean NaNs, then find minVal, maxVal with minmax_nans
  // cv::Mat src_float_clean_buffer = src_float.clone();
  src_float.copyTo(src_float_clean_buffer);
  remove_nans<float>(src_float_clean_buffer, NAN_DEPTH);
  min_max_loc_nans(src_float_clean_buffer, minVal, maxVal, NAN_DEPTH);
  src_float_clean_ptr = &src_float_clean_buffer;

#elif 1
  // cv::minMaxLoc() does not work with multi-channel arrays.
  // If you need to find minimum or maximum elements across all the channels,
  // use reshape() first to reinterpret the array as single-channel.
  // reshape(int cn, int rows=0) const :
  //    Changes the 2D matrix’s shape and/or the number of channels without copying the data.
  cv::Mat src_float_reshape = src_float.reshape(1);
  min_max_loc(src_float_reshape, minVal, maxVal);
  src_float_clean_ptr = &src_float; // no cleaning

#elif 1
  // cv::minMaxLoc() does not work with multi-channel arrays.
  // If you need to find minimum or maximum elements across all the channels,
  // use reshape() first to reinterpret the array as single-channel.
  // reshape(int cn, int rows=0) const :
  //    Changes the 2D matrix’s shape and/or the number of channels without copying the data.
  cv::Mat src_float_reshape = src_float.reshape(1);
  // timer.printTime("reshape");
  double minVal_double, maxVal_double;
  cv::minMaxLoc(src_float_reshape, &minVal_double, &maxVal_double);
  minVal = minVal_double;
  maxVal = maxVal_double;
  src_float_clean_ptr = &src_float; // no cleaning

#else // find at the same time minVal, maxVal and store NaNs
  boost::unordered_set<unsigned int> src_nan_indices_set;
  store_nans_and_minmax_set<float>(src_float, minVal, maxVal, src_nan_indices_set);
  src_float_clean_ptr = &src_float; // no cleaning
#endif

  //printf("minVal:%g, maxVal:%g\n", minVal, maxVal);
  // timer.printTime("min_max_loc");
  compute_alpha_beta(minVal, maxVal, alpha_trans, beta_trans);
  // timer.printTime("compute_alpha_beta");

  // convert the image
#if 1 // simple iterations
  //  int rows = dst_uchar.rows;
  //  int values_per_row = dst_uchar.cols * dst_uchar.channels();
  int rows = src_float_clean_ptr->rows;
  int values_per_row = src_float_clean_ptr->cols * src_float_clean_ptr->channels();
  if (src_float_clean_ptr->isContinuous() && dst_uchar.isContinuous()) {
    rows = 1;
    values_per_row =src_float_clean_ptr->rows * src_float_clean_ptr->cols * src_float_clean_ptr->channels();
  }
  for (int row = 0; row < rows; ++row) {
    // get the address of row
    const float* src_ptr = src_float_clean_ptr->ptr<float>(row);
    uchar* dst_ptr = dst_uchar.ptr<uchar>(row);
    // change each value
    for (int col = 0; col < values_per_row; ++col) {
#if 0 // use src_nan_indices_set
      if (src_nan_indices_set.count(col + row * values_per_row))
        *dst_ptr = NAN_UCHAR;
      else
#endif
        // convert float distance to uchar
        *dst_ptr = dist_to_image_val(*src_ptr, alpha_trans, beta_trans);
      // store the indice if we found a NaN
      if (store_indices && *dst_ptr == NAN_UCHAR)
        src_nan_indices->push_back(col + row * values_per_row);
      ++src_ptr;
      ++dst_ptr;
    } // end loop col
  } // end loop row

#elif 1 // use the evil ptr<>() and iterate, does not work for non-continuous?
  // IplImage src_float_ipl = src_float, dst_uchar_ipl = dst_uchar;
  int nvals = dst_uchar.cols * dst_uchar.rows * dst_uchar.channels();
  const float* src_ptr = src_float_clean_ptr->ptr<float>();
  uchar* dst_ptr = dst_uchar.ptr<uchar>();
  for (int col = 0; col < nvals; ++col) {
    // convert float distance to uchar
    *dst_ptr = dist_to_image_val(*src_ptr, alpha_trans, beta_trans);
    // store the indice if we found a NaN
    if (store_indices && *dst_ptr == NAN_UCHAR)
      src_nan_indices->push_back(col);
    ++src_ptr;
    ++dst_ptr;
  } // end loop col

#elif 1
  cv::convertScaleAbs(*src_float_clean_buffer, dst_uchar, alpha_trans, beta_trans);

#elif 1
  // a bit faster than cv::convertScaleAbs()
  dst_uchar = alpha_trans * (*src_float_clean_buffer) + beta_trans;

#elif 1
  // use CV convertTo() -> very slow!
  cv::Mat dst_uchar_reshape;
  src_float_reshape.convertTo(dst_uchar_reshape, CV_8U, alpha_trans, beta_trans);
  // cv::Mat mask_reshape = (src_float_reshape == NAN_DEPTH);
  //  printf("mask_reshape: %i x %i (%i channels, type:%i, step1:%i)\n",
  //         mask_reshape.cols, mask_reshape.rows, mask_reshape.channels(), mask_reshape.type(), mask_reshape.step1());
  dst_uchar_reshape.setTo(NAN_UCHAR, /*mask_reshape*/ (src_float_reshape == NAN_DEPTH));
  dst_uchar = dst_uchar_reshape.reshape(src_float.channels());
#endif
  // timer.printTime("after cv::convertScaleAbs()");
} // end convert_float_to_uchar();

} // end namespace vision_utils

#endif // CONVERT_FLOAT_TO_UCHAR_H
