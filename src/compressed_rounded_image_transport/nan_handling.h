/*!
  \file        nan_handling.h
  <\author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/15

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

Some functions to remove or store NaNs in images.
 */

#ifndef NAN_HANDLING_H
#define NAN_HANDLING_H

#include <boost/unordered_set.hpp>
#include <opencv2/core/core.hpp>

namespace image_utils {

//! what will symbolize NaN for an uchar image
static const uchar NAN_UCHAR = 0;
/*!
 * How we represent NaN in depth images.
 * We do not use std::numeric_limits<float>::quiet_NaN(),
 * as its presence in images considerably slows down their processing.
 */
static const float NAN_DEPTH = 0;

////////////////////////////////////////////////////////////////////////////////

//! \return \true if distance is a NAN (or a simulated one)
inline bool is_nan_depth(const float & distance) {
  return (distance == NAN_DEPTH || isnan(distance));
}

////////////////////////////////////////////////////////////////////////////////

/*! remove all NaN in a matrix
  Template<_T>: the basic type of the mat, ex float for cv::Mat3f
 \param img
    the image to clean
 \param new_val
    what to put instead of the NaN
*/
template<class _BasicType>
inline void remove_nans(cv::Mat & img, _BasicType new_val = (_BasicType)(0)) {
  int values_per_row = img.cols * img.channels();
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    _BasicType* src_ptr = img.ptr<_BasicType>(row);
    // change each value
    for (int col = 0; col < values_per_row; ++col) {
      if (is_nan_depth(*src_ptr))
        *src_ptr = new_val;
      ++src_ptr;
    } // end loop col
  } // end loop row
} // end remove_nans()

////////////////////////////////////////////////////////////////////////////////

/*! remove all NaN in a matrix, and perform a min/max search.
  Template<_T>: the basic type of the mat, ex float for cv::Mat3f
 \param img
    the image to clean
 \param min_val, max_val
    output: the min and max values of img (NaN excluded of course)
 \param new_val
    what to put instead of the NaN
*/
template<class _BasicType>
inline void remove_nans_and_minmax(cv::Mat & img,
                                   _BasicType & min_val, _BasicType & max_val,
                                   _BasicType new_val = (_BasicType)(0)) {
  int nrows = img.rows, values_per_row = img.cols * img.channels();
  bool minmax_were_set = false;
  min_val = max_val = new_val;
  for (int row = 0; row < nrows; ++row) {
    // get the address of row
    _BasicType* src_ptr = (_BasicType*) img.ptr(row);
    // change each value
    for (int col = 0; col < values_per_row; ++col) {
      // printf("remove_nans_and_minmax: (%i,%i): %g\n", col, row, *src_ptr);
      if (is_nan_depth(*src_ptr))
        *src_ptr = new_val;
      else if (!minmax_were_set) {
        min_val = max_val = *src_ptr;
        minmax_were_set = true;
      }
      else if (min_val > *src_ptr)
        min_val = *src_ptr;
      else if (max_val < *src_ptr)
        max_val = *src_ptr;
      ++src_ptr;
    } // end loop col
  } // end loop row
} // end remove_nans_and_minmax()

////////////////////////////////////////////////////////////////////////////////

/*! store all NaN in a matrix, and perform a min/max search.
  Template<_T>: the basic type of the mat, ex float for cv::Mat3f
 \param img
    the image to clean
 \param min_val, max_val
    output: the min and max values of img (NaN excluded of course)
 \param new_val
    what to put instead of the NaN
*/
template<class _BasicType>
inline void store_nans_and_minmax_set(const cv::Mat & img,
                                      _BasicType & min_val, _BasicType & max_val,
                                      boost::unordered_set<unsigned int> & src_nan_indices) {
  int values_per_row = img.cols * img.channels();
  min_val = std::numeric_limits<_BasicType>::max();
  max_val = -std::numeric_limits<_BasicType>::max();
  src_nan_indices.clear();
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const _BasicType* src_ptr = img.ptr<_BasicType>(row);
    // change each value
    for (int col = 0; col < values_per_row; ++col) {
      if (is_nan_depth(*src_ptr))
        src_nan_indices.insert(col + row * values_per_row);
      else if (min_val > *src_ptr)
        min_val = *src_ptr;
      else if (max_val < *src_ptr)
        max_val = *src_ptr;
      ++src_ptr;
    } // end loop col
  } // end loop row
} // end store_nans_and_minmax_set()

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
bool is_nan_pt(const Pt3 & pt) {
  return (is_nan_depth(pt.x) || is_nan_depth(pt.y) || is_nan_depth(pt.z));
}

} // end namespace image_utils

#endif // NAN_HANDLING_H
