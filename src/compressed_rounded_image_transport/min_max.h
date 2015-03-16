/*!
  \file        min_max.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
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

\todo Description of the file

 */

#ifndef MIN_MAX_H
#define MIN_MAX_H

#include <opencv2/core/core.hpp>

/*!
  Simultaneous min & max using only 3*N/2 comparisons

  "Introduction to Algorithms" by Cormen, Leiserson, Rivest
  pp. 186,187 - ISBN: 0-07-013143-0

  Written by Dann Corbit 9/25/2007, Donated to the public domain

  Some explainations here:
  http://prpds.blogspot.com.es/2011/07/simultaneous-minimum-and-maximum.html
  \param a
    the input data.
    It can contain some NaNs. They will be discarded in that case.
 \param arr_size
    the size of a
 \param min_e, max_e
    output data: the min and max values.
 \param init_check_nans
    should be set to "true" if array contains NaNs.
    Makes a careful initialization of the variables
    to avoid returning NaNs.
 */
template < class _T > // works with stl string class etc...
inline void minmax(const _T * a, // input array
                   size_t arr_size, // array length
                   _T * min_e, // smallest thing found
                   _T * max_e, // biggest thing found
                   bool init_check_nans = false) {
  _T min_et;
  _T max_et;
  size_t i, n_start;
  if (init_check_nans) {
    if (arr_size % 2) { // arr size odd
      min_et = (isnan(a[0]) ? std::numeric_limits<_T>::max() : a[0]);
      max_et = (isnan(a[0]) ? -std::numeric_limits<_T>::max() : a[0]);
      n_start = 1;
    } else { // arr size even (multiple of 2) -> check it as others
      min_et = std::numeric_limits<_T>::max();
      max_et = -std::numeric_limits<_T>::max();
      n_start = 0;
    }
  } // end init_check_nans
  else { // smart init
    if (arr_size % 2) { // arr size odd
      min_et = a[0];
      max_et = a[0];
      n_start = 1;
    } else { // arr size even (multiple of 2)
      if (a[0] > a[1]) {
        max_et = a[0];
        min_et = a[1];
      } else {
        min_et = a[0];
        max_et = a[1];
      }
      n_start = 2;
    } // end if arr_size even
  } // end smart init
  // printf("min_et:%g, max_et:%g\n", min_et, max_et);

  for (i = n_start; i < arr_size; i += 2) {
    // a comparison with NaN always return false
#if 0 // inline if (ternary if) operator - http://en.wikipedia.org/wiki/%3F:
    if (a[i] > a[i + 1]) {
      max_et = (max_et < a[i] ? a[i] : max_et);
      min_et = (min_et > a[i + 1] ? a[i + 1] : min_et);
    } else {
      max_et = (max_et < a[i + 1] ?  a[i + 1] : max_et);
      min_et = (min_et > a[i] ? a[i] : min_et);
    }
#else
    if (a[i] > a[i + 1]) {
      if (max_et < a[i])          max_et = a[i];
      if (min_et > a[i + 1])      min_et = a[i + 1];
    } else {
      if (max_et < a[i + 1])      max_et = a[i + 1];
      if (min_et > a[i])          min_et = a[i];
    }
#endif
  } // end for i
  *min_e = min_et;
  *max_e = max_et;
} // end minmax()

////////////////////////////////////////////////////////////////////////////////

/*!
 * A more convenient version of minmax() for Matrices of OpenCV.
 * It proved to be 30% faster than cv::minMaxLoc().
 * \param float_in
 *    the input array
 * \param minVal, maxVal
 *    the respective minimals and maximals of the matrix.
 */
template<class _T>
inline void min_max_loc(const cv::Mat & float_in, _T & minVal, _T & maxVal) {
#if 1
  minmax(float_in.ptr<_T>(), float_in.cols * float_in.rows * float_in.channels(),
         &minVal, &maxVal, true);
#elif 1
  minVal = std::numeric_limits<double>::max();
  maxVal = -std::numeric_limits<double>::max();
  unsigned int n_values = float_in.cols * float_in.rows;
  const float* float_in_data = float_in.ptr<float>(0);
  for (unsigned int i = 0; i < n_values; ++i) {
    if (*float_in_data < minVal)
      minVal = *float_in_data;
    else if (*float_in_data > maxVal)
      maxVal = *float_in_data;
    ++float_in_data;
  } // end loop i
#endif
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * A version of minmax(). Does the same job (finding min/max in an array),
 * while discarding a specific value, that can represent a NaN value.
 * It can be seen as using a mask on the input array.
 * \param a, arr_size, min_e, max_e
 *    \see minmax().
 * \param NAN_VALUE
 *    the NaN representation.
 */
template < class _T > // works with stl string class etc...
inline void minmax_nans(const _T * a, // input array
                        size_t arr_size, // array length
                        _T * min_e, // smallest thing found
                        _T * max_e, // biggest thing found
                        const _T NAN_VALUE) {
  _T min_et = std::numeric_limits<_T>::max();
  _T max_et = -std::numeric_limits<_T>::max();
  size_t i, n_start;
  if (arr_size % 2) { // arr size odd
    if (a[0] != NAN_VALUE)
      max_et = min_et = a[0];
    n_start = 1;
  } else { // arr size even (multiple of 2) -> check it as others
    n_start = 0;
  }
  // printf("min_et:%g, max_et:%g\n", min_et, max_et);

  for (i = n_start; i < arr_size; i += 2) {
    // check NAN_VALUE cases
    if (a[i] == NAN_VALUE && a[i + 1] == NAN_VALUE) {
      continue;
    }
    else if (a[i] == NAN_VALUE) {
      if (max_et < a[i + 1])      max_et = a[i + 1];
      else if (min_et > a[i + 1]) min_et = a[i + 1];
      continue;
    }
    else if (a[i + 1] == NAN_VALUE) {
      if (max_et < a[i])       max_et = a[i];
      else if (min_et > a[i])  min_et = a[i];
      continue;
    }

    // from here, nobody is NAN_VALUE
#if 0 // inline if (ternary if) operator - http://en.wikipedia.org/wiki/%3F:
    if (a[i] > a[i + 1]) {
      max_et = (max_et < a[i] ? a[i] : max_et);
      min_et = (min_et > a[i + 1] ? a[i + 1] : min_et);
    } else {
      max_et = (max_et < a[i + 1] ?  a[i + 1] : max_et);
      min_et = (min_et > a[i] ? a[i] : min_et);
    }
#else
    if (a[i] > a[i + 1]) {
      if (max_et < a[i])          max_et = a[i];
      if (min_et > a[i + 1])      min_et = a[i + 1];
    } else {
      if (max_et < a[i + 1])      max_et = a[i + 1];
      if (min_et > a[i])          min_et = a[i];
    }
#endif
  } // end for i
  *min_e = min_et;
  *max_e = max_et;
} // end minmax()

////////////////////////////////////////////////////////////////////////////////

/*!
 * A more convenient version of minmax_nans() for Matrices of OpenCV.
 * It proved to be 30% faster than cv::minMaxLoc().
 * \param float_in
 *    the input array
 * \param minVal, maxVal
 *    the respective minimals and maximals of the matrix.
 * \param NAN_VALUE
 *    cf minmax_nans()
 */
template<class _T>
inline void min_max_loc_nans(const cv::Mat & float_in, _T & minVal, _T & maxVal,
                             const _T & NAN_VALUE) {
  minmax_nans(float_in.ptr<_T>(), float_in.cols * float_in.rows * float_in.channels(),
              &minVal, &maxVal, NAN_VALUE);
}

#endif // MIN_MAX_H
