/*!
  \file        get_all_different_values.h
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

#ifndef GET_ALL_DIFFERENT_VALUES_H
#define GET_ALL_DIFFERENT_VALUES_H
// std includes
#include <opencv2/core/core.hpp>
#include <vector>

namespace vision_utils {

/*!
 * Find all different values of an image and put them into a vector.
 * There is no repetition in that vector
 * \param img
 * \param out
 *    the vector containing all values of img with no repetition
 * \param mask
 *    optionnaly, a mask to apply on the image
 */
template<class _T>
inline void get_all_different_values(const cv::Mat_<_T> & img,
                                     std::set<_T> & out,
                                     bool ignore_zeros = true,
                                     const cv::Mat1b & mask = cv::Mat1b()) {
  assert(img.isContinuous());
  // determine if use mask
  bool use_mask = (!mask.empty());
  const uchar* mask_value = (use_mask ? mask.ptr<uchar>() : NULL);
  // iterate on image
  out.clear();
  const _T* img_value = (const _T*) img.ptr();
  unsigned int n_pixels = img.cols * img.rows;
  for (unsigned int pixel_idx = 0; pixel_idx < n_pixels; ++pixel_idx) {
    if ((!ignore_zeros || *img_value) && (!use_mask || *mask_value++))
      out.insert(*img_value);
    ++img_value;
  } // end loop pixel_idx
} // end get_all_different_values()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Find all different values of an image and put them into a vector.
 * There is no repetition in that vector
 * \param img
 * \param out
 *    the vector containing all values of img with no repetition
 * \param mask
 *    optionnaly, a mask to apply on the image
 */
template<class _T>
inline void get_all_different_values(const cv::Mat_<_T> & img,
                                     std::vector<_T> & out,
                                     bool ignore_zeros = true,
                                     const cv::Mat1b & mask = cv::Mat1b()) {
  std::set<_T> img_labels;
  get_all_different_values(img, img_labels, ignore_zeros, mask);
  // copy set to vector
  // cf http://stackoverflow.com/questions/5034211/c-copy-set-to-vector
  out.clear();
  out.reserve(img_labels.size());
  std::copy(img_labels.begin(), img_labels.end(), std::back_inserter(out));
} // end get_all_different_values()

} // end namespace vision_utils

#endif // GET_ALL_DIFFERENT_VALUES_H
