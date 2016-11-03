/*!
  \file        get_all_non_null_values_and_com.h
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

#ifndef GET_ALL_NON_NULL_VALUES_AND_COM_H
#define GET_ALL_NON_NULL_VALUES_AND_COM_H
// std includes
#include <map>
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>
#include "vision_utils/region_growth.h"
#include "vision_utils/infosimage.h"

namespace vision_utils {

template<class _T>
inline bool get_all_non_null_values_and_com(const cv::Mat_<_T> & img,
                                            std::map<_T, cv::Point> & out,
                                            bool ignore_zeros = true,
                                            bool ensure_com_on_mask = false,
                                            const cv::Mat1b & mask = cv::Mat1b()) {
  if (img.empty()) {
    out.clear();
    return true;
  }
  assert(img.isContinuous());
  typedef typename std::map<_T, cv::Point3i> ComsMap;
  ComsMap coms;
  // determine if use mask
  bool use_mask = (!mask.empty());
  const uchar* mask_value = (use_mask ? mask.ptr<uchar>() : NULL);
  // iterate on image
  int cols = img.cols, rows = img.rows;
  out.clear();
  for (int row = 0; row < rows; ++row) {
    // get the address of row
    const _T* imgdata = img.ptr(row);
    for (int col = 0; col < cols; ++col) {
      if ((!ignore_zeros || imgdata[col]) && (!use_mask || *mask_value++)) {
        typename ComsMap::iterator coms_pos = coms.find(imgdata[col]);
        if (coms_pos == coms.end())
          coms.insert(coms_pos,
                      std::pair<_T, cv::Point3i>(imgdata[col], cv::Point3i(col, row, 1)));
        else {
          coms_pos->second.x += col;
          coms_pos->second.y += row;
          ++coms_pos->second.z;
        }
      } // end if if ((!use_mask || *mask_value++) && data[col])
    } // end loop col
  } // end loop row

  // convert coms -> out
  out.clear();
  typename ComsMap::const_iterator coms_it = coms.begin();
  ClosestPointInMask cl;
  while (coms_it != coms.end()) { // normalize by the number of points
    _T user_idx = coms_it->first;
    cv::Point com(1.f * coms_it->second.x / coms_it->second.z,
                  1.f * coms_it->second.y / coms_it->second.z);
    if (ensure_com_on_mask) // find closest poiint on mask
      com = cl.find(img, com, user_idx, user_idx);
    if (!bbox_full(img).contains(com)) {
      printf("com %s outside of img '%s'\n",
             printP2(com).c_str(), infosImage(img).c_str());
      return false;
    }
    out.insert(std::pair<_T, cv::Point>(user_idx, com));
    ++coms_it;
  }
  return true;
} // end get_all_different_values()

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline bool get_all_non_null_values_and_com_fast(const cv::Mat_<_T> & img,
                                                 std::map<_T, cv::Point> & out,
                                                 bool ignore_zeros = true,
                                                 bool ensure_com_on_mask = false,
                                                 const cv::Mat1b & mask = cv::Mat1b(),
                                                 int maxval = 255) {
  if (img.empty()) {
    out.clear();
    return true;
  }
  assert(img.isContinuous());
  std::vector<cv::Point3i> coms(maxval + 1, cv::Point3i(0, 0, 0));
  // determine if use mask
  bool use_mask = (!mask.empty());
  const uchar* mask_value = (use_mask ? mask.ptr<uchar>() : NULL);
  // iterate on image
  int cols = img.cols, rows = img.rows;
  for (int row = 0; row < rows; ++row) {
    // get the address of row
    const _T* imgdata = img.ptr(row);
    for (int col = 0; col < cols; ++col) {
      if ((!ignore_zeros || imgdata[col]) && (!use_mask || *mask_value++)) {
        cv::Point3i* curr_pt = &(coms[imgdata[col]]);
        curr_pt->x += col;
        curr_pt->y += row;
        ++curr_pt->z;
      } // end if if ((!use_mask || *mask_value++) && data[col])
    } // end loop col
  } // end loop row

  // convert coms -> out
  out.clear();
  ClosestPointInMask cl;
  // if user_idx of type _T, infinite loop with _T=uchar and maxval = 255
  int max_val_int = maxval;
  for (int user_idx = 0; user_idx <= max_val_int; ++user_idx) {
    if (!coms[user_idx].z)
      continue;
    cv::Point3i* curr_pt = &(coms[user_idx]);
    cv::Point com(1.f * curr_pt->x / curr_pt->z,
                  1.f * curr_pt->y / curr_pt->z);
    if (ensure_com_on_mask) // find closest poiint on mask
      com = cl.find(img, com, (_T) user_idx, (_T) user_idx);
    if (!bbox_full(img).contains(com)) {
      printf("com %s outside of img '%s'\n",
             printP2(com).c_str(), infosImage(img).c_str());
      return false;
    }
    out.insert(std::pair<_T, cv::Point>(user_idx, com));
  } // end loop i
  return true;
} // end get_all_different_values()

} // end namespace vision_utils

#endif // GET_ALL_NON_NULL_VALUES_AND_COM_H
