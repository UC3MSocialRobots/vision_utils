/*!
  \file        redimContent.h
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

#ifndef REDIMCONTENT_H
#define REDIMCONTENT_H
// std includes
#include <opencv2/core/core.hpp>
#include <vector>
#include <vision_utils/bboxes_included.h>
#include <vision_utils/bbox_full.h>
#include <vision_utils/boundingBox.h>
#include <vision_utils/boundingBox_vec.h>
#include <vision_utils/resizeMonochromeImage.h>

namespace vision_utils {

/*!
 *\brief   fit the non nul bounding box of src to fit to the size of dest
 *\param   src monochrome image
 *\param   dest monochrome image
 */
inline bool redimContent(const cv::Mat1b & src, cv::Mat1b & dest) {
  //printf("redimContent('%s', '%s'')", infosImage(src).c_str(), infosImage(dest).c_str());
  //cout << "bbox 1:" << cvGetImageROI(src).cols << "," << cvGetImageROI(src).rows << endl;
  //    cv::Rect bbox_orig = cvGetImageROI(src);
  //    //cout << "bbox res:" << bbox.cols << "," << bbox.rows << endl;
  //    cvSetImageROI(src, boundingBox(src));
  //    resizeMonochromeImage(src, dest);
  //    cvResetImageROI(src);
  //    cvSetImageROI(src, bbox_orig);

  cv::Rect bbox_src = boundingBox(src);
  if (!bboxes_included(bbox_full(src), bbox_src))
    return false;
  resizeMonochromeImage(src(bbox_src), dest);
  return true;
}

////////////////////////////////////////////////////////////////////////////

#define USE_MAT1B
#ifdef USE_MAT1B
typedef uchar MyBool;
#else // USE_MAT1B
typedef bool MyBool;
#endif // USE_MAT1B

template<class Pt2Iterable>
inline void redimContent_vector_without_repetition_given_src_bbox(
    const Pt2Iterable & src,
    Pt2Iterable & rep,
    const cv::Rect & src_bbox,
    const cv::Rect & dst_bbox,
    MyBool* resized_array_ptr,
    bool keep_ratio = false) {
  assert(src != rep);
  //printf("redimContent_vector_without_repetition_given_src_bbox"
//               "(bbox_src:%s, dst_bbox:%s, keep_ratio:%i)",
//               rectangle_to_string(src_bbox).c_str(),
//               rectangle_to_string(dst_bbox).c_str(), keep_ratio);

  /* compute the transformation
          { x' = aX * x + bX
          { y' = aY * y + bY
           with (src_bbox.x, src_bbox.y) matching to (0, 0)
           and  (src_bbox.width, src_bbox.height) matching
                   to (dst_bbox.width, dst_bbox.height)
    */
  float aX = (src_bbox.width != 1 ?
                                  1. * dst_bbox.width / src_bbox.width :
                                  0);
  float aY = (src_bbox.height != 1 ?
                                   1. * dst_bbox.height / src_bbox.height :
                                   0);
  if (keep_ratio) {
    if (fabs(aX) < fabs(aY)) // adjust Y : y' = aX * y + bY
      aY = aX;
    else // adjust x : x' = aY * x + bX
      aX = aY;
  } // end if keep_ratio

  // this is true, keep_ratio or not
  float bX = - aX * src_bbox.x;
  float bY = - aY * src_bbox.y;
  //printf("x' = %g x + %g,  y' = %g y + %g", aX, bX, aY, bY);

  /* stock the results in an array */
  /* first init the array */
  unsigned int npts = src.size(), dstcols = dst_bbox.width, dstrows = dst_bbox.height;
  bzero(resized_array_ptr, sizeof(MyBool) * dstcols * dstrows);

  /* then mark the present points */
  int nb_points = 0;
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    unsigned int index = dstcols
                         *(int) (aY * src[pt_idx].y + bY) // y
                         + (int) (aX * src[pt_idx].x + bX); // x
    MyBool* resized_array_pos_ptr = &(resized_array_ptr[index]);
    if (! *resized_array_pos_ptr) { // not initialized before
      ++nb_points;
      *resized_array_pos_ptr = true;
    }
  } // end loop pt_idx
  //printf("nb_points:%i", nb_points);

  /* recopy the points marked in the array in the std::vector */
  rep.clear();
  rep.resize(nb_points);
  int rep_idx = 0;
  MyBool* resized_array_pos_ptr = &resized_array_ptr[0];
  for (unsigned int y = 0; y < dstrows; ++y) {
    for (unsigned int x = 0; x < dstcols; ++x) {
      if (*resized_array_pos_ptr++) {
        rep[rep_idx].x = dst_bbox.x + x;
        rep[rep_idx].y = dst_bbox.y + y;
        ++rep_idx;
      }
    } // end loop x
  } // end loop y

  //printf("rep_size:%i", rep.size());
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   fit a vector of points to a bounding box without repetition.
 * src and rep have to be different
 * resized_array_ptr needs to be allocated beforehand to (dst_bbox.height * dst_bbox.width)
 */
template<class Pt2Iterable>
inline void redimContent_vector_without_repetition_given_resized_array(
    const Pt2Iterable & src,
    Pt2Iterable & rep,
    const cv::Rect & dst_bbox,
    MyBool* resized_array_ptr,
    bool keep_ratio = false) {

  //printf("redimContent_vector_without_repetition_given_resized_array"
//               "(dst_bbox:%s, keep_ratio:%i)",
//               rectangle_to_string(dst_bbox).c_str(), keep_ratio);
  cv::Rect src_bbox = boundingBox_vec
                      <Pt2Iterable, cv::Rect>(src);
  redimContent_vector_without_repetition_given_src_bbox<Pt2Iterable>
      (src, rep,
       src_bbox,
       dst_bbox,
       resized_array_ptr,
       keep_ratio);
} // end redimContent_vector_without_repetition_given_resized_array()

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   fit a vector of points to a bounding box without repetition.
 *src and rep have to be different
 */
template<class Pt2Iterable>
inline void redimContent_vector_without_repetition(
    const Pt2Iterable & src,
    Pt2Iterable & rep,
    const cv::Rect & dst_bbox,
    bool keep_ratio = false) {

  //printf("redimContent_vector_without_repetition"
//               "(dst_bbox:%s, keep_ratio:%i)",
//               rectangle_to_string(dst_bbox).c_str(), keep_ratio);
  assert(src != rep);
#ifdef USE_MAT1B
  cv::Mat1b resized_array;
  resized_array.create(dst_bbox.height, dst_bbox.width); // ROWS, COLS
#else // USE_MAT1B
  bool resized_array [dst_bbox.width * dst_bbox.height];
#endif // USE_MAT1B

  redimContent_vector_without_repetition_given_resized_array<Pt2Iterable>
      (src, rep,
       dst_bbox,
     #ifdef USE_MAT1B
       resized_array.data,
     #else // USE_MAT1B
       &resized_array[0],
    #endif // USE_MAT1B
      keep_ratio);
  //delete resized_array;
}

} // end namespace vision_utils

#endif // REDIMCONTENT_H
