/*!
  \file        paste_img_compute_rois.h
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

#ifndef PASTE_IMG_COMPUTE_ROIS_H
#define PASTE_IMG_COMPUTE_ROIS_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <string>
#include "vision_utils/rectangle_intersection.h"
#include "vision_utils/bboxes_included.h"
#include "vision_utils/print_rect.h"

namespace vision_utils {

/*!
 Determine the ROIs for pasting an image onto another.
 \param topaste
 \param dst
 \param topaste_x, topaste_y
    The coordinates where to paste "topaste" into "dst"
 \param topaste_roi (out)
   If width = 0, means "topaste" would be out of "dst" (= nothing to do).
   Otherwise, the ROI of "topaste" that needs to be copied.
 \param dst_roi (out)
   If width = 0, means "topaste" would be out of "dst" (= nothing to do).
   Otherwise, the ROI of "dst" where the ROI of "topaste" needs to be pasted.
*/
template<class Image>
inline void paste_img_compute_rois(const Image & topaste, const Image & dst,
                                   int topaste_x, int topaste_y,
                                   cv::Rect & topaste_roi, cv::Rect & dst_roi) {
  dst_roi = cv::Rect(0, 0, dst.cols, dst.rows);
  topaste_roi = cv::Rect(topaste_x, topaste_y, topaste.cols, topaste.rows);

  // sizes check
  if (topaste.cols == 0 || topaste.rows == 0 || dst.rows == 0 || dst.cols == 0) {
    dst_roi.width = dst_roi.height = topaste_roi.width = topaste_roi.height = 0;
    return;
  }

  cv::Rect inter_roi = rectangle_intersection(dst_roi, topaste_roi);
  // disjoint rectangles => do nothing
  if (inter_roi.width <= 0 || inter_roi.height <= 0) {
    dst_roi.width = dst_roi.height = topaste_roi.width = topaste_roi.height = 0;
    return;
  }
  // the corresponding width and height in input images will be the one of the inter
  dst_roi.width = inter_roi.width;
  dst_roi.height = inter_roi.height;
  topaste_roi.width = inter_roi.width;
  topaste_roi.height = inter_roi.height;
  // adjust x, depending on which side of the left edge of "dst" we paste
  if (topaste_x >= 0) { // we paste on the right side of the left edge of "dst"
    dst_roi.x = inter_roi.x;
    topaste_roi.x = 0;
  }
  else { // we paste on the left side of "dst"
    dst_roi.x = 0;
    topaste_roi.x = -topaste_x;
  }
  // same thing with y
  if (topaste_y >= 0) {
    dst_roi.y = inter_roi.y;
    topaste_roi.y = 0;
  }
  else {
    dst_roi.y = 0;
    topaste_roi.y = -topaste_y;
  }
} // end paste_img_compute_rois()

/*! copies to \arg dst
    matrix elements of \arg topaste that are
    marked with non-zero \arg mask elements. */
template<class Image>
inline void paste_img(const Image & topaste, Image & dst,
                      int topaste_x, int topaste_y,
                      const cv::Mat* mask = NULL,
                      const std::string title = "",
                      const cv::Scalar title_color = cv::Scalar(0, 0, 255)) {
  //  printf("paste_img()\n");

  cv::Rect topaste_roi, dst_roi;
  paste_img_compute_rois(topaste, dst, topaste_x, topaste_y, topaste_roi, dst_roi);
  //  printf("topaste_roi:(%ix%i)+(%ix%i), dst_roi:(%ix%i)+(%ix%i)\n",
  //         topaste_roi.x, topaste_roi.y, topaste_roi.width, topaste_roi.height,
  //         dst_roi.x, dst_roi.y, dst_roi.width, dst_roi.height);
  if (topaste_roi.width == 0 || topaste_roi.height == 0
      || dst_roi.width == 0 || dst_roi.height == 0)
    return;

  // make the proper pasting
  Image topaste_sub = topaste(topaste_roi),
      dst_sub = dst(dst_roi);
  bool use_mask = (mask != NULL);
  if (use_mask && !bbox_included_image(topaste_roi, *mask)) {
    printf("paste_img(): incorrect dims of mask (%i,%i), topaste_roi:%s\n",
           mask->cols, mask->rows, print_rect(topaste_roi).c_str());
    use_mask = false;
  }
  if (use_mask) {
    const cv::Mat mask_sub = (*mask)(topaste_roi);
    topaste_sub.copyTo(dst_sub, mask_sub);
  }
  else
    topaste_sub.copyTo(dst_sub);

  // put the title if needed
  if (title != "")
    cv::putText(dst, title,
                cv::Point(dst_roi.x + 5, dst_roi.y + dst_roi.height - 10),
                cv::FONT_HERSHEY_PLAIN, 1, title_color);
}

} // end namespace vision_utils

#endif // PASTE_IMG_COMPUTE_ROIS_H
