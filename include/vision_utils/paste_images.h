/*!
  \file        paste_images.h
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

#ifndef PASTE_IMAGES_H
#define PASTE_IMAGES_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>
#include "vision_utils/draw_text_centered.h"
#include "vision_utils/paste_img_compute_rois.h"
#include <vision_utils/resize_if_bigger.h>
#include "vision_utils/titlemaps.h"

namespace vision_utils {

static const unsigned int HEADER_SIZE = 30;

inline cv::Rect paste_images_image_roi
(unsigned int img_idx,
 bool horizontal_pasting = true,
 const int width1 = 50,
 const int height1 = 50,
 bool draw_headers = true)
{
  return cv::Rect(
        width1 * (horizontal_pasting ? img_idx : 0),
        height1 * (horizontal_pasting ? 0 : img_idx),
        width1 + (!horizontal_pasting && draw_headers ? HEADER_SIZE : 0),
        height1 + (horizontal_pasting && draw_headers ? HEADER_SIZE : 0)
        );
} // end paste_images_image_roi()

////////////////////////////////////////////////////////////////////////////////

inline int paste_images_pixel_belong_to_image
(int pixel_x, int pixel_y,
 bool horizontal_pasting = true,
 const int width1 = 50,
 const int height1 = 50,
 bool draw_headers = true)
{
  int img_idx = 0;
  cv::Point pixel(pixel_x, pixel_y);
  while (img_idx < 200) { // foolproof
    cv::Rect img_roi = paste_images_image_roi
                       (img_idx, horizontal_pasting, width1, height1, draw_headers);
    if (img_roi.contains(pixel)) // pixel in ROI? we found it!
      return img_idx;
    if (img_roi.x > pixel_x || img_roi.y > pixel_y) // we went over the pixel
      return -1;
    ++img_idx;
  }
  // should not be reached - ever
  return -1;
} // end paste_images_pixel_belong_to_image()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Paste a list of images horizontally into a collage image.
 * \param imgs
 *  The series of input images
 * \param out
 *  The image where all images of "imgs" are pasted.
 * \param width1
 *  The wanted width of a column, in pixels.
 * \param height
 *  The wanted total height of "out", in pixels
 * \param colpadding
 *  The white space between columns, in pixels.
 *  Not taken into acount if "do_not_constrain_width" is true.
 * \param draw_headers
 *  true for drawing column headers
 * \param column_titlemap
 *  the function for convertnig column indices into header titles (strings)
 * \param masks
 *  if not empty, must be the same size than imgs.
 *  Contains the mask for each input image.
 * \param do_not_constrain_width
 *  if true, only constrain images height, and not width.
 *  Thus, pasted images can be wider than "width1" and overlap each other.
 */
template<class _T>
void paste_images(const std::vector<cv::Mat_<_T> > & imgs,
                  cv::Mat_<_T> & out,
                  bool horizontal_pasting = true,
                  const int width1 = 50,
                  const int height1 = 50,
                  const int itempadding = 5,
                  bool draw_headers = true,
                  Map column_titlemap = int_to_uppercase_letter,
                  const std::vector<cv::Mat1b> & masks = std::vector<cv::Mat1b>(),
                  bool do_not_constrain_width = false)
{
  //  printf("paste_images: %i imgs, width1:%i, height1:%i, draw_headers:%i, "
  //         "do_not_constrain_width:%i\n",
  //         imgs.size(), width1, height1, draw_headers, do_not_constrain_width);
  unsigned int nimgs = imgs.size();
  // fool-proof sizes
  if (nimgs == 0) {
    out = cv::Mat_<_T>(1, 1);
    return;
  }

  bool use_masks = (masks.size() > 0);
  if (use_masks && nimgs != masks.size()) {
    printf("paste_images(): Wrong number of masks: %i != %li\n", nimgs, masks.size());
    return;
  }

  // check size big enough
  int header_size = (draw_headers ? HEADER_SIZE : 0);
  int width_final = (horizontal_pasting ? width1 * nimgs : width1 + header_size);
  int height_final = (horizontal_pasting ? height1 + header_size : height1 * nimgs);
  if (out.cols < width_final || out.rows < height_final)
    out.create(height_final, width_final);
  int allowed_width1 = (do_not_constrain_width ?
                          1E5
                        : width1 - (horizontal_pasting ? itempadding: 0));
  int allowed_height1 = height1 - (horizontal_pasting ? 0: itempadding);

  // clear background
  out.setTo(255);

  // draw ROIS - DEBUG
  //for (uint img_idx = 0; img_idx < nimgs; ++img_idx)
  //  cv::rectangle(out, paste_images_image_roi(img_idx, horizontal_pasting, width1, height1, draw_headers),
  //                cv::Scalar::all(rand() % 200), -1);

  // paste all images
  for (unsigned int img_idx = 0; img_idx < nimgs; ++img_idx) {
    // resize image if bigger than allowed space
    cv::Mat_<_T> curr_img_resized;
    cv::Mat1b curr_mask_resized;
    resize_if_bigger
        (imgs[img_idx], curr_img_resized, allowed_width1, allowed_height1,
         cv::INTER_NEAREST, true, true);
    if (use_masks) {
      resize_if_bigger
          (masks[img_idx], curr_mask_resized, allowed_width1, allowed_height1,
           cv::INTER_NEAREST, true, false);
    }

    // cv::imshow("curr_img_resized", curr_img_resized); cv::waitKey(0);
    int paste_x = (width1 - curr_img_resized.cols) / 2
                  + (horizontal_pasting ? img_idx * width1 : header_size);
    int paste_y = (height1 - curr_img_resized.rows) / 2
                  + (horizontal_pasting ? 0 : img_idx * height1);
    paste_img(curr_img_resized, out, paste_x, paste_y,
                           (use_masks ? &curr_mask_resized : NULL));
    //    printf("curr_img_resized:%i x %i, paste_x:%i, paste_y:%i\n",
    //           curr_img_resized.cols, curr_img_resized.rows, paste_x, paste_y);
  } // end loop img_idx

  // draw headers
  if (draw_headers) {
    for (uint img_idx = 0; img_idx < nimgs; ++img_idx) {
      int header_x = (horizontal_pasting ? width1 * (.5 + img_idx)
                                         : header_size / 2);
      int header_y = (horizontal_pasting ? height1 + header_size / 2
                                         : height1 * (.5 + img_idx));
      draw_text_centered
          (out, column_titlemap(img_idx), cv::Point(header_x, header_y),
           CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 0, 0));
    } // end for img_idx
  } // end if (draw_headers)

} // end if paste_images();

} // end namespace vision_utils

#endif // PASTE_IMAGES_H
