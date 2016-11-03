/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/11/2
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

  odo Description of the file
 */

#ifndef DRAW_TEXT_ROTATED_H
#define DRAW_TEXT_ROTATED_H

#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/paste_img_compute_rois.h"
#include "vision_utils/rotate_image.h"

namespace vision_utils {

/*!
 \param img
    the image where the text will be put
 \param buffer1, buffer2
    Two buffers for rendering. Memory allocation is made inside the function.
    the horizontal text will be put in buffer1.
    buffer2 is the rotated version of buffer1
    (cv::warpAffine() can not operate in-place)
 \param text
    The text to be put
 \param org
    Where to put the text in img
 \param angle_rad
    The angle of rotation for the text, in radians.
 \param fontFace, fontScale, color, thickness, linetype
    \see cv::putText()
*/
inline void draw_text_rotated
(cv::Mat3b& img, cv::Mat1b& buffer1, cv::Mat1b& buffer2,
 const std::string& text,
 const cv::Point & org, const double & angle_rad,
 const int & fontFace, const double & fontScale, const cv::Scalar & color,
 const int thickness=1, const int linetype=8) {
  // do nothing if angle almost 0
  if (cos(angle_rad) > .995) {
    cv::putText(img, text, org, fontFace, fontScale, color, thickness, linetype);
    buffer1.create(1, 1);
    return;
  }

  // get size of text if not rotated
  int baseline = 0;
  cv::Size txt_size =
      cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  int buffer_dim = 2 * std::max(txt_size.width, txt_size.height);

  // create buffer if not big enough
  if (buffer1.cols < buffer_dim || buffer1.rows < buffer_dim) {
    //printf("resize\n");
    buffer1.create(buffer_dim, buffer_dim);
  }
  // buffer2 alloc made by rotate_image();

  // write text with no rotation in buffer
  cv::Point text_pos(buffer_dim / 2, buffer_dim / 2);
  buffer1 = 0;
  cv::putText(buffer1, text, text_pos, fontFace, fontScale,
              cv::Scalar::all(255), thickness, linetype);

  // find the ROI where buffer2 would be pasted
  cv::Rect topaste_roi, dst_roi;
  paste_img_compute_rois((cv::Mat) buffer1, (cv::Mat) img,
                                      org.x - text_pos.x, org.y - text_pos.y,
                                      topaste_roi, dst_roi);
  // do nothing if out of bounds
  if (topaste_roi.width == 0)
    return;

#if 1
  // rotate buffer1 into buffer2
  rotate_image(buffer1, buffer2, angle_rad, text_pos,
               cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
  // find a background color different from text color
  // cv::Vec3b bg_color = (color[0] == 0 ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0));

  // now paint img with color, use buffer2 as mask
  //  paste_img((cv::Mat) buffer3, img, org.x - text_pos.x, org.y - text_pos.y,
  //                         &buffer2);
  img(dst_roi).setTo(color, buffer2(topaste_roi));
#else
  // make a reverse lookup to know if pixels were activated
  IplImage buffer1_ipl = (IplImage) buffer1;
  double cos_a = cos(angle_rad), sin_a = sin(angle_rad);
  int min_col = topaste_roi.x, max_col = topaste_roi.x + topaste_roi.width;
  int min_row = topaste_roi.y, max_row = topaste_roi.y + topaste_roi.height;
  //  printf("min_col:%i, max_col:%i, min_row:%i, max_row:%i\n",
  //           min_col, max_col, min_row, max_row);
  // iterate on the window of buffer1 that needs to be pasted
  for (int row = min_row; row < max_row; ++row) {
    // get the address of row
    uchar* img_data = img.ptr<uchar>(dst_roi.y + row);
    for (int col = min_col; col < max_col; ++col) {
      // compute where would be the coordinates of the rotated pixel in buffer1
      int buffer1_col = text_pos.x
                        + ROTATE_COSSIN_X(col - text_pos.x, row - text_pos.y, cos_a, sin_a);
      int buffer1_row = text_pos.y
                        + ROTATE_COSSIN_Y(col - text_pos.x, row - text_pos.y, cos_a, sin_a);
      //  printf"col:%i, row:%i, buffer1_col:%i, buffer1_row:%i",
      //                    col, row, buffer1_col, buffer1_row);

      // if coordinates are inside bounds and buffer1 at this pixel is white,
      // paint it
      if (buffer1_col >= 0 && buffer1_col < buffer1.cols
          && buffer1_row >= 0 && buffer1_row < buffer1.rows
          && CV_IMAGE_ELEM(&buffer1_ipl, uchar, buffer1_row, buffer1_col) == 255) {
        img_data[3 * (dst_roi.x + col)    ] = color[0];
        img_data[3 * (dst_roi.x + col) + 1] = color[1];
        img_data[3 * (dst_roi.x + col) + 2] = color[2];
      }
    } // end loop col
  } // end loop row

#endif
} // end draw_text_rotated();

} // end namespace vision_utils

#endif // DRAW_TEXT_ROTATED_H

