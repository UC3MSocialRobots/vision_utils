/*!
  \file        user_image_to_rgb.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/7

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

#ifndef USER_IMAGE_TO_RGB_H
#define USER_IMAGE_TO_RGB_H

#include <opencv2/core/core.hpp>
#include <stdio.h>

/*!
 \param user
 \param out
 \param data_size
          8 if user indices are unsigned char,
          16 if they are short ints
*/
inline void user_image_to_rgb(const cv::Mat & user, cv::Mat3b & out,
                              int data_size = 8) {
  // paint image in black
  unsigned int rows = user.rows, cols = user.cols;
  out.create(rows, cols);
  out.setTo(0);
  if (rows == 0 && cols == 0) {
    printf("user_image_to_rgb: empty input image!\n");
    return;
  }
#if 0
  // iterate through user
  IplImage image_ipl = (IplImage) user;
  int indices_image_val;
  for (unsigned int row = 0; row < rows; ++row) {
    uchar* _out_img_ptr = out.ptr<uchar>(row);
    for (unsigned int col = 0; col < cols; ++col) {
      // XnLabel = unsigned short
      switch (data_size) {
        case 8:
          indices_image_val = CV_IMAGE_ELEM(&image_ipl, uchar, row, col);
          break;
        case 16:
        default:
          indices_image_val = CV_IMAGE_ELEM(&image_ipl, unsigned short, row, col);
          break;
      } // end switch
      if (indices_image_val!= 0) {
        color_utils::indexed_color255
            (_out_img_ptr[3 * col + 2],
            _out_img_ptr[3 * col + 1],
            _out_img_ptr[3 * col    ],
            indices_image_val);
      }
    } // end loop col
  } // end loop row
#else
  assert(user.isContinuous());
  assert(out.isContinuous());
  int ndata = cols * rows;
  // compute a set predetermined colors
  int ncolors = 24, i = 0;
  cv::Vec3b color_lut[ncolors];
  color_lut[i++] = cv::Vec3b(255, 0, 0);
  color_lut[i++] = cv::Vec3b(0, 255, 0);
  color_lut[i++] = cv::Vec3b(0, 0, 255);

  color_lut[i++] = cv::Vec3b(255, 255, 0);
  color_lut[i++] = cv::Vec3b(255, 0, 255);
  color_lut[i++] = cv::Vec3b(0, 255, 255);

  color_lut[i++] = cv::Vec3b(255, 160, 0);
  color_lut[i++] = cv::Vec3b(255, 0, 160);
  color_lut[i++] = cv::Vec3b(255, 160, 160);
  color_lut[i++] = cv::Vec3b(160, 255, 0);
  color_lut[i++] = cv::Vec3b(0, 255, 160);
  color_lut[i++] = cv::Vec3b(160, 255, 160);
  color_lut[i++] = cv::Vec3b(160, 0, 255);
  color_lut[i++] = cv::Vec3b(0, 160, 255);
  color_lut[i++] = cv::Vec3b(160, 160, 255);

  color_lut[i++] = cv::Vec3b(160, 128, 0);
  color_lut[i++] = cv::Vec3b(160, 0, 128);
  color_lut[i++] = cv::Vec3b(160, 128, 128);
  color_lut[i++] = cv::Vec3b(128, 160, 0);
  color_lut[i++] = cv::Vec3b(0, 160, 128);
  color_lut[i++] = cv::Vec3b(128, 160, 128);
  color_lut[i++] = cv::Vec3b(128, 0, 160);
  color_lut[i++] = cv::Vec3b(0, 128, 160);
  color_lut[i++] = cv::Vec3b(128, 128, 160);

  cv::Vec3b* out_ptr = out.ptr<cv::Vec3b>(0);
  if (data_size == 8) { // uchar
    const uchar*  user_ptr = user.ptr<uchar>(0);
    for (int i = 0; i < ndata; ++i) {
      if (*user_ptr)
        out_ptr[i] = color_lut[*user_ptr % ncolors];
      ++user_ptr;
    } // end for (i)
  } else if (data_size == 16) { // unsigned short
    const unsigned short*  user_ptr = user.ptr<unsigned short>(0);
    for (int i = 0; i < ndata; ++i) {
      if (*user_ptr)
        out_ptr[i] = color_lut[*user_ptr % ncolors];
      ++user_ptr;
    } // end for (i)
  } else {
    printf("Incorrect data_size:%i\n", data_size);
  }
#endif
} // end user_image_to_rgb();

////////////////////////////////////////////////////////////////////////////////

inline cv::Mat3b user_image_to_rgb(const cv::Mat & user, int data_size = 8) {
  cv::Mat3b rgb;
  user_image_to_rgb(user, rgb, data_size);
  return rgb;
}

#endif // USER_IMAGE_TO_RGB_H
