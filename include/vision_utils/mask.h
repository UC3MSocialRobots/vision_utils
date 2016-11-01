/*!
  \file        mask.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/11/20

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

\section Parameters
  - \b "foo"
        [string] (default: "bar")
        Description of the parameter.

\section Subscriptions
  - \b "/foo"
        [xxx]
        Descrption of the subscription

\section Publications
  - \b "~foo"
        [xxx]
        Descrption of the publication

 */

#ifndef MASK_H
#define MASK_H

#include <opencv2/core/core.hpp>

namespace vision_utils {

inline bool is_nan_const_double(const double & val) { return isnan(val);}

inline bool is_nan_const_float(const float & val) { return isnan(val);}

inline bool is_zero_uchar(const uchar & val) { return val == 0; }

inline bool is_zero_vec3b(const cv::Vec3b & val) {
  return val[0] == 0 && val[1] == 0 && val[2] == 0;
}

inline bool is_white_vec3b(const cv::Vec3b & val) {
  return val[0] == 255 && val[1] == 255 && val[2] == 255;
}

inline bool is_non_zero_vec3b(const cv::Vec3b & val) {
  return val[0] || val[1] || val[2];
}

/*!
 * Apply a mask function on an input image.
 * \param in
 *    the image for generating the mask.
 * \param out
 *    the output mask.
 * \param excluding_fn_ptr
 *    the function that determines if a value belongs to the mask or not.
 *    The mask is initially equal to 255 for all pixels.
 *    For a pixel with a given value, if the function returns true,
 *    then the given pixel does not belong to the mask,
 *    and its value in the mask is set to 0.
 *    For instance, with a RGB image, you can use is_zero_vec3b()
 *    to keep only non null values of the image in the mask.
 */
template<class _T>
inline void mask(const cv::Mat & in,
                 cv::Mat1b & out,
                 bool (*excluding_fn_ptr)(const _T & zero_val)) {
  out.create(in.size());
  out.setTo(255);
  for (int row = 0; row < in.rows; ++row) {
    // get the address of row
    const _T* in_data = in.ptr<_T>(row);
    uchar* out_data = out.ptr<uchar>(row);
    for (int col = 0; col < in.cols; ++col) {
      if ((*excluding_fn_ptr)(in_data[col]))
        out_data[col] = 0;
    } // end loop col
  } // end loop row
} // end mask()

////////////////////////////////////////////////////////////////////////////////

void color_mask(const cv::Mat3b & img, const cv::Scalar color,
                cv::Mat1b & mask) {
  unsigned int rows = img.rows, cols = img.cols;
  mask.create(img.size());
  mask.setTo(0);
  cv::Vec3b color3b(color[0], color[1], color[2]);
  for (unsigned int row = 0; row < rows; ++row) {
    const cv::Vec3b*row_ptr = img.ptr<cv::Vec3b>(row);
    uchar* mask_buffer_ptr = mask.ptr<uchar>(row);
    for (unsigned int col = 0; col < cols; ++col) {
      if (row_ptr[col] == color3b)
        mask_buffer_ptr[col] = 255;
    } // end loop col
  } // end loop row
} // end color_mask();

////////////////////////////////////////////////////////////////////////////////

bool color_mask_ncolors(const cv::Mat3b & img,
                        const unsigned int ncolors,
                        const cv::Scalar* colors,
                        cv::Mat1b & mask) {
  unsigned int rows = img.rows, cols = img.cols;
  mask.create(img.size());
  mask.setTo(0);
  std::vector<cv::Vec3b> colors_vec;
  for (unsigned int i = 0; i < ncolors; ++i)
    colors_vec.push_back(cv::Vec3b(colors[i][0], colors[i][1], colors[i][2]));
  for (unsigned int row = 0; row < rows; ++row) {
    const cv::Vec3b*row_ptr = img.ptr<cv::Vec3b>(row);
    uchar* mask_buffer_ptr = mask.ptr<uchar>(row);
    for (unsigned int col = 0; col < cols; ++col) {
      bool ok = false;
      for (unsigned int i = 0; i < ncolors; ++i) {
        if (row_ptr[col] != colors_vec[i])
          continue;
        mask_buffer_ptr[col] = i;
        ok = true;
        break;
      } // end loop i
      if (!ok) {
        printf("color_mask_ncolors(): color (%i,%i,%i) not belonging to table!\n",
               row_ptr[col][0], row_ptr[col][1], row_ptr[col][2]);
        return false;
      }
    } // end loop col
  } // end loop row
  return true;
} // end color_mask();

} // end namespace vision_utils

#endif // MASK_H
