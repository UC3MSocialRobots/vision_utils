/*!
  \file        write_rgb_and_depth_image_to_image_file.h
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

#ifndef WRITE_RGB_AND_DEPTH_IMAGE_TO_IMAGE_FILE_H
#define WRITE_RGB_AND_DEPTH_IMAGE_TO_IMAGE_FILE_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
#include "vision_utils/file_format.h"

namespace vision_utils {

/*!
  write depth and rgb files to PNG images.
 \param filename_prefix
    Where to write the files, for instance "/tmp/test"
 \param rgb_img
    The 3 channels RGB image.
    If NULL, no RGB output will be written.
 \param depth_img_as_uchar, alpha, beta
    The data of the float image converted to uchar,
    using convert_float_to_uchar()
    If depth_img_as_uchar = NULL, no depth output will be written.
 \example filename_prefix = "/tmp/test"
    will generate "/tmp/test_depth.png"; "/tmp/test_depth_params.png";
    "/tmp/test_rgb.png";
*/
inline bool write_rgb_and_depth_image_as_uchar_to_image_file
(const std::string & filename_prefix,
 const cv::Mat * rgb_img = NULL,
 const cv::Mat * depth_img_as_uchar = NULL,
 const ScaleFactorType * alpha = NULL, const ScaleFactorType * beta = NULL,
 FileFormat format = FILE_PNG,
 bool debug_info = true)
{
  std::string extension = format2extension(format);
  ParamsVec params = format2params(format);
  // depth file
  if (depth_img_as_uchar != NULL) {
    std::ostringstream depth_img_filename;
    depth_img_filename << filename_prefix << "_depth" << extension;
    if (!cv::imwrite(depth_img_filename.str(), *depth_img_as_uchar, params)) {
      printf("write_rgb_and_depth_image_as_uchar_to_image_file(): "
             "could not write depth image '%s'\n", depth_img_filename.str().c_str());
      return false;
    }

    // depth params file
    std::ostringstream params_textfile_filename;
    params_textfile_filename << filename_prefix << "_depth_params.yaml";
    cv::FileStorage fs(params_textfile_filename.str(), cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
      printf("write_rgb_and_depth_image_as_uchar_to_image_file(): "
             "could not open depth params file '%s'\n", params_textfile_filename.str().c_str());
      return false;
    }
    fs << "alpha" << *alpha;
    fs << "beta" << *beta;
    fs.release();
    if (debug_info)
      printf("Written depth files '%s' and '%s'.\n",
             depth_img_filename.str().c_str(), params_textfile_filename.str().c_str());
  } // end (depth_img_as_uchar != NULL)

  // RGB file
  if (rgb_img != NULL) {
    std::ostringstream rgb_img_filename;
    rgb_img_filename << filename_prefix << "_rgb"  << extension;
    if (!cv::imwrite(rgb_img_filename.str(), *rgb_img, params)) {
      printf("write_rgb_and_depth_image_as_uchar_to_image_file(): "
             "could not write rgb image '%s'\n", rgb_img_filename.str().c_str());
      return false;
    }
    if (debug_info)
      printf("Written RGB file '%s'.\n", rgb_img_filename.str().c_str());
  } // end (rgb_img != NULL)
  return true;
} // end write_rgb_and_depth_image_as_uchar_to_image_file();

////////////////////////////////////////////////////////////////////////////////

/*!
  Converts the float image to uchar, then write depth and rgb files to PNG images.
  If rgb_img == NULL, no RGB output is written.
  If depth_img == NULL, no depth output is written.
  \see write_rgb_and_depth_image_as_uchar_to_image_file(),
       convert_float_to_uchar()
*/
inline bool write_rgb_and_depth_image_to_image_file
(const std::string & filename_prefix,
 const cv::Mat * rgb_img = NULL,
 const cv::Mat * depth_img = NULL,
 FileFormat format = FILE_PNG,
 bool debug_info = true)
{
  if (depth_img != NULL) {
    cv::Mat depth_img_as_uchar, src_float_clean_buffer;
    ScaleFactorType alpha, beta;
    convert_float_to_uchar(*depth_img, depth_img_as_uchar, src_float_clean_buffer,
                           alpha, beta);
    return write_rgb_and_depth_image_as_uchar_to_image_file
        (filename_prefix, rgb_img, &depth_img_as_uchar, &alpha, &beta,
         format, debug_info);
  } // end (depth_img != NULL)
  else
    return write_rgb_and_depth_image_as_uchar_to_image_file
        (filename_prefix, rgb_img, NULL, NULL, NULL, format, debug_info);
}

} // end namespace vision_utils

#endif // WRITE_RGB_AND_DEPTH_IMAGE_TO_IMAGE_FILE_H
