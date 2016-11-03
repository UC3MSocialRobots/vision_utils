/*!
  \file        read_rgb_and_depth_image_from_image_file.h
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

#ifndef READ_RGB_AND_DEPTH_IMAGE_FROM_IMAGE_FILE_H
#define READ_RGB_AND_DEPTH_IMAGE_FROM_IMAGE_FILE_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include "vision_utils/convert_uchar_to_float.h"
#include "vision_utils/file_format.h"
#include "vision_utils/file_exists.h"

namespace vision_utils {

/*!
  read depth and rgb files from PNG images.
 \param filename_prefix
    Where to read the files, for instance "/tmp/test"
 \param rgb_img
    The 3 channels RGB image.
    If NULL, no RGB input will be read.
 \param depth_img_as_uchar, alpha, beta
    The data of the float-as-uchar image,
    which can be used for convert_uchar_to_float();
    If depth_img_as_uchar = NULL, no depth input will be read.
 \example filename_prefix = "/tmp/test"
    will generate "/tmp/test_depth.png"; "/tmp/test_depth_params.png";
    "/tmp/test_rgb.png";
*/
inline bool read_rgb_and_depth_image_as_uchar_from_image_file
(const std::string & filename_prefix,
 cv::Mat * rgb_img = NULL,
 cv::Mat * depth_img_as_uchar = NULL,
 ScaleFactorType * alpha = NULL, ScaleFactorType * beta = NULL,
 FileFormat format = FILE_PNG)
{
  std::string extension = format2extension(format);
  if (depth_img_as_uchar != NULL) {
    // depth img
    std::ostringstream depth_img_filename;
    depth_img_filename << filename_prefix << "_depth" << extension;
    if (!file_exists(depth_img_filename.str())) {
      printf("depth img file '%s' does not exist, cannot read it!\n",
             depth_img_filename.str().c_str());
      return false;
    }
    *depth_img_as_uchar = cv::imread(depth_img_filename.str(), CV_LOAD_IMAGE_GRAYSCALE);
    if (depth_img_as_uchar->empty()) {
      printf("depth_img_as_uchar '%s' is corrupted!\n", depth_img_filename.str().c_str());
      return false;
    }
    // params file
    std::ostringstream params_textfile_filename;
    params_textfile_filename << filename_prefix << "_depth_params.yaml";
    if (!file_exists(params_textfile_filename.str())) {
      printf("params_textfile img file '%s' does not exist, cannot read it!\n",
             params_textfile_filename.str().c_str());
      return false;
    }
    cv::FileStorage fs(params_textfile_filename.str(), cv::FileStorage::READ);
    fs["alpha"] >> *alpha;
    fs["beta"] >> *beta;
    fs.release();
    //    printf("Read depth file '%s' (params_textfile:'%s').\n",
    //           depth_img_filename.str().c_str(),
    //           params_textfile_filename.str().c_str());
  } // end (depth_img_as_uchar != NULL)

  // rgb img
  if (rgb_img != NULL) {
    std::ostringstream rgb_img_filename;
    rgb_img_filename << filename_prefix << "_rgb" << extension;
    if (file_exists(rgb_img_filename.str()))
      *rgb_img = cv::imread(rgb_img_filename.str());
    else { // try another format
      std::ostringstream rgb_jpg_img_filename;
      rgb_jpg_img_filename << filename_prefix << "_rgb.jpg";
      if (!file_exists(rgb_jpg_img_filename.str())) {
        printf("rgb img file '%s' does not exist, cannot read it!\n",
               rgb_img_filename.str().c_str());
        return false;
      }
      *rgb_img = cv::imread(rgb_jpg_img_filename.str());
    }
    //printf("Read rgb file '%s'.\n", rgb_img_filename.str().c_str());
    if (rgb_img->empty()) {
      printf("rgb_img '%s' is corrupted!\n", rgb_img_filename.str().c_str());
      return false;
    }
  } // end (rgb_img != NULL)
  return true;
} // end read_rgb_and_depth_image_as_uchar_from_image_file();

////////////////////////////////////////////////////////////////////////////////

/*!
  Read RGB and depth-as-uchar images from PNG,
  then converts depth-as-uchar to depth-as-float.
  If rgb_img == NULL, no RGB input is read.
  If depth_img == NULL, no depth input is read.
  \see read_rgb_and_depth_image_as_uchar_from_image_file(),
       convert_float_to_uchar()
*/
inline bool read_rgb_and_depth_image_from_image_file
(const std::string & filename_prefix,
 cv::Mat * rgb_img = NULL,
 cv::Mat * depth_img = NULL,
 FileFormat format = FILE_PNG)
{
  if (depth_img != NULL) {
    cv::Mat depth_img_as_uchar;
    ScaleFactorType alpha = 1, beta = 0;
    bool ok = read_rgb_and_depth_image_as_uchar_from_image_file
              (filename_prefix, rgb_img, &depth_img_as_uchar, &alpha, &beta, format);
    if (!ok)
      return false;
    convert_uchar_to_float(depth_img_as_uchar, *depth_img, alpha, beta);
    return true;
  } // end (depth_img != NULL)
  return read_rgb_and_depth_image_as_uchar_from_image_file
      (filename_prefix, rgb_img, NULL, NULL, NULL, format);
}

} // end namespace vision_utils

#endif // READ_RGB_AND_DEPTH_IMAGE_FROM_IMAGE_FILE_H
