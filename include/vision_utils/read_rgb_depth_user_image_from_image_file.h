/*!
  \file        read_rgb_depth_user_image_from_image_file.h
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

#ifndef READ_RGB_DEPTH_USER_IMAGE_FROM_IMAGE_FILE_H
#define READ_RGB_DEPTH_USER_IMAGE_FROM_IMAGE_FILE_H
// std includes
#include "vision_utils/file_format.h"
#include "vision_utils/read_rgb_and_depth_image_from_image_file.h"

#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

bool read_rgb_depth_user_image_from_image_file(const std::string & filename_prefix,
                                               cv::Mat* rgb = NULL,
                                               cv::Mat* depth = NULL,
                                               cv::Mat1b* user_mask = NULL,
                                               FileFormat format = FILE_PNG) {
  // read depth and rgb
  if (!read_rgb_and_depth_image_from_image_file
      (filename_prefix, rgb, depth, format))
    return false;
  // read user mask
  if (user_mask == NULL)
    return true;
  std::ostringstream user_mask_filename;
  user_mask_filename << filename_prefix << "_user_mask"  << format2extension(format);
  if (!file_exists(user_mask_filename.str())) {
    printf("user mask file '%s' does not exist, cannot read it!\n",
           user_mask_filename.str().c_str());
    return false;
  }
  *user_mask = cv::imread(user_mask_filename.str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (user_mask->empty()) {
    printf("user mask file '%s' is corrupted!\n", user_mask_filename.str().c_str());
    return false;
  }
  return true;
  //printf("Read user mask file '%s'.\n", user_mask_filename.str().c_str());
}

} // end namespace vision_utils

#endif // READ_RGB_DEPTH_USER_IMAGE_FROM_IMAGE_FILE_H
