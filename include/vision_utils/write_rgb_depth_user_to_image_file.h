/*!
  \file        write_rgb_depth_user_to_image_file.h
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

#ifndef WRITE_RGB_DEPTH_USER_TO_IMAGE_FILE_H
#define WRITE_RGB_DEPTH_USER_TO_IMAGE_FILE_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include <vision_utils/write_rgb_and_depth_image_to_image_file.h>
#include <vision_utils/user_image_to_rgb.h>
#include <vision_utils/convert_n_colors.h>

namespace vision_utils {

inline bool write_rgb_depth_user_to_image_file
(const std::string & filename_prefix,
 const cv::Mat3b * rgb = NULL,
 const cv::Mat1f * depth = NULL,
 const cv::Mat1b * user = NULL,
 cv::Mat3b * user_illus = NULL,
 FileFormat format = FILE_PNG,
 bool debug_info = true)
{
  if (!write_rgb_and_depth_image_to_image_file
      (filename_prefix, rgb, depth, format, debug_info))
    return false;

  if((user != NULL && user_illus == NULL)
     || (user == NULL && user_illus != NULL)) {
    printf("write_rgb_depth_user_to_image_file(): user and user_illus "
           "should both be either NULL or non NULL\n");
    return false;
  }
  std::string extension = format2extension(format);
  ParamsVec params = format2params(format);
  std::ostringstream user_mask_filename;
  user_mask_filename << filename_prefix << "_user_mask" << extension;
  if (!cv::imwrite(user_mask_filename.str(), *user, params)) {
    printf("write_user_mask_and_depth_image_as_uchar_to_image_file(): "
           "could not write user_mask image '%s'\n", user_mask_filename.str().c_str());
    return false;
  }
  if (debug_info)
    printf("Written user file '%s'.\n", user_mask_filename.str().c_str());

  user_image_to_rgb(*user, *user_illus, 8);
  std::ostringstream user_mask_illus_filename;
  user_mask_illus_filename << filename_prefix << "_user_mask_illus" << extension;
  if (!cv::imwrite(user_mask_illus_filename.str(), *user_illus, params)) {
    printf("write_user_mask_illus_and_depth_image_as_uchar_to_image_file(): "
           "could not write user_mask_illus image '%s'\n", user_mask_illus_filename.str().c_str());
    return false;
  }
  if (!convert_n_colors(user_mask_illus_filename.str(), 256,
                        user_mask_illus_filename.str()))
    return false;
  if (debug_info)
    printf("Written user illus file '%s'.\n", user_mask_illus_filename.str().c_str());
  return true;
}

} // end namespace vision_utils

#endif // WRITE_RGB_DEPTH_USER_TO_IMAGE_FILE_H
