/*!
  \file        read_depth_rgb_from_yaml_file.h
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

#ifndef READ_DEPTH_RGB_FROM_YAML_FILE_H
#define READ_DEPTH_RGB_FROM_YAML_FILE_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

inline bool read_depth_rgb_from_yaml_file(const std::string & yaml_filename_prefix,
                                          cv::Mat & rgb, cv::Mat & depth) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_rgb_depth.yaml";
  if (!file_exists(full_filename.str())) {
    printf("File '%s' does not exist!\n", full_filename.str().c_str());
    return false;
  }
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::READ);
  fs["rgb"] >> rgb;
  fs["depth"] >> depth;
  fs.release();
  if (rgb.empty() || depth.empty()) {
    printf("read_depth_rgb_from_yaml_file('%s'): "
           "depth(%s) or RGB(%s) images are empty!",
           full_filename.str().c_str(),
           infosImage(depth).c_str(),
           infosImage(rgb).c_str());
    return false;
  }
  return true;
} // end read_depth_rgb_from_yaml_file();

} // end namespace vision_utils

#endif // READ_DEPTH_RGB_FROM_YAML_FILE_H
