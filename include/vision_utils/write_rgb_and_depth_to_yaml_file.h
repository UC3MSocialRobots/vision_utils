/*!
  \file        write_rgb_and_depth_to_yaml_file.h
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

#ifndef WRITE_RGB_AND_DEPTH_TO_YAML_FILE_H
#define WRITE_RGB_AND_DEPTH_TO_YAML_FILE_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

inline void write_rgb_and_depth_to_yaml_file(const std::string & yaml_filename_prefix,
                                             const cv::Mat & rgb, const cv::Mat & depth,
                                             bool debug_info = true) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_rgb_depth.yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::WRITE);
  fs << "rgb" << rgb;
  fs << "depth" << depth;
  fs.release();
  if (debug_info)
    printf("Succesfully written depth and rgb in '%s'\n",
           full_filename.str().c_str());
} // end write_rgb_and_depth_to_yaml_file();

} // end namespace vision_utils

#endif // WRITE_RGB_AND_DEPTH_TO_YAML_FILE_H
