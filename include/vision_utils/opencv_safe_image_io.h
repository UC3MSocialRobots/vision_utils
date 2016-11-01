/*!
  \file        opencv_safe_image_io.h
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
 */
#ifndef OPENCV_SAFE_IMAGE_IO_H
#define OPENCV_SAFE_IMAGE_IO_H

#include <opencv2/highgui/highgui.hpp>

namespace vision_utils {

bool imread_safe(cv::Mat & ans,
                 const std::string& filename,
                 int flags=1) {
  // test file existence
  std::ifstream my_file(filename.c_str());
  if (!my_file.good()) {
    printf("imread_safe(): file '%s' does not exist!\n",
           filename.c_str());
    return false;
  }
  // try and read it
  bool success = false;
  try {
    ans = cv::imread(filename, flags);
    success = !ans.empty();
  }
  catch (cv::Exception e) {
    printf("imread_safe(): exception '%s'\n", e.what());
  }
  if (!success) {
    printf("imread_safe(): could not load '%s'\n", filename.c_str());
    return false;
  }
  return true;
} // end imread_safe()
} // end namespace vision_utils

#endif // OPENCV_SAFE_IMAGE_IO_H
