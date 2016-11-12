/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/11/12
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

#ifndef PPL_TAGS_IMAGES_H
#define PPL_TAGS_IMAGES_H

// vision_utils
#include <vision_utils/ppl_attributes.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>

namespace vision_utils {

template<class T>
inline cv::Mat_<T> get_image_tag(const people_msgs::Person & pp,
                                 const std::string & img_name) {
  cv::Mat_<T> ans;
  // get filename
  // try to read image
  return ans;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline bool set_image_tag(people_msgs::Person & pp,
                          const std::string & img_name,
                          const cv::Mat_<T> & img) {
   return false;
}

} // end namespace vision_utils

#endif // PPL_TAGS_IMAGES_H

