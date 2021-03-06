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
#include <vision_utils/file_exists.h>
#include <vision_utils/timestamp.h>
#include <vision_utils/file_format.h>
// ROS
#include <ros/ros.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
// C++
#include <sstream>

namespace vision_utils {

template<class T>
inline bool get_image_tag(const people_msgs::Person & pp,
                          const std::string & img_tagname,
                          cv::Mat_<T> & img) {
  // get filename
  std::string img_file;
  if (!get_tag(pp, img_tagname, img_file))
    return false;
  // try to read image
  bool ok = true;
  try {
    img = cv::imread(img_file, cv::IMREAD_UNCHANGED);
  }
  catch (cv::Exception & e) {
    ok = false;
  }
  if (img.empty())
    ok = false;
  if (!ok)
    printf("Error reading file '%s'!\n", img_file.c_str());
  return ok;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline bool set_image_tag(people_msgs::Person & pp,
                          const std::string & img_tagname,
                          const cv::Mat_<T> & img) {
  std::ostringstream filename;
  int i = 0;
  //std::vector<int> params;
  //params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  //params.push_back(0);
  while(i < 100) { // 100 tries
    ++i;
    filename.str("");
    filename << "/tmp/ppl_" << vision_utils::timestamp()
             << "_" << img_tagname;
    if (i > 1) filename << i;
    filename << ".png";
    if (file_exists(filename.str()))
      continue;
    if (!cv::imwrite(filename.str(), img))//, params))
      return false;
    return set_tag(pp, img_tagname, filename.str());
  }
  ROS_WARN("Could not save '%s' to '%s'", img_tagname.c_str(), filename.str().c_str());
  return false;
}

} // end namespace vision_utils

#endif // PPL_TAGS_IMAGES_H

