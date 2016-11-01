/*!
  \file        ros_object_from_file.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/10/30
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

#ifndef ROS_OBJECT_FROM_FILE_H
#define ROS_OBJECT_FROM_FILE_H

#include <vision_utils/retrieve_file.h>
#include <vision_utils/ros_object_from_string.h>

namespace vision_utils {

template<class _T>
void ros_object_from_file(const std::string & filename,
                          _T & object) {
  std::string object_str;
  retrieve_file(filename, object_str);
  ros_object_from_string(object_str, object);
}

} // end namespace vision_utils

#endif // ROS_OBJECT_FROM_FILE_H
