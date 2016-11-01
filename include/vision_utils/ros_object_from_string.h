/*!
  \file        ros_object_from_string.h
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

#ifndef ROS_OBJECT_FROM_STRING_H
#define ROS_OBJECT_FROM_STRING_H

// std
#include <fstream>

namespace vision_utils {

template<class _T>
void ros_object_from_string(const std::string & object_str,
                            _T & my_value) {
  uint32_t serial_size2 = object_str.size();
  ros::serialization::IStream stream_in((unsigned char*) object_str.data(), serial_size2);
  ros::serialization::Serializer<_T>::read(stream_in, my_value);
}

} // end namespace vision_utils

#endif // ROS_OBJECT_FROM_STRING_H
