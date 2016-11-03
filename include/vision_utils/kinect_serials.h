/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/11/2
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

#ifndef KINECT_SERIALS_H
#define KINECT_SERIALS_H
#include <string>

namespace vision_utils {

inline std::string KINECT_SERIAL_ARNAUD() { return "A00365A10630110A"; }
inline std::string KINECT_SERIAL_LAB() { return "A00365A82054035A"; }
inline std::string DEFAULT_KINECT_SERIAL() { return KINECT_SERIAL_LAB(); }

} // end namespace vision_utils

#endif // KINECT_SERIALS_H

