/*!
  \file        concatenate_vector.h
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

#ifndef CONCATENATE_VECTOR_H
#define CONCATENATE_VECTOR_H
// std includes
#include <sstream> // for ostringstream
#include <string>
#include <vector>

namespace vision_utils {

inline std::string concatenate_vector(const std::vector<std::string> & vec) {
  //printf("concatenate_vector()");
  std::ostringstream out;
  for (std::vector<std::string>::const_iterator line = vec.begin();
       line != vec.end() ; ++line) {
    // add a newline except for first line
    if (line != vec.begin())
      out << std::endl;
    out << *line;
  }
  return out.str();
}

} // end namespace vision_utils

#endif // CONCATENATE_VECTOR_H
