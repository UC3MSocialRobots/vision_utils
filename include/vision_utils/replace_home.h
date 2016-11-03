/*!
  \file        replace_home.h
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

#ifndef REPLACE_HOME_H
#define REPLACE_HOME_H
// std includes
#include <string>

namespace vision_utils {

inline std::string replace_home(const std::string & filepath) {
  std::string filepath_replaced = filepath;
  std::string::size_type home_pos = filepath_replaced.find('~');
  if (home_pos != std::string::npos) {
    std::string home = getenv("HOME");
    //find_and_replace(filepath_replaced, "~", home);
    filepath_replaced.replace(home_pos, 1, home);
  }
  return filepath_replaced;
}

} // end namespace vision_utils

#endif // REPLACE_HOME_H
