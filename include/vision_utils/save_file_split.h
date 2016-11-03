/*!
  \file        save_file_split.h
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

#ifndef SAVE_FILE_SPLIT_H
#define SAVE_FILE_SPLIT_H
// std includes
#include <string>
#include <vector>

namespace vision_utils {

inline void save_file_split(const std::string & filepath,
                            const std::vector<std::string> & content) {
  //printf("save_file_vec('%s')", filepath.c_str());
  save_file(filepath, concatenate_vector(content));
}
} // end namespace vision_utils

} // end namespace vision_utils

#endif // SAVE_FILE_SPLIT_H
