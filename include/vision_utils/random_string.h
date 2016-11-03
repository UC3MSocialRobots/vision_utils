/*!
  \file        random_string.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/3
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

#ifndef RANDOM_STRING_H
#define RANDOM_STRING_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

/*! Generate a random string made of characters in the range a -> z
 * \param length the size of the wanted string
 */
std::string random_string(const int length) {
  //printf("random_string(%i)\n", length);

  // init the random generator
  //srand( time(NULL));

  // add letters in a loop
  std::ostringstream buffer;
  for (int char_idx = 0; char_idx< length; ++char_idx) {
      char new_char = (rand() % 26) + 'a';
      buffer << new_char;
    }

  return buffer.str();
}

} // end namespace vision_utils

#endif // RANDOM_STRING_H
