/*!
  \file        contains_any_letter(.h
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

#ifndef CONTAINS_ANY_LETTER_H
#define CONTAINS_ANY_LETTER_H
// std includes
#include <string>
#include <vision_utils/to_lowercase.h>

namespace vision_utils {

//! return true if it contains a letter in A-Z || a-z
bool contains_any_letter(const std::string & sentence) {
  std::string sentence_lower = sentence;
  to_lowercase(sentence_lower);
  for (unsigned int char_idx = 0; char_idx< sentence_lower.size(); ++char_idx) {
      // 'a' = 97, 'z'=122
      int ascii = (int) sentence_lower.at(char_idx);
      if (ascii >= 97 && ascii <= 122)
        return true;
    }
  return false;
}

} // end namespace vision_utils

#endif // CONTAINS_ANY_LETTER_H
