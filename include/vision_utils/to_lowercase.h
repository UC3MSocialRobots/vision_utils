/*!
  \file        to_lowercase(.h
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

#ifndef TO_LOWERCASE_H
#define TO_LOWERCASE_H
// std includes
#include <string>
#include <vision_utils/find_and_replace.h>

namespace vision_utils {

//! convert a string to lowercase
void to_lowercase(std::string & sentence) {
  std::transform(sentence.begin(), sentence.end(), sentence.begin(), tolower);
  find_and_replace(sentence, "Ó", "ó");
}

} // end namespace vision_utils

#endif // TO_LOWERCASE_H
