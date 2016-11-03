/*!
  \file        to_uppercase(.h
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

#ifndef TO_UPPERCASE_H
#define TO_UPPERCASE_H
// std includes
#include <string>

namespace vision_utils {

//! convert a string to uppercase
void to_uppercase(std::string & sentence) {
  std::transform(sentence.begin(), sentence.end(), sentence.begin(), toupper);
}

} // end namespace vision_utils

#endif // TO_UPPERCASE_H
