/*!
  \file        titlemaps.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/12

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

#ifndef TITLEMAPS_H
#define TITLEMAPS_H

#include "vision_utils/utils/string_casts.h"

namespace titlemaps {

typedef std::string (*Map)(const uint);

//! \example 0->"A", 1->"B", etc.
inline std::string int_to_uppercase_letter(const uint idx) {
  return StringUtils::cast_to_string((char) ('A' + idx));
}

////////////////////////////////////////////////////////////////////////////////

//! \example 0->"a", 1->"b", etc.
inline std::string int_to_lowercase_letter(const uint idx) {
  return StringUtils::cast_to_string((char) ('a' + idx));
}

////////////////////////////////////////////////////////////////////////////////

//! \example 0->"1", 1->"2", etc.
inline std::string int_to_number(const uint idx) {
  return StringUtils::cast_to_string(1 + idx);
}

} // end namespace titlemaps

#endif // TITLEMAPS_H
