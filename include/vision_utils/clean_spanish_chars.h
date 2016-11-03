/*!
  \file        clean_spanish_chars.h
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

#ifndef CLEAN_SPANISH_CHARS_H
#define CLEAN_SPANISH_CHARS_H
// std includes
#include <string>

namespace vision_utils {

/*! remove all the spanish characters in a string, that is
 * ¿ -> "", ¡ -> "",
 * à -> a,  á -> a,  é -> e, í -> i,
 * ó -> o,  ú -> u,  ü -> u, ñ -> ny
 * \param string the string to clean
 */
inline void clean_spanish_chars(std::string& string) {
  find_and_replace(string, "¿", "");
  find_and_replace(string, "¡", "");
  find_and_replace(string, "á", "a");
  find_and_replace(string, "Á", "A");
  find_and_replace(string, "ç", "c");
  find_and_replace(string, "é", "e");
  find_and_replace(string, "É", "E");
  find_and_replace(string, "í", "i");
  find_and_replace(string, "Í", "I");
  find_and_replace(string, "ó", "o");
  find_and_replace(string, "Ó", "O");
  find_and_replace(string, "ú", "u");
  find_and_replace(string, "Ú", "U");
  find_and_replace(string, "ñ", "ny");
  find_and_replace(string, "Ñ", "NY");
}

} // end namespace vision_utils

#endif // CLEAN_SPANISH_CHARS_H
