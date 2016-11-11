/*!
  \file        remove_accents.h
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

#ifndef REMOVE_ACCENTS_H
#define REMOVE_ACCENTS_H
// std includes
#include <string>

namespace vision_utils {

/*! change all accented letters to normal letters
 ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ
 \example eéèêaàäâçc would become eeeeaaaacc */
inline std::string remove_accents(const std::string & str) {
  std::string ans = str;
  clean_spanish_chars(ans);
  find_and_replace(ans, "â", "a");
  find_and_replace(ans, "Â", "A");
  find_and_replace(ans, "à", "a");
  find_and_replace(ans, "À", "A");
  find_and_replace(ans, "è", "e");
  find_and_replace(ans, "È", "E");
  find_and_replace(ans, "ê", "e");
  find_and_replace(ans, "Ê", "E");
  find_and_replace(ans, "î", "i");
  find_and_replace(ans, "Î", "I");
  find_and_replace(ans, "ô", "o");
  find_and_replace(ans, "Ô", "O");
  find_and_replace(ans, "ù", "u");
  find_and_replace(ans, "Ù", "U");
  find_and_replace(ans, "û", "u");
  find_and_replace(ans, "Û", "U");
  find_and_replace(ans, "ü", "u");
  find_and_replace(ans, "Ü", "U");
  return ans;
}

} // end namespace vision_utils

#endif // REMOVE_ACCENTS_H
