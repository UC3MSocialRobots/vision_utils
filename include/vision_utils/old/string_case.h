/*!
  \file
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/8/22

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
#ifndef STRING_CASE_H
#define STRING_CASE_H

#include <string>

namespace vision_utils {
//cut:random_string
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

////////////////////////////////////////////////////////////////////////////////
//cut:to_lowercase(
//! convert a string to lowercase
void to_lowercase(std::string & sentence) {
  std::transform(sentence.begin(), sentence.end(), sentence.begin(), tolower);
  find_and_replace(sentence, "Ó", "ó");
}

////////////////////////////////////////////////////////////////////////////////
//cut:to_uppercase(
//! convert a string to uppercase
void to_uppercase(std::string & sentence) {
  std::transform(sentence.begin(), sentence.end(), sentence.begin(), toupper);
}

////////////////////////////////////////////////////////////////////////////////
//cut:contains_any_letter(
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


#endif // STRING_CASE_H

