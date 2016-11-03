/*!
  \file        load_letter_dict.h
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

#ifndef LOAD_LETTER_DICT_H
#define LOAD_LETTER_DICT_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>
// vision_utils
#include <vision_utils/img_path.h>
#include <vision_utils/retrieve_file.h>

namespace vision_utils {

typedef std::set<char> LetterDict;

bool load_letter_dict(const vision_utils::LanguageId &src_language, LetterDict & dict) {
  printf("Loading the dictionary for the language %i\n", src_language);
  std::string dict_filename = "";
  if (src_language == vision_utils::LANGUAGE_ENGLISH)
    dict_filename = IMG_DIR "hangman/english.characters";
  else if (src_language == vision_utils::LANGUAGE_FRENCH)
    dict_filename = IMG_DIR "hangman/spanish.characters";
  else if (src_language == vision_utils::LANGUAGE_SPANISH)
    dict_filename = IMG_DIR "hangman/spanish.characters";
  else {
    printf("t here is no letter list for language %i!\n", src_language);
    return false;
  }

  printf("load_letter_dict('%s')\n", dict_filename.c_str());

  // read the file as a string
  std::string file_str;
  if (!vision_utils::retrieve_file(dict_filename, file_str))
    return false;
  // push all the letters in the set
  dict.clear();
  for (unsigned int letter_idx = 0 ; letter_idx < file_str.length() ;
       ++letter_idx)
    dict.insert( file_str.at(letter_idx) );
  return true;
}

} // end namespace vision_utils

#endif // LOAD_LETTER_DICT_H
