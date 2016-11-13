/*!
  \file        clean_string_from_weird_chars.h
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

#ifndef CLEAN_STRING_FROM_WEIRD_CHARS_H
#define CLEAN_STRING_FROM_WEIRD_CHARS_H
// std includes
#include <algorithm> // for std::min(), std::max()...
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
// vision_utils
#include <vision_utils/contains_any_letter.h>
#include <vision_utils/load_letter_dict.h>
#include <vision_utils/string_split.h>
#include <vision_utils/to_lowercase.h>

namespace vision_utils {

bool clean_string_from_weird_chars(std::string & sentence_to_clean,
                                   const vision_utils::LanguageId src_language) {
//  printf("clean_string_from_weird_chars(%s)\n",
//            sentence_to_clean.substr(0, std::min(10, (int) sentence_to_clean.size())).c_str());

  // convert to lowercase
  vision_utils::to_lowercase(sentence_to_clean);
  //printf("After to_lowercase:'%s'\n", sentence_to_clean.c_str());

  /*
    *  check if the word contains weird letters
    */
  LetterDict _letters;
  if (load_letter_dict(src_language, _letters)) {
    std::ostringstream sentence_to_clean_no_weird_words;

    // split to lines
    std::vector<std::string> ocr_lines;
    vision_utils::StringSplit(sentence_to_clean, "\n", &ocr_lines);

    // check each line
    for (std::vector<std::string>::const_iterator ocr_line = ocr_lines.begin();
         ocr_line != ocr_lines.end() ; ++ocr_line) {
      std::ostringstream ocr_line_clean;
      // split to words
      std::vector<std::string> ocr_words;
      vision_utils::StringSplit(*ocr_line, " ", &ocr_words);

      // check each word
      for (std::vector<std::string>::const_iterator ocr_word = ocr_words.begin();
           ocr_word != ocr_words.end() ; ++ocr_word) {
        // check if there is a weird char
        bool contains_weird_char = false;
        for (unsigned int letter_idx = 0 ; letter_idx < ocr_word->length() ;
             ++letter_idx) {
          char current_char = ocr_word->at(letter_idx);
          if (_letters.find(current_char) == _letters.end()) {
            contains_weird_char = true;
            break;
          }
        } // end for letter_idx
        if (contains_weird_char == false) {
          ocr_line_clean << *ocr_word << ' ';
          continue;
        } // end if contains_weird_char

        // if we reached here, remove it
        printf("Removing word '%s'\n", ocr_word->c_str());
      } // end for ocr_word

      sentence_to_clean_no_weird_words << ocr_line_clean.str() << std::endl;
    } // end for ocr_line

    sentence_to_clean = sentence_to_clean_no_weird_words.str();
//    printf("After cleaning weird words:'%s'\n",
//              sentence_to_clean.c_str());
  } // end if (load_letter_dict())

  /*
     * check if each line contains at least a character
     */
  if (src_language == vision_utils::LANGUAGE_ENGLISH ||
      src_language == vision_utils::LANGUAGE_SPANISH ||
      src_language == vision_utils::LANGUAGE_FRENCH) {
    // split to lines
    std::ostringstream sentence_to_clean_no_empty_lines;
    std::vector<std::string> ocr_lines;
    vision_utils::StringSplit(sentence_to_clean, "\n", &ocr_lines);

    // check each line
    for (std::vector<std::string>::const_iterator ocr_line = ocr_lines.begin();
         ocr_line != ocr_lines.end() ; ++ocr_line) {
      bool current_line_contains_any_letters =
          vision_utils::contains_any_letter(*ocr_line);
      if (current_line_contains_any_letters)
        sentence_to_clean_no_empty_lines << *ocr_line << "\n";
    }
    sentence_to_clean = sentence_to_clean_no_empty_lines.str();
    //printf("After removing empty lines:'%s'\n", sentence_to_clean.c_str());
  } // end if language = ENGLISH...
  return true;
}

} // end namespace vision_utils

#endif // CLEAN_STRING_FROM_WEIRD_CHARS_H
