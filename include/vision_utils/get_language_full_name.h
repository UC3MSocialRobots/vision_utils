/*!
  \file        get_language_full_name.h
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

#ifndef GET_LANGUAGE_FULL_NAME_H
#define GET_LANGUAGE_FULL_NAME_H
// std includes
#include <string>

namespace vision_utils {

/*! \return the full name of a language.
  \example "english" for LANGUAGE_ENGLISH */
std::string get_language_full_name(const LanguageId & language_id) {
  //printf("get_language_full_name(%i)\n", language_id);
  switch (language_id) {
  case vision_utils::LANGUAGE_AFRIKAANS: return "afrikaans";
  case vision_utils::LANGUAGE_ALBANIAN: return "albanian";
  case vision_utils::LANGUAGE_ARABIC: return "arabic";
  case vision_utils::LANGUAGE_BULGARIAN: return "bulgarian";
  case vision_utils::LANGUAGE_CATALAN: return "catalan";
  case vision_utils::LANGUAGE_CHINESE_SIMPLIFIED: return "simplified chinese";
  case vision_utils::LANGUAGE_CROATIAN: return "croatian";
  case vision_utils::LANGUAGE_CZECH: return "czech";
  case vision_utils::LANGUAGE_DANISH: return "danish";
  case vision_utils::LANGUAGE_DUTCH: return "dutch";
  case vision_utils::LANGUAGE_ENGLISH: return "english";
  case vision_utils::LANGUAGE_ESTONIAN: return "estonian";
  case vision_utils::LANGUAGE_FINNISH: return "finnish";
  case vision_utils::LANGUAGE_FRENCH: return "french";
  case vision_utils::LANGUAGE_GERMAN: return "german";
  case vision_utils::LANGUAGE_GREEK: return "greek";
  case vision_utils::LANGUAGE_HAITIAN_CREOLE: return "haitian_creole";
  case vision_utils::LANGUAGE_HINDI: return "hindi";
  case vision_utils::LANGUAGE_HUNGARIAN: return "hungarian";
  case vision_utils::LANGUAGE_ICELANDIC: return "icelandic";
  case vision_utils::LANGUAGE_INDONESIAN: return "indonesian";
  case vision_utils::LANGUAGE_ITALIAN: return "italian";
  case vision_utils::LANGUAGE_JAPANESE: return "japanese";
  case vision_utils::LANGUAGE_KOREAN: return "korean";
  case vision_utils::LANGUAGE_LATVIAN: return "latvian";
  case vision_utils::LANGUAGE_LITUANIAN: return "lituanian";
  case vision_utils::LANGUAGE_MALAY: return "malay";
  case vision_utils::LANGUAGE_MACEDONIAN: return "macedonian";
  case vision_utils::LANGUAGE_NORWEGIAN: return "norwegian";
  case vision_utils::LANGUAGE_PERSIAN: return "persian";
  case vision_utils::LANGUAGE_POLISH: return "polish";
  case vision_utils::LANGUAGE_PORTUGUESE: return "portuguese";
  case vision_utils::LANGUAGE_ROMANIAN: return "romanian";
  case vision_utils::LANGUAGE_RUSSIAN: return "russian";
  case vision_utils::LANGUAGE_SERBIAN: return "serbian";
  case vision_utils::LANGUAGE_SLOVAK: return "slovak";
  case vision_utils::LANGUAGE_SLOVENIAN: return "slovenian";
  case vision_utils::LANGUAGE_SPANISH: return "spanish";
  case vision_utils::LANGUAGE_SWAHILI: return "swahili";
  case vision_utils::LANGUAGE_SWEDISH: return "swedish";
  case vision_utils::LANGUAGE_THAI: return "thai";
  case vision_utils::LANGUAGE_TURKISH: return "turkish";
  case vision_utils::LANGUAGE_VIETNAMESE: return "vietnamese";
  case vision_utils::LANGUAGE_URDU: return "urdu";
  case vision_utils::LANGUAGE_WELSH: return "welsh";
  default:
    return "an unknwon language";
  } // end switch language_id
}

} // end namespace vision_utils

#endif // GET_LANGUAGE_FULL_NAME_H
