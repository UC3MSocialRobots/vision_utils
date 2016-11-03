/*!
  \file        build_languages_map.h
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

#ifndef BUILD_LANGUAGES_MAP_H
#define BUILD_LANGUAGES_MAP_H
// std includes
#include <map>
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

typedef std::string CountryDomain;
typedef int LanguageId;
typedef std::map<LanguageId, CountryDomain> LanguageMap;
typedef LanguageMap::value_type LanguagePair;

enum { // sorted list please!
  LANGUAGE_UNKNOWN = -1,
  LANGUAGE_LAST_USED = 0,
  //
  LANGUAGE_AFRIKAANS,
  LANGUAGE_ALBANIAN,
  LANGUAGE_ARABIC,
  LANGUAGE_BULGARIAN,
  LANGUAGE_CATALAN,
  LANGUAGE_CHINESE_SIMPLIFIED,
  LANGUAGE_CROATIAN,
  LANGUAGE_CZECH,
  LANGUAGE_DANISH,
  LANGUAGE_DUTCH,
  LANGUAGE_ENGLISH,
  LANGUAGE_ESTONIAN,
  LANGUAGE_FINNISH,
  LANGUAGE_FRENCH,
  LANGUAGE_GERMAN,
  LANGUAGE_GREEK,
  LANGUAGE_HAITIAN_CREOLE,
  LANGUAGE_HINDI,
  LANGUAGE_HUNGARIAN,
  LANGUAGE_ICELANDIC,
  LANGUAGE_INDONESIAN,
  LANGUAGE_ITALIAN,
  LANGUAGE_JAPANESE,
  LANGUAGE_KOREAN,
  LANGUAGE_LATVIAN,
  LANGUAGE_LITUANIAN,
  LANGUAGE_MALAY,
  LANGUAGE_MACEDONIAN,
  LANGUAGE_NORWEGIAN,
  LANGUAGE_PERSIAN,
  LANGUAGE_POLISH,
  LANGUAGE_PORTUGUESE,
  LANGUAGE_ROMANIAN,
  LANGUAGE_RUSSIAN,
  LANGUAGE_SERBIAN,
  LANGUAGE_SLOVAK,
  LANGUAGE_SLOVENIAN,
  LANGUAGE_SPANISH,
  LANGUAGE_SWAHILI,
  LANGUAGE_SWEDISH,
  LANGUAGE_THAI,
  LANGUAGE_TURKISH,
  LANGUAGE_VIETNAMESE,
  LANGUAGE_URDU,
  LANGUAGE_WELSH
};

////////////////////////////////////////////////////////////////////////////////
/*! a function to populate a map with the Google languages */
void build_languages_map(LanguageMap & map) {
  // printf("build_language_map()\n");
  map.insert( LanguagePair(LANGUAGE_UNKNOWN, "??") );
  map.insert( LanguagePair(LANGUAGE_AFRIKAANS, "af") );
  map.insert( LanguagePair(LANGUAGE_ALBANIAN, "sq") );
  map.insert( LanguagePair(LANGUAGE_ARABIC, "ar") );
  map.insert( LanguagePair(LANGUAGE_BULGARIAN, "bg") );
  map.insert( LanguagePair(LANGUAGE_CATALAN, "ca") );
  map.insert( LanguagePair(LANGUAGE_CHINESE_SIMPLIFIED, "zh-CN") );
  map.insert( LanguagePair(LANGUAGE_CROATIAN, "hr") );
  map.insert( LanguagePair(LANGUAGE_CZECH, "cs") );
  map.insert( LanguagePair(LANGUAGE_DANISH, "da") );
  map.insert( LanguagePair(LANGUAGE_DUTCH, "nl") );
  map.insert( LanguagePair(LANGUAGE_ENGLISH, "en") );
  map.insert( LanguagePair(LANGUAGE_ESTONIAN, "et") );
  map.insert( LanguagePair(LANGUAGE_FINNISH, "fi") );
  map.insert( LanguagePair(LANGUAGE_FRENCH, "fr") );
  map.insert( LanguagePair(LANGUAGE_GERMAN, "de") );
  map.insert( LanguagePair(LANGUAGE_GREEK, "el") );
  map.insert( LanguagePair(LANGUAGE_HAITIAN_CREOLE, "ht") );
  map.insert( LanguagePair(LANGUAGE_HINDI, "hi") );
  map.insert( LanguagePair(LANGUAGE_HUNGARIAN, "hu") );
  map.insert( LanguagePair(LANGUAGE_ICELANDIC, "is") );
  map.insert( LanguagePair(LANGUAGE_INDONESIAN, "ms") );
  map.insert( LanguagePair(LANGUAGE_ITALIAN, "it") );
  map.insert( LanguagePair(LANGUAGE_JAPANESE, "ja") );
  map.insert( LanguagePair(LANGUAGE_KOREAN, "kr") );
  map.insert( LanguagePair(LANGUAGE_LATVIAN, "lv") );
  map.insert( LanguagePair(LANGUAGE_LITUANIAN, "lt") );
  map.insert( LanguagePair(LANGUAGE_MALAY, "id") );
  map.insert( LanguagePair(LANGUAGE_MACEDONIAN, "mk") );
  map.insert( LanguagePair(LANGUAGE_NORWEGIAN, "no") );
  map.insert( LanguagePair(LANGUAGE_PERSIAN, "fa") );
  map.insert( LanguagePair(LANGUAGE_POLISH, "pl") );
  map.insert( LanguagePair(LANGUAGE_PORTUGUESE, "pt") );
  map.insert( LanguagePair(LANGUAGE_ROMANIAN, "ro") );
  map.insert( LanguagePair(LANGUAGE_RUSSIAN, "ru") );
  map.insert( LanguagePair(LANGUAGE_SERBIAN, "sr") );
  map.insert( LanguagePair(LANGUAGE_SLOVAK, "sk") );
  map.insert( LanguagePair(LANGUAGE_SLOVENIAN, "sl") );
  map.insert( LanguagePair(LANGUAGE_SPANISH, "es") );
  map.insert( LanguagePair(LANGUAGE_SWAHILI, "sw") );
  map.insert( LanguagePair(LANGUAGE_SWEDISH, "sv") );
  map.insert( LanguagePair(LANGUAGE_THAI, "th") );
  map.insert( LanguagePair(LANGUAGE_TURKISH, "tr") );
  map.insert( LanguagePair(LANGUAGE_URDU, "ur") );
  map.insert( LanguagePair(LANGUAGE_VIETNAMESE, "vi") );
  map.insert( LanguagePair(LANGUAGE_WELSH, "cy") );

}

} // end namespace vision_utils

#endif // BUILD_LANGUAGES_MAP_H
