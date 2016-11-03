/*!
  \file        detect_language.h
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

#ifndef DETECT_LANGUAGE_H
#define DETECT_LANGUAGE_H
// std includes
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
// vision_utils
#include <vision_utils/extract_from_tags.h>
#include <vision_utils/find_and_replace.h>
#include <vision_utils/retrieve_url.h>

namespace vision_utils {

//! here is defined the active translator engine
//#define USE_GOOGLE_API_V1 // deprecated since june 2011
//#define USE_GOOGLE_API_V2 // costly since december 2011
#define USE_MICROSOFT_TRANSLATOR_V2 // aaaargh, microsoft!

#ifdef USE_GOOGLE_API_V2
#define GOOGLE_API_KEY "AIzaSyDugyKvZraqZjIh9038_HZZewrVQu1NwQ0";
#endif // USE_GOOGLE_API_V2

#ifdef USE_MICROSOFT_TRANSLATOR_V2
#define MICROSOFT_APPLICATION_ID "6844AE3580856D2EC7A64C79F55F11AA47CB961B";
#endif // USE_MICROSOFT_TRANSLATOR_V2


/*! calls the google appi to detect the language */
LanguageId detect_language(const std::string & sentence_to_translate) {
  printf("detect_language('%s')\n", sentence_to_translate.c_str());

  // prepair the string for http encoding
  std::string sentence_to_translate_url = sentence_to_translate;
  vision_utils::find_and_replace(sentence_to_translate_url,
                                 " ", "%20");
  vision_utils::find_and_replace(sentence_to_translate_url,
                                 "\n", "%0A");

  // build the URL
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  std::ostringstream url;
  url << "https://www.googleapis.com/language/translate/v2/detect?";
  url << "key=" << GOOGLE_API_KEY;
  url << "&prettyprint=false";
  url << "&q=" << sentence_to_translate_url;
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  // http://api.microsofttranslator.com/v2/Http.svc/Detect?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B&text=Hello,%20my%20friend!
  std::ostringstream url;
  url << "http://api.microsofttranslator.com/v2/Http.svc/Detect?";
  url << "appId=" << MICROSOFT_APPLICATION_ID;
  url << "&text=" << sentence_to_translate_url;
#endif // USE_MICROSOFT_TRANSLATOR_V2


  // send the request
  std::string server_answer;
  vision_utils::retrieve_url(url.str(), server_answer);

  // extract from the tags
  int search_pos = 0;
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  CountryDomain country_domain = vision_utils::extract_from_tags(server_answer,
                                                                 "\"language\":\"",
                                                                 "\"",
                                                                 search_pos);
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  CountryDomain country_domain = vision_utils::extract_from_tags
      (server_answer, ">", "<", search_pos);
#endif // USE_MICROSOFT_TRANSLATOR_V2

  // convert the string to a language id
  LanguageMap language_map;
  build_languages_map(language_map);
  LanguageId lang_id;
  if (!get_language_id_from_country_domain(language_map, country_domain , lang_id))
    return LANGUAGE_UNKNOWN;
  return lang_id;

}

} // end namespace vision_utils

#endif // DETECT_LANGUAGE_H
