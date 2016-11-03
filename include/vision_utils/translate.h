/*!
  \file        translate.h
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

#ifndef TRANSLATE_H
#define TRANSLATE_H
// std includes
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
// vision_utils
#include <vision_utils/extract_from_tags.h>
#include <vision_utils/find_and_replace.h>
#include <vision_utils/retrieve_url.h>

namespace vision_utils {

//! \return sentence_to_translate if fails
std::string translate(const std::string & sentence_to_translate,
                      const LanguageId & language_orig,
                      const LanguageId & language_dest) {

  printf("translate(sentence:'%s'' : %s -> %s)\n",
           sentence_to_translate.c_str(),
           get_language_full_name(language_orig).c_str(),
           get_language_full_name(language_dest).c_str());

  // don't translate if the languages are identical
  if (language_orig == language_dest)
    return sentence_to_translate;

  // prepair the string for http encoding
  std::string sentence_to_translate_url = sentence_to_translate;
  vision_utils::find_and_replace(sentence_to_translate_url,
                                 " ", "%20");
  vision_utils::find_and_replace(sentence_to_translate_url,
                                 "\n", "%0A");

  // find the ides
  LanguageMap language_map;
  build_languages_map(language_map);
  // autodetect language if needed
  LanguageId language_orig_detect = language_orig;
  if (language_orig == LANGUAGE_UNKNOWN)
    language_orig_detect = detect_language(sentence_to_translate);
  if (language_orig_detect == LANGUAGE_UNKNOWN) { // detection failed -> quit
    printf("Impossible to detect the language of '%s'\n",
             sentence_to_translate.c_str());
    return sentence_to_translate;
  }

  // don't translate if the languages are identical
  if (language_orig_detect == language_dest)
    return sentence_to_translate;

  CountryDomain prefix_orig, prefix_dest;
  if (!get_country_domain_from_language_id
      (language_map, language_orig_detect, prefix_orig)
      || !get_country_domain_from_language_id(language_map, language_dest, prefix_dest))
    return sentence_to_translate;

  // build the URL
  std::ostringstream url;

#ifdef USE_GOOGLE_API_V1
  //https://ajax.googleapis.com/ajax/services/language/translate?v=1.0&q=Hello,%20my%20friend!&langpair=en%7Ces
  url << "https://ajax.googleapis.com/ajax/services/language/translate?v=1.0";
  url << "&q=" << sentence_to_translate_url;
  url << "&langpair=" << prefix_orig << "|" << prefix_dest;
#endif // USE_GOOGLE_API_V1

#ifdef USE_GOOGLE_API_V2
  url << "https://www.googleapis.com/language/translate/v2?";
  url << "key=" << GOOGLE_API_KEY;
  url << "&source=" << prefix_orig;
  url << "&target=" << prefix_dest;
  url << "&prettyprint=false";
  url << "&callback=handleResponse";
  url << "&q=" << sentence_to_translate_url;
#endif // USE_GOOGLE_API_V2

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  // languages available at:
  // http://api.microsofttranslator.com/V2/Http.svc/GetLanguagesForTranslate?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B
  // example:
  // http://api.microsofttranslator.com/v2/Http.svc/Translate?appId=6844AE3580856D2EC7A64C79F55F11AA47CB961B&text=Hello,%20my%20friend!&from=en&to=fr
  url << "http://api.microsofttranslator.com/v2/Http.svc/Translate?";
  url << "appId=" << MICROSOFT_APPLICATION_ID;
  url << "&text=" << sentence_to_translate_url;
  url << "&from=" << prefix_orig;
  url << "&to=" << prefix_dest;
#endif // USE_MICROSOFT_TRANSLATOR_V2
  // send the request
  std::string server_answer;
  vision_utils::retrieve_url(url.str(), server_answer);
  //printf("server_answer:%s\n", server_answer.c_str());

  // extract from the tags
  int search_pos = 0;
#if defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)
  std::string ans = vision_utils::extract_from_tags
      (server_answer, "\"translatedText\":\"", "\"", search_pos);
#endif // defined(USE_GOOGLE_API_V1) || defined(USE_GOOGLE_API_V2)

#ifdef USE_MICROSOFT_TRANSLATOR_V2
  std::string ans = vision_utils::extract_from_tags
      (server_answer, ">", "<", search_pos);
#endif // USE_MICROSOFT_TRANSLATOR_V2

  // replace special characters
  vision_utils::find_and_replace(ans, "&#39;", "'");
  vision_utils::find_and_replace(ans, "&quot;", "'");

  return ans;
}

////////////////////////////////////////////////////////////////////////////////
//// functions for string processing

} // end namespace vision_utils

#endif // TRANSLATE_H
