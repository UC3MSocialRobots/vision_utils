/*!
  \file        build_given_language_in_multilanguage_line.h
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

#ifndef BUILD_GIVEN_LANGUAGE_IN_MULTILANGUAGE_LINE_H
#define BUILD_GIVEN_LANGUAGE_IN_MULTILANGUAGE_LINE_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
// vision_utils
#include <vision_utils/string_split.h>
#include <vision_utils/extract_language_id.h>
#include <vision_utils/find_given_language_in_multilanguage_line.h>
#include <vision_utils/translate.h>

namespace vision_utils {

/*! first, try to see if the given language is given
   *  in \arg textMultiLanguage.
   * If it is not, translate from one of the given languages,
   * by default trying with the english version (en:),
   * if english not available with whatever is given.
   * \example "es:hola|en:hi|fr:bonjour"
   *
   * Multiple instances of a given language are supported,
   * one of these will be chosen randomly.
   * \example textMultiLanguage="en:Hello|en:Hi|fr:Salut"
   *            if (target_language == LANGUAGE_ENGLISH),
   *              will return randomly "Hello" or "Hi"
   *            if (target_language == LANGUAGE_FRENCH)
   *              will return "Salut"
   */
bool build_given_language_in_multilanguage_line(
    const std::string & textMultiLanguage,
    const LanguageId target_language,
    const LanguageMap & languages_map,
    std::string & ans) {
  std::vector<std::string> versions;
  vision_utils::StringSplit(textMultiLanguage, "|", &versions);

  std::string sentence_good_language;
  // first search in the wanted language
  bool was_found = vision_utils::find_given_language_in_multilanguage_line(
        versions, target_language, languages_map, sentence_good_language);
  if (was_found) {
    printf("The wanted language %i was supplied by the user.\n",
             target_language);
    ans = sentence_good_language;
    return true;
  }

  // if not present, search english and translate it
  was_found = vision_utils::find_given_language_in_multilanguage_line(
        versions, vision_utils::LANGUAGE_ENGLISH, languages_map, sentence_good_language);
  if (was_found) {
    printf("LANGUAGE_ENGLISH was supplied by the user. "
             "Translating from that to %i\n",
             target_language);
    ans = vision_utils::translate(sentence_good_language,
                                  vision_utils::LANGUAGE_ENGLISH,
                                  target_language);
    return true;
  }

  // otherwise, translate the first that comes
  for(std::vector<std::string>::const_iterator version = versions.begin();
      version != versions.end() ; ++version) {

    // extract the language key
    vision_utils::LanguageId version_language_id;
    bool language_id_found;
    language_id_found = vision_utils::extract_language_id
        (*version, languages_map, version_language_id);
    if (!language_id_found)
      continue;

    // extract the corresponding sentence
    was_found = vision_utils::find_given_language_in_multilanguage_line(
          versions, version_language_id, languages_map, sentence_good_language);
    if (was_found) {
      printf("%i was supplied by the user. "
               "Translating from that ('%s') to %i\n",
               version_language_id,
               sentence_good_language.c_str(),
               target_language);
      ans = vision_utils::translate(sentence_good_language,
                                    version_language_id,
                                    target_language);
      return true;
    }
  } // end loop version

  // if everything failed, assume it is current language
  /*printf("I couldn't detect any known language in the sentence '%s'. "
              "Did you respect the sayTextNL() syntax ?\n",
              textMultiLanguage.c_str());*/

  ans = textMultiLanguage;
  // ans = versions.front();
  return false;
}

} // end namespace vision_utils

#endif // BUILD_GIVEN_LANGUAGE_IN_MULTILANGUAGE_LINE_H
