/*!
  \file        find_given_language_in_multilanguage_line.h
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

#ifndef FIND_GIVEN_LANGUAGE_IN_MULTILANGUAGE_LINE_H
#define FIND_GIVEN_LANGUAGE_IN_MULTILANGUAGE_LINE_H
// std includes
#include <string>
#include <vector>

namespace vision_utils {

/*!
  \param versions a vector of strings,
  each string is the country domain and the sentence in this language.
  Multiple instances of a given language are supported,
  one of these will be chosen randomly.
  \example versions={"en:Hello","en:Hi","fr:Salut"}
           if (target_language == LANGUAGE_ENGLISH),
             will return randomly "Hello" or "Hi"
           if (target_language == LANGUAGE_FRENCH)
             will return "Salut"
  \return true if success

  \example if target_language == LANGUAGE_ENGLISH,
  will search a sentence that starts with "en:"
  */
bool find_given_language_in_multilanguage_line(
    const std::vector<std::string> & versions,
    const LanguageId target_language,
    const LanguageMap & map,
    std::string & ans) {

  // check in each version if it was found
  CountryDomain current_version_lang_prefix;
  CountryDomain target_country_domain;
  if (!get_country_domain_from_language_id
      (map, target_language, target_country_domain))
    return false;

  std::vector<std::string> good_versions;
  for(std::vector<std::string>::const_iterator current_version = versions.begin();
      current_version != versions.end() ; ++current_version) {
    if (extract_country_domain(*current_version, current_version_lang_prefix)
        && current_version_lang_prefix == target_country_domain) {
      // we remove the prefix and the colon
      good_versions.push_back(current_version->substr(1 + target_country_domain.size()));
    }
  } // end loop versions

  if (good_versions.empty())
    return false;

  ans = good_versions.at(rand() % good_versions.size());
  return true;
}

} // end namespace vision_utils

#endif // FIND_GIVEN_LANGUAGE_IN_MULTILANGUAGE_LINE_H
