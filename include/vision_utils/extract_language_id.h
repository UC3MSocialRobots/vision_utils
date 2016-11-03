/*!
  \file        extract_language_id.h
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

#ifndef EXTRACT_LANGUAGE_ID_H
#define EXTRACT_LANGUAGE_ID_H
// std includes
#include <string>

namespace vision_utils {

/*!
  extract the language indicator and makes an inverse lookup in the map
  \param ans will be populated
  \return true if success
  \example sentence="en:foo" return LANGUAGE_ENGLISH
  */
bool extract_language_id(const std::string & sentence,
                          const LanguageMap & map,
                          LanguageId & ans) {
  //printf("extract_language_id(sentence:'%s')\n", sentence.c_str());
  std::string country_domain;
  if (!extract_country_domain(sentence, country_domain))
    return false;
  //printf("country_domain:'%s'\n", country_domain.c_str());
  return get_language_id_from_country_domain(map, country_domain, ans);
}

} // end namespace vision_utils

#endif // EXTRACT_LANGUAGE_ID_H
