/*!
  \file        get_country_domain_from_language_id.h
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

#ifndef GET_COUNTRY_DOMAIN_FROM_LANGUAGE_ID_H
#define GET_COUNTRY_DOMAIN_FROM_LANGUAGE_ID_H
// std includes
#include <stdio.h> // for printf(), etc
// vision_utils
#include <vision_utils/direct_search.h>
#include <vision_utils/map_keys_to_string.h>

namespace vision_utils {

/*! make a direct search in the language map
  \return true if success */
bool get_country_domain_from_language_id(const LanguageMap & languages_map,
                                         const LanguageId & language_id,
                                         CountryDomain & ans)
{
  ans = "??";
  if (!vision_utils::direct_search(languages_map, language_id, ans)) {
    printf("Cannot resolve language %i to a string, "
           "is it a proper ID? (supported:%s) "
           "did u build the map using build_languages_map()?\n",
           language_id,
           vision_utils::map_keys_to_string(languages_map).c_str());
    return false;
  }
  return true;
}

} // end namespace vision_utils

#endif // GET_COUNTRY_DOMAIN_FROM_LANGUAGE_ID_H
