/*!
  \file        get_language_id_from_country_domain.h
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

#ifndef GET_LANGUAGE_ID_FROM_COUNTRY_DOMAIN_H
#define GET_LANGUAGE_ID_FROM_COUNTRY_DOMAIN_H
// std includes
#include <stdio.h> // for printf(), etc
// vision_utils
#include <vision_utils/map_values_to_string.h>
#include <vision_utils/reverse_search.h>

namespace vision_utils {

/*! make an inversed search in the language map
  \return true if success */
bool get_language_id_from_country_domain(const LanguageMap & languages_map,
                                         const CountryDomain & country_domain,
                                         LanguageId & ans)
{
  //printf("get_language_id_from_country_domain(country_domain:'%s')\n", country_domain.c_str());
  ans = LANGUAGE_UNKNOWN;
  if (!vision_utils::reverse_search(languages_map, country_domain, ans)) {
    printf("Cannot resolve country_domain '%s' to an int, "
           "is it a proper domain? (supported:%s) "
           "did u build the map using build_languages_map()?\n",
           country_domain.c_str(),
           vision_utils::map_values_to_string(languages_map).c_str());
    return false;
  }
  return true;
}

} // end namespace vision_utils

#endif // GET_LANGUAGE_ID_FROM_COUNTRY_DOMAIN_H
