/*!
  \file        extract_country_domain.h
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

#ifndef EXTRACT_COUNTRY_DOMAIN_H
#define EXTRACT_COUNTRY_DOMAIN_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

/*!
  extract the language indicator at the beginning of the sentence
  \param sentence a sentence starting with an indicator
  \param ans will be populated
  \return true if success
  \example extract_language_id("en:foo", ans)
  will return true and ans = "en"
  */
bool extract_country_domain(const std::string & sentence,
                             CountryDomain & ans) {

  size_t sep_pos = sentence.find(':');
  if ( sep_pos == std::string::npos ) {
    /* printf("The language version '%s' does not respect the syntax.\n",
                 sentence.c_str());*/
    return false;
  }

  ans = sentence.substr(0, sep_pos);
  return true;

}

} // end namespace vision_utils

#endif // EXTRACT_COUNTRY_DOMAIN_H
