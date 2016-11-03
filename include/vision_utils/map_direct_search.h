/*!
  \file        map_direct_search.h
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

#ifndef MAP_DIRECT_SEARCH_H
#define MAP_DIRECT_SEARCH_H
// std includes
#include <map>

namespace vision_utils {

/*!
 * Perform a direct search on a map.
 * \param map
 *   The map to be searched.
 * \param search_key
 *   The key we want.
 * \param value_lookup_result (out)
 *   The value corresponding to \a search_key in the map,
 *   if \a search_key is present.
 *   Undetermined otherwise.
 * \return
 *  true if \a search_key was found.
 * \example map=[1->"value1"; 2->"value2"; 3->"value3"; 4->"value4"]
 *  search_key=0: returns false, value_lookup_result not affected
 *  search_key=1: returns true, value_lookup_result="value1"
 *  search_key=2: returns true, value_lookup_result="value2"
 */
template<class _Key, class _Value>
inline bool direct_search(const std::map<_Key, _Value> & map,
                           const _Key & search_key,
                           _Value & value_lookup_result) {
  typename std::map<_Key, _Value>::const_iterator it = map.find(search_key);
  if (it == map.end()) {
    return false;
  }
  value_lookup_result = it->second;
  return true;
} // end direct_search()

} // end namespace vision_utils

#endif // MAP_DIRECT_SEARCH_H
