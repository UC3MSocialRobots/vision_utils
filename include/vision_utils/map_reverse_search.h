/*!
  \file        map_reverse_search.h
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

#ifndef MAP_REVERSE_SEARCH_H
#define MAP_REVERSE_SEARCH_H
// std includes
#include <map>

namespace vision_utils {

/*!
 * Perform an inverse search on a map.
 * \param map
 *   The map to be searched.
 * \param search_value
 *   The value we want.
 * \param key_lookup_result (out)
 *   The key corresponding to \a search_value in the map,
 *   if \a search_value is present.
 *   Undetermined otherwise.
 * \return
 *  true if \a search_value was found.
 * \example map=[1->"value1"; 2->"value2"; 3->"value3"; 4->"value4"]
 *  search_value="value0": returns false, key_lookup_result not affected
 *  search_value="value1": returns true, key_lookup_result=1
 *  search_value="value2": returns true, key_lookup_result=2
 */
template<class _Key, class _Value>
inline bool reverse_search (const std::map<_Key, _Value> & map,
                            const _Value & search_value,
                            _Key & key_lookup_result) {
  for (typename std::map<_Key, _Value>::const_iterator map_iterator = map.begin();
       map_iterator != map.end();
       map_iterator ++) {
    if (map_iterator->second == search_value) {
      key_lookup_result = map_iterator->first;
      return true;
    }
  } // end loop map_iterator
  return false;
} // end reverse_search()

} // end namespace vision_utils

#endif // MAP_REVERSE_SEARCH_H
