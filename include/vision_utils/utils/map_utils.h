/*!
  \file        map_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/5/28

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

Some useful functions for playing around with std::map objects.

*/

#ifndef MAP_UTILS_H
#define MAP_UTILS_H

#include <map>

namespace map_utils {

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

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

/*!
 * Dumps all keys of a map into a container, for instance a std::vector.
 * \param m
 *   The map to be dumped.
 * \param v
 *   The vector to be filled with keys.
 * \example map=[1->"value1"; 2->"value2"; 3->"value3"; 4->"value4"]
 *   v=[1;2;3;4]
 */
template <typename M, typename V>
inline void map_keys_to_container( const  M & m, V & v ) {
  v.reserve(m.size());
  v.clear();
  for( typename M::const_iterator it = m.begin(); it != m.end(); ++it ) {
    v.push_back( it->first );
  }
} // end map_keys_to_container

////////////////////////////////////////////////////////////////////////////////

/*!
 * Dumps all values of a map into a container, for instance a std::vector.
 * \param m
 *   The map to be dumped.
 * \param v
 *   The vector to be filled with values.
 * \example map=[1->"value1"; 2->"value2"; 3->"value3"; 4->"value4"]
 *   v=["value1";"value2";"value3";"value4"]
 */
template <typename M, typename V>
inline void map_values_to_container( const  M & m, V & v ) {
  v.reserve(m.size());
  v.clear();
  for( typename M::const_iterator it = m.begin(); it != m.end(); ++it ) {
    v.push_back( it->second );
  }
} // end map_values_to_container

} // end namespace map_utils
#endif // MAP_UTILS_H
