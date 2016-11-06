/*!
  \file        map_values_to_container.h
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

#ifndef MAP_VALUES_TO_CONTAINER_H
#define MAP_VALUES_TO_CONTAINER_H
// std includes
#include <vector>

namespace vision_utils {

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

} // end namespace vision_utils

#endif // MAP_VALUES_TO_CONTAINER_H
