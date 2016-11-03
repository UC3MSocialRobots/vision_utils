/*!
  \file        map_keys_to_string.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/3
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

#ifndef MAP_KEYS_TO_STRING_H
#define MAP_KEYS_TO_STRING_H
// std includes
#include <sstream> // for ostringstream
#include <string>

namespace vision_utils {

/*!
 Convert all the keys of a map to a string
 \param m
    the map
 \return std::string
*/
template< class MapType >
std::string map_keys_to_string(const MapType & m) {
  typedef typename MapType::const_iterator const_iterator;
  std::ostringstream ans_stream;
  for( const_iterator iter = m.begin(), iend = m.end(); iter != iend; ++iter )
    ans_stream << iter->first << ";";
  return ans_stream.str();
}

} // end namespace vision_utils

#endif // MAP_KEYS_TO_STRING_H
