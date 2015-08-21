/*!
  \file        string_casts_stl.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/2

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

Some useful functions for casting STL objects to strings.
 */

#ifndef STRING_CASTS_STL_H
#define STRING_CASTS_STL_H

#include <string>
#include <sstream>

namespace StringUtils {

/*! Convert a vector to a fancy string
 \param vector
    anything that can be accesed with iterators
 \return std::string the representation of this vector, separated by ;
 */
template<class _Iterable>
inline std::string iterable_to_string(const _Iterable & obj) {
  if (obj.size() == 0)
    return "{}";
  std::ostringstream ans_stream;
  ans_stream << "[";
  typename _Iterable::const_iterator it = obj.begin();
  while (it != obj.end()) {
    ans_stream << *it++;
    if (it!= obj.end())
      ans_stream << "; ";
  }
  ans_stream << "]";
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

template<class _Iterable>
inline std::string iterable_to_int_string(const _Iterable & obj) {
  if (obj.size() == 0)
    return "{}";
  std::ostringstream ans_stream;
  ans_stream << "[";
  typename _Iterable::const_iterator it = obj.begin();
  while (it != obj.end()) {
    ans_stream << (int) *it++;
    if (it!= obj.end())
      ans_stream << "; ";
  }
  ans_stream << "]";
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

/*! Convert a obj to a string, concatenating all values
 \param obj
    anything that can be accesed using []
 \return std::string the representation of this obj
 */
template<class _Accessible>
inline std::string accessible_to_string(const _Accessible & obj) {
  if (obj.size() == 0)
    return "{}";
  std::ostringstream ans_stream;
  ans_stream << "[";
  for (unsigned int idx = 0; idx < obj.size(); ++idx)
    ans_stream << obj[idx] <<  (idx < obj.size() - 1 ? "; " : "");
  ans_stream << "]";
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

/*! Convert a obj to a string, concatenating all values with no spaces
 \param obj
    anything that can be accesed using []
 \return std::string the representation of this obj
 */
template<class _Accessible>
inline std::string accessible_to_string_nospaces(const _Accessible & obj) {
  std::ostringstream ans_stream;
  for (unsigned int idx = 0; idx < obj.size(); ++idx)
    ans_stream << obj[idx];
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

/*!
 Convert a map to a string
 \param m
    the map
 \return std::string
*/
template< class MapType >
std::string map_to_string(const MapType & m) {
  std::ostringstream ans_stream;
  for( typename MapType::const_iterator iter = m.begin(), iend = m.end();
       iter != iend; ++iter )
    ans_stream << iter->first << "->" << iter->second << ";";
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

/*!
 Convert all the values of a map to a string
 \param m
    the map
 \return std::string
*/
template< class MapType >
std::string map_values_to_string(const MapType & m) {
  typedef typename MapType::const_iterator const_iterator;
  std::ostringstream ans_stream;
  for( const_iterator iter = m.begin(), iend = m.end(); iter != iend; ++iter )
    ans_stream << iter->second << ";";
  return ans_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

} // end namespace StringUtils

#endif // STRING_CASTS_STL_H
