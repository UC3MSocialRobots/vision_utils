/*!
  \file        stats2string.h
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

#ifndef STATS2STRING_H
#define STATS2STRING_H
// std includes
#include <algorithm> // for std::min(), std::max()...
#include <sstream> // for ostringstream
#include <string>
#include <vector>

namespace vision_utils {

template<class T>
inline std::string stats2string(std::vector<T> & data) {
  T min_value = *(std::min_element(data.begin(), data.end()));
  T max_value = *(std::max_element(data.begin(), data.end()));
  double mean, std_dev;
  mean_std_dev(data, mean, std_dev);
  // convert to string
  std::ostringstream ans;
  ans << data.size() << " elts"
      << ", min:" << min_value
      << ", max:" << max_value
      << ", mean:" << mean
      << ", std_dev:" << std_dev;
  return ans.str();
}

} // end namespace vision_utils

#endif // STATS2STRING_H
