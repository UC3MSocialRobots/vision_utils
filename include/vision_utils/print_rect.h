/*!
  \file        print_rect.h
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

#ifndef PRINT_RECT_H
#define PRINT_RECT_H
// std includes
#include <sstream> // for ostringstream
#include <string>
#include <vector>

namespace vision_utils {

/*!
 * print a rectangle in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Rect>
inline std::string print_rect(const Rect & p) {
  std::ostringstream ans;
  ans << "(" << p.x << ", " << p.y << ")+("
      << p.width << ", " << p.height << ")";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * print a vector of rectangles in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Rect>
inline std::string print_rects(const std::vector<Rect> & v) {
  std::ostringstream ans;
  for (unsigned int rec_idx = 0; rec_idx < v.size(); ++rec_idx)
    ans << print_rect(v[rec_idx]) << "; ";
  return ans.str();
}

} // end namespace vision_utils

#endif // PRINT_RECT_H
