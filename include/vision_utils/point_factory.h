/*!
  \file        point_factory.h
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

#ifndef POINT_FACTORY_H
#define POINT_FACTORY_H

namespace vision_utils {

//! a simple factory to make a geometry_msgs::Point with one function
inline geometry_msgs::Point point_factory(const double & x = 0,
                                          const double & y = 0,
                                          const double & z = 0) {
  geometry_msgs::Point ans;
  ans.x = x;
  ans.y = y;
  ans.z = z;
  return ans;
} // end point_factory()

} // end namespace vision_utils

#endif // POINT_FACTORY_H
