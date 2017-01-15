/*!
  \file        convert_sensor_data_to_xy.h
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

#ifndef CONVERT_SENSOR_DATA_TO_XY_H
#define CONVERT_SENSOR_DATA_TO_XY_H
// std includes
#include <sensor_msgs/LaserScan.h>
#include <vector>

namespace vision_utils {

//! convert from polar to xy coordinates for a laser data
template<class Pt2>
static inline void convert_sensor_data_to_xy(const sensor_msgs::LaserScan & laser_msg,
                                             std::vector<Pt2> & out_vector) {
  out_vector.clear();
  out_vector.reserve(laser_msg.ranges.size());
  const float* curr_range = &(laser_msg.ranges[0]);
  float curr_angle = laser_msg.angle_min;
  for (unsigned int idx = 0; idx < laser_msg.ranges.size(); ++idx) {
    //printf("idx:%i, curr_range:%g", idx, *curr_range);
    Pt2 newpt;
    newpt.x = *curr_range * cos(curr_angle);
    newpt.y = *curr_range * sin(curr_angle);
    out_vector.push_back(newpt);
    ++curr_range;
    curr_angle += laser_msg.angle_increment;
  } // end loop idx
} // end convert_sensor_data_to_xy()

} // end namespace vision_utils

#endif // CONVERT_SENSOR_DATA_TO_XY_H
