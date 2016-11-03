/*!
  \file        angle_between_vector_and_plan.h
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

#ifndef ANGLE_BETWEEN_VECTOR_AND_PLAN_H
#define ANGLE_BETWEEN_VECTOR_AND_PLAN_H
// std includes
#include <vector>

namespace vision_utils {

/*!
     *
 * @param std::vector
 * @param normal_to_plan
 * @return something between -180 and 180
 */
template<class Vector3>
static inline double angle_between_vector_and_plan(const Vector3 & vector,
                                                   const Vector3 & normal_to_plan) {
  Vector3 vector_proj = project_vector_on_plan(vector, normal_to_plan);
  // we use v-> . p-> = |v| * |p| * cos(v->, p->)
  // to find the cos
  double cos_angle = vector.dot(vector_proj) / (norm(vector) * norm(
                                                  vector_proj));
  //    //printf("vector_proj:%s", printP(vector_proj).c_str());
  //    //printf("cos_angle:%f", cos_angle);
  double angle = acos(cos_angle) * 180.f / M_PI;
  if (vector.dot(normal_to_plan) < 0)
    return -angle;
  //return 2 * PI - angle;
  return angle;
}

} // end namespace vision_utils

#endif // ANGLE_BETWEEN_VECTOR_AND_PLAN_H
