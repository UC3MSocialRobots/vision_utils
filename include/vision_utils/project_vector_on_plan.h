/*!
  \file        project_vector_on_plan.h
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

#ifndef PROJECT_VECTOR_ON_PLAN_H
#define PROJECT_VECTOR_ON_PLAN_H
// std includes
#include <vector>

namespace vision_utils {

/*!
     *
 * @param std::vector
 * @param normal_to_plan
 * @return the projection of std::vector on the plan
 */
template<class Vector3>
static inline Vector3 project_vector_on_plan(const Vector3 & vector,
                                             const Vector3 & normal_to_plan) {
  Vector3 vector_normal_comp = (vector.dot(normal_to_plan)) * normal_to_plan;
  //printf("vector:%s", printP(vector).c_str());
  //    //printf("normal_to_plan:%s", printP(normal_to_plan).c_str());
  //    //printf("module:%f", vector.dot(normal_to_plan));
  //    //printf("vector_normal_comp:%s", printP(vector_normal_comp).c_str());
  return vector - vector_normal_comp;
}

} // end namespace vision_utils

#endif // PROJECT_VECTOR_ON_PLAN_H
