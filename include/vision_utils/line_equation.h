/*!
  \file        line_equation.h
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

#ifndef LINE_EQUATION_H
#define LINE_EQUATION_H

namespace vision_utils {

/**
 * @brief  return the line equation between 2 points
     *
 * @param A
 * @param B
 * @return Point3
 */
template<class Point2, class Point3>
static inline Point3 line_equation(Point2 A, Point2 B) {
  //    double a = (B.y - A.y);
  //    double b = (A.x - B.x);
  //    double c = (B.x * A.y) - (B.y * A.x);
  //    if (b != 0) {/* normaliszation */
  //        a = a / b;
  //        c = c / b;
  //        b = 1;
  //    }
  //    //cout << a << " x + " << b << " y + " << c << " = 0" << endl;
  //    return Point3(a, b, c);

  double b = (A.x - B.x);
  if (b != 0) /* normaliszation */
    return Point3((B.y - A.y) / b,
                  1,
                  (B.x * A.y - B.y * A.x) / b);
  else {
    double a = (B.y - A.y);
    if (a != 0) // b = 0
      return Point3(1,
                    0,
                    (B.x * A.y - B.y * A.x) / a);
    else // a = 0, b = 0
      return Point3(0, 0, 0);
  }
}

} // end namespace vision_utils

#endif // LINE_EQUATION_H
