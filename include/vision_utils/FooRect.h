/*!
  \file        FooRect.h
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

#ifndef FOORECT_H
#define FOORECT_H

namespace vision_utils {

//! a foo implementation for a rectangle
template<class _Type>
class FooRect_ {
public:
  //! ctor
  FooRect_(_Type _x, _Type _y, _Type _w, _Type _h) :
    x(_x), y(_y), width(_w), height(_h) {}

  _Type x;
  _Type y;
  _Type width;
  _Type height;
};

typedef FooRect_<int> FooRect;

} // end namespace vision_utils

#endif // FOORECT_H
