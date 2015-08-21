/*!
  \file        clamp.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/10/15

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

clamp(), a function to constrain a value in a given span

 */

#ifndef CLAMP_H
#define CLAMP_H


/*!
 \param Value
    the value to be clamped
 \param Min
 \param Max
 \return T
    if Value < Min, T = Min.
    if Value > Max, T = Max.
    otherwise, T = Value
*/
template<typename T>
inline T clamp(T Value, const T Min, const T Max) {
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

template<typename T>
inline T modulo_real(T Value, const T Min, const T Max) {
  T ans = Value;
  while (ans >= Max)
    ans = Min + (ans - Max);
  while (ans < Min)
    ans = Max + (ans - Min);
  return ans;
}

#endif // CLAMP_H
