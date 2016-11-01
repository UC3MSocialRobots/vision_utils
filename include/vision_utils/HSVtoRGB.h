/*!
  \file        HSVtoRGB.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/10/29
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

\todo Description of the file
 */

#ifndef HSVTORGB_H
#define HSVTORGB_H

namespace vision_utils {

/*! from http://www.cs.rit.edu/~ncs/color/t_convert.html#RGB%20to%20HSV%20&%20HSV%20to%20RGB
  \param The hue value H runs from 0 to 360º.
  \param The saturation S is the degree of strength or purity and is from 0 to 1.
  Purity is how much white is added to the color, so S=1 makes the purest color (no white).
  \param Brightness V also ranges from 0 to 1, where 0 is the black.
*/
template<class Float>
inline void HSVtoRGB( const Float h, const Float s, const Float v,
                      Float & r, Float & g, Float & b) {
  if( s == 0 ) {
    // achromatic (grey)
    r = g = b = v;
    return;
  }
  float h2 = h / 60;   // sector 0 to 5
  int i = floor( h2 );
  float f = h2 - i;   // factorial part of h
  float p = v * ( 1 - s );
  float q = v * ( 1 - s * f );
  float t = v * ( 1 - s * ( 1 - f ) );
  switch( i ) {
    case 0:
      r = v; g = t; b = p;
      break;
    case 1:
      r = q; g = v; b = p;
      break;
    case 2:
      r = p; g = v; b = t;
      break;
    case 3:
      r = p; g = q; b = v;
      break;
    case 4:
      r = t; g = p; b = v;
      break;
    default:  // case 5:
      r = v; g = p; b = q;
      break;
  } // end switch i
} // end HSVtoRGB

} // end namespace vision_utils

#endif // HSVTORGB_H
