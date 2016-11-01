/*!
  \file        colormaps.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/13

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

#ifndef COLORMAPS_H
#define COLORMAPS_H

#include "vision_utils/color_utils.h"
#include "vision_utils/clamp.h"

namespace vision_utils {

//template<class Color4_255>
//typedef Color4_255 (*RatioColormap_)(const double &);

typedef cv::Scalar (*RatioColormap)(const double &);

//! \return always black
template<class Color4_255>
Color4_255 ratio2black_(const double & /*ratio*/) {
  return Color4_255(0, 0, 0, 0);
}
//! template specialization for cv::Scalar
RatioColormap ratio2black = &ratio2black_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

//! \return 0:black -> 1:white
template<class Color4_255>
Color4_255 ratio2grey_(const double & ratio) {
  uchar v = ratio * 255;
  return Color4_255(v, v, v, v);
}
//! template specialization for cv::Scalar
RatioColormap ratio2grey = &ratio2grey_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

//! \return 0:white -> 1:black
template<class Color4_255>
Color4_255 ratio2grey_inv_(const double & ratio) {
  uchar v = 255 * (1. - ratio);
  return Color4_255(v, v, v, v);
}

//! template specialization for cv::Scalar
RatioColormap ratio2grey_inv = &ratio2grey_inv_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

//! \return 0:red -> 0.5:blue -> 1:magenta
template<class Color4_255>
Color4_255 ratio2hue_(const double & ratio) {
  return hue2rgb<Color4_255>(ratio * 180);
}

//! template specialization for cv::Scalar
RatioColormap ratio2hue = &ratio2hue_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

//! \return 1:red -> 0:green
template<class Color4_255>
Color4_255 ratio2red_green_(const double & ratio) {
  // ratio=1-> red=0 degrees , ratio=0 -> green=120 degrees=60 for hue2rgb()
  int ratio_inv = 60 * (1. - ratio);
  ratio_inv = (ratio_inv < 0 ? 0 : (ratio_inv > 60 ? 60 : ratio_inv));
  return hue2rgb<Color4_255>(ratio_inv);
}

//! template specialization for cv::Scalar
RatioColormap ratio2red_green = &ratio2red_green_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

//! \return [1 0.5]:red -> 0:green
template<class Color4_255>
Color4_255 ratio2red_green_half_(const double & ratio) {
  return ratio2red_green_<Color4_255>( 2. * ratio );
}

//! template specialization for cv::Scalar
RatioColormap ratio2red_green_half = &ratio2red_green_half_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////// index colormaps
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//template<class Color4_255>
//typedef Color4_255 (*IndexColormap_)(const int &);

typedef cv::Scalar (*IndexColormap)(const int &);

//! \return one of the predefined colors
template<class Color4_255>
Color4_255 index2predefined_color_(const int & index) {
  Color4_255 ans = color<Color4_255>(index, -1);
  // set fourth value to 0
  ans[3] = 0;
  return ans;
}

//! template specialization for cv::Scalar
IndexColormap index2predefined_color = &index2predefined_color_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

//! \return one of the predefined colors, but much brighter (ok for black texts on top)
template<class Color4_255>
Color4_255 index2predefined_color_washed_(const int & index) {
  Color4_255 ans = color<Color4_255>(index, -1);
  ans[0] = (ans[0] + 255) / 2;
  ans[1] = (ans[1] + 255) / 2;
  ans[2] = (ans[2] + 255) / 2;
  return ans;
}

//! template specialization for cv::Scalar
IndexColormap index2predefined_color_washed = &index2predefined_color_washed_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

//! \return a greyscale color;
template<class Color4_255>
Color4_255 index2grey_(const int & index) {
  // 157 is prime and its multiples are nicely distributed in [0..255]
  uchar v = (index * 157) % 255;
  return Color4_255(v, v, v, v);
}

//! template specialization for cv::Scalar
IndexColormap index2grey = &index2grey_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

//! \return a grey_washedscale color;
template<class Color4_255>
Color4_255 index2grey_washed_(const int & index) {
  Color4_255 ans = index2grey_<Color4_255>(index);
  ans[0] = (ans[0] + 255) / 2;
  ans[1] = (ans[1] + 255) / 2;
  ans[2] = (ans[2] + 255) / 2;
  return ans;
}

//! template specialization for cv::Scalar
IndexColormap index2grey_washed = &index2grey_washed_<cv::Scalar>;

////////////////////////////////////////////////////////////////////////////////

} // end namespace vision_utils

#endif // COLORMAPS_H
