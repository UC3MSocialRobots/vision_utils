#ifndef COLOR_UTILS_H
#define COLOR_UTILS_H

#include <stdio.h>
#include <stdlib.h>

// compressed_rounded_image_transport
#include <color/hue_utils.h>
// #include <src/cv_conversion_float_uchar.h>
// utils
#include <geom/foo_point.h>

namespace color_utils {
//typedef AD::Point3<double> Color;

static const int NB_PREDEFINED_COLORS = 22;

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline void _indexed_color_templated
(_T & r, _T & g, _T & b,
 const int & index,
 const _T & MAX, const _T & HIGH, const _T & MED, const _T & LOW, const _T & ZERO) {

  switch (index % NB_PREDEFINED_COLORS) {
  case 0: r = MAX; g = ZERO; b = ZERO; break;
  case 1: r = ZERO; g = ZERO; b = MAX; break;
  case 2: r = MAX; g = MAX; b = ZERO; break;
  case 3: r = MAX; g = ZERO; b = MAX; break;

  case 4: r = MAX; g = MED; b = ZERO; break;
  case 5: r = MED; g = MAX; b = ZERO; break;
  case 6: r = MED; g = ZERO; b = MAX; break;
  case 7: r = MAX; g = ZERO; b = MED; break;
  case 8: r = ZERO; g = MED; b = MAX; break;
  case 9: r = ZERO; g = MAX; b = MED; break;

  case 10: r = HIGH; g = ZERO; b = ZERO; break;
  case 11: r = ZERO; g = HIGH; b = ZERO; break;
  case 12: r = ZERO; g = ZERO; b = HIGH; break;

  case 13: r = HIGH; g = HIGH; b = ZERO; break;
  case 14: r = HIGH; g = ZERO; b = HIGH; break;
  case 15: r = ZERO; g = HIGH; b = HIGH; break;

  case 16: r = HIGH; g = LOW; b = ZERO; break;
  case 17: r = LOW; g = HIGH; b = ZERO; break;
  case 18: r = LOW; g = ZERO; b = HIGH; break;
  case 19: r = HIGH; g = ZERO; b = LOW; break;
  case 20: r = ZERO; g = LOW; b = HIGH; break;
  default:
  case 21: r = ZERO; g = HIGH; b = LOW; break;
  } // end switch
} // end _indexed_color_templated

////////////////////////////////////////////////////////////////////////////////

template<class Uchar>
inline void indexed_color255(Uchar & r, Uchar & g, Uchar & b,
                             const int index = rand()) {
  _indexed_color_templated<Uchar>(r, g,  b, index,
                                  (Uchar) 255, (Uchar) 153, (Uchar) 128, (Uchar) 77, (Uchar) 0);
}

////////////////////////////////////////////////////////////////////////////////

inline void indexed_color_norm(float & r, float & g, float & b,
                               const int index = rand()) {
  _indexed_color_templated<float>(r, g, b, index, 1, 0.7, 0.5, 0.33, 0);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   return the predefined color number i
 */
template<class Color3_255>
inline Color3_255 color(int idx = -1, int max = -1) {
  if (idx == -1)
    return color_utils::hue2rgb<Color3_255>(rand() % 128);
  if (max == -1) {
    unsigned char r, g, b;
    color_utils::indexed_color255(r, g, b, idx);
    return Color3_255(b, g, r);
  }
  return color_utils::hue2rgb<Color3_255>(128.f * idx / max);
} //

////////////////////////////////////////////////////////////////////////////

/*!
     \brief return the predefined color number i as a scalar
    */
template<class Color4_255>
inline Color4_255 color_scalar(int i = -1, int max = -1) {
  geometry_utils::FooPoint3<unsigned char> color3 =
      color_utils::color<geometry_utils::FooPoint3<unsigned char> >(i, max);
  return Color4_255(color3.x, color3.y, color3.z, 0);
} // end color_scalar()

} // end namespace color_utils

#endif // COLOR_UTILS_H
