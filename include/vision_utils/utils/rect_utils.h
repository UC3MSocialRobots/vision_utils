#ifndef RECT_UTILS_H
#define RECT_UTILS_H

#include <vector>
#include <sstream>
#include "vision_utils/utils/error.h"

namespace geometry_utils {

/*! shrink a rectangle by a given ratio, keeping the same center
  * \arg ratio
      for instance, 0.5 will divide the sides of the rect by 2.
      0.9 will shrink just a bit the size of the rectangle.
  */
template<class Rect>
inline Rect shrink_rec(const Rect & in,
                       const double & ratio_width,
                       const double & ratio_height) {
  Rect ans;
  ans.x = in.x + in.width * (1. - ratio_width) / 2;
  ans.y = in.y + in.height * (1. - ratio_height) / 2;
  ans.width = in.width * ratio_width;
  ans.height = in.height * ratio_height;
  return ans;
}

template<class Rect>
inline Rect shrink_rec(const Rect & in, const double & ratio) {
  return shrink_rec<Rect>(in, ratio, ratio);
}

////////////////////////////////////////////////////////////////////////////////

template<class Img, class Rect>
inline Rect img_bbox(const Img & i) {
  return Rect(0, 0, i.cols, i.rows);
} // end biggest_rect()

////////////////////////////////////////////////////////////////////////////////

//! \retun true if \a small is included into \a big
template<class Rect>
inline bool bboxes_included(const Rect & big, const Rect & small) {
  if (big.x > small.x)
    return 0;
  if (big.y > small.y)
    return 0;
  if (big.x + big.width < small.x + small.width)
    return 0;
  if (big.y + big.height < small.y + small.height)
    return 0;
  return 1;
}

////////////////////////////////////////////////////////////////////////////////

template<class Rect, class Img>
inline bool bbox_included_image(const Rect & r, const Img & img) {
  return bboxes_included(img_bbox<Img, Rect>(img), r);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * print a rectangle in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Rect>
inline std::string print_rect(const Rect & p) {
  std::ostringstream ans;
  ans << "(" << p.x << ", " << p.y << ")+("
      << p.width << ", " << p.height << ")";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * print a vector of rectangles in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Rect>
inline std::string print_rects(const std::vector<Rect> & v) {
  std::ostringstream ans;
  for (unsigned int rec_idx = 0; rec_idx < v.size(); ++rec_idx)
    ans << print_rect(v[rec_idx]) << "; ";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

//! copy a Rectangle like class into another
template<class RectA, class RectB>
inline void copy_rectangles(const RectA & A, RectB & B) {
  B.x = A.x;
  B.y = A.y;
  B.width = A.width;
  B.height = A.height;
} // end biggest_rect()

////////////////////////////////////////////////////////////////////////////////

//! \return the index of the biggest rectangle in a set
template<class Rect>
inline int biggest_rect_index(const std::vector<Rect> & rects) {
  if (rects.size() < 1) {
    maggiePrint("Cannot call biggest_rect() on an empty vector!");
    return -1;
  }
  int best_rec_idx = 0;
  double best_rec_area = rects[0].width * rects[0].height;
  for (unsigned int curr_rec_idx = 1; curr_rec_idx < rects.size(); ++curr_rec_idx) {
    double curr_rec_area = rects[curr_rec_idx].width * rects[curr_rec_idx].width;
    if (curr_rec_area > best_rec_area) {
      best_rec_area = curr_rec_area;
      best_rec_idx = curr_rec_idx;
    }
  } // end loop rec_idx
  return best_rec_idx;
} // end biggest_rect_index()

////////////////////////////////////////////////////////////////////////////////

//! \return the biggest rectangle of a set
template<class Rect>
inline Rect biggest_rect(const std::vector<Rect> & rects) {
  if (rects.size() < 1) {
    maggiePrint("Cannot call biggest_rect() on an empty vector!");
    return Rect();
  }
  return rects[biggest_rect_index(rects)];
} // end biggest_rect()

////////////////////////////////////////////////////////////////////////////////

/*! \return the rectangle intersection between r1 and r2
 *  \see http://docs.opencv.org/modules/core/doc/basic_structures.html#Rect_
 */
template<class Rect>
inline Rect rectangle_intersection(const Rect & r1,
                                   const Rect & r2) {
  return (r1 & r2);
} // end biggest_rect()

////////////////////////////////////////////////////////////////////////////////

template<class Img, class Rect>
inline Rect rectangle_intersection_img(const Img & i,
                                       const Rect & r) {
  return rectangle_intersection(img_bbox<Img, Rect>(i), r);
} // end biggest_rect()

////////////////////////////////////////////////////////////////////////////////

/*!
 * \param in
 *   a rectangle
 * \return
 *    the center of the rectangle
 */
template<class Rect, class Point>
inline Point rect_center(const Rect & in) {
  return Point(in.x + in.width / 2, in.y + in.height / 2);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Remove the rectangles that include other rectangles in their area,
 * or the contrary.
 * \param recs_in
 *  The rectangles to be filtered.
 * \param recs_out
 *  [OUT] the filtered rectangles. None of them contains another one.
 * \param remove_including
 *  If true, remove the rectangles that include other rectangles in their area
 *            (keep only the smallest ones).
 *  If false, remove the rectangles that are included into another
 *            (keep only the biggest ones).
 */
template<class Rect>
inline void remove_including_rectangles(const std::vector<Rect> & recs_in,
                                        std::vector<Rect> & recs_out,
                                        bool remove_including = true) {
  // printf("remove_including_rectangles(%i recs)\n", recs_in.size());
  recs_out.clear();
  recs_out.reserve(recs_in.size());
  const Rect* r1 = &(recs_in[0]);
  for( unsigned int r1_idx = 0; r1_idx < recs_in.size(); r1_idx++ ) {
    bool r1_ok = true;
    const Rect* r2 = &(recs_in[0]);
    for( unsigned int r2_idx = 0; r2_idx < recs_in.size(); r2_idx++ ) {
      if (r1_idx == r2_idx) { // jump to next rectangle
        ++r2;
        continue;
      }
      //printf("r1_idx:%i, r2_idx:%i, r1:%s, r2:%s, inter:%s, equal:%i\n",
      //       r1_idx, r2_idx, print_rect(*r1).c_str(), print_rect(*r2).c_str(),
      //       print_rect(rectangle_intersection(*r1, *r2)).c_str(),
      //       rectangle_intersection(*r1, *r2) == (remove_including ? *r1 : *r2));

      // rectangles equal -> keep the one with the lowest index
      if(*r1 == *r2) {
        if (r1_idx > r2_idx) {
          // printf("Case 1\n");
          r1_ok = false;
          break;
        }
        else { // jump to next rectangle
          ++r2;
          continue;
        }
      } // end if(*r1 == *r2)

      // if r2 included in r1, break
      if(rectangle_intersection(*r1, *r2) == (remove_including ? *r1 : *r2)) {
        // printf("Case 2\n");
        r1_ok = false;
        break;
      }
      ++r2; // increment pointer
    } // end for r2_idx
    if( r1_ok ) // no rectangle is included in r1
      recs_out.push_back(*r1);
    ++r1; // increment pointer
  } // end for r1_idx
} // end remove_including_rectangles()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Remove the rectangles included in other rectangles in their area.
 * or the contrary.
 * \param recs_in
 *  The rectangles to be filtered.
 * \param recs_out
 *  [OUT] the filtered rectangles. None of them contains another one.
 */
template<class Rect>
inline void remove_included_rectangles(const std::vector<Rect> & recs_in,
                                       std::vector<Rect> & recs_out) {
  remove_including_rectangles(recs_in, recs_out, false);
}

} // end namespace geometry_utils

#endif // RECT_UTILS_H
