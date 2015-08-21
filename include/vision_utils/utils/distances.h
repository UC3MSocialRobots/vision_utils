#ifndef DISTANCES_H
#define DISTANCES_H

#include <vector>
#include <limits>
#include <vision_utils/utils/error.h>

namespace geometry_utils {

////////////////////////////////////////////////////////////////////////////////

/*!
  \return the distance AB
  */
template<class Point2_A, class Point2_B>
static inline double
distance_points(const Point2_A & A, const Point2_B & B) {
  return hypot(A.x - B.x, A.y - B.y);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param A
 \param B
 \return the quantity AB * AB. It is faster to compute than their actual distance
 and enough for, for instance, comparing two distances
*/
template<class Point2_A, class Point2_B>
static inline double
distance_points_squared(const Point2_A & A, const Point2_B & B) {
  return (A.x - B.x) * (A.x - B.x) +  (A.y - B.y) * (A.y - B.y);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param A  a 3D pt (x, y, z)
 \param B  a 3D pt (x, y, z) - it can be of another type than A
 \return the quantity AB * AB. It is faster to compute than their actual distance
 and enough for, for instance, comparing two distances
*/
template<class Point3_A, class Point3_B>
static inline double
distance_points3_squared(const Point3_A & A, const Point3_B & B) {
  return (A.x - B.x) * (A.x - B.x)
      +  (A.y - B.y) * (A.y - B.y)
      +  (A.z - B.z) * (A.z - B.z);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param A  a 3D pt (x, y, z)
 \param B  a 3D pt (x, y, z) - it can be of another type than A
 \return the quantity AB * AB. It is faster to compute than their actual distance
 and enough for, for instance, comparing two distances
*/
template<class Point3_A, class Point3_B>
static inline double
distance_points3(const Point3_A & A, const Point3_B & B) {
  return sqrt((A.x - B.x) * (A.x - B.x)
              +  (A.y - B.y) * (A.y - B.y)
              +  (A.z - B.z) * (A.z - B.z));
}

////////////////////////////////////////////////////////////////////////////////

/*!
  \return the projection of P on the line ax + by + c = 0
  */
template<class Point2>
static inline Point2
proj_point_on_line(const Point2 & P,
                   const double & a, const double & b, const double & c) {
  return Point2(
        // x:
        (-a * c  - a * b *  P.y + b * b * P.x)
        / (a * a + b * b),
        // y:
        (-b * c  - a * b *  P.x + a * a * P.y)
        / (a * a + b * b)
        );
}

////////////////////////////////////////////////////////////////////////////////

/*!
  \return the projection of P on the line ax + by + c = 0
  */
template<class Point2>
static inline double
dist_point_line(const Point2 & P,
                const double & a, const double & b, const double & c) {
  return distance_points(P, proj_point_on_line(P, a, b, c));
}

////////////////////////////////////////////////////////////////////////////////

/*!
 return the distance to a segment
 \param pt
 \param seg1
 \param seg2
 \return double
*/
template<class Point2>
static inline double distance_point_segment(const Point2 & pt,
                                            const Point2 & seg1,
                                            const Point2 & seg2) {
  // compute the projection of P3 on [P1, P2]
  Point2 diff (seg2.x - seg1.x, seg2.y - seg1.y);
  double u = 1.f * ((pt.x - seg1.x) * diff.x + (pt.y - seg1.y) * diff.y)
             / (diff.x * diff.x + diff.y * diff.y);
  maggieDebug3("p_curr:%s, p_next:%s, u:%g",
               printP2(seg1).c_str(), printP2(seg2).c_str(), u);

  if (u < 0) // the projection is before seg1
    return distance_points(pt, seg1);
  else if (u > 1) // the projection is after seg2
    return distance_points(pt, seg2);
  else { // proj on the segment
    return distance_points(pt, Point2
                           ( // the proj
                             u * seg2.x + (1 - u) * seg1.x,
                             u * seg2.y + (1 - u) * seg1.y
                             ));
    //cout << " proj:" << proj.x << "," << proj.y << endl;
  }
}

////////////////////////////////////////////////////////////////////////////////

/*!
 return the squared euclididan distance to a segment
 \param pt
    the point whose distance to the segment we want to compute
 \param seg1
   the first end of the segment
 \param seg2
   the second end of the segment
 \param proj
  the corresponding projection on the segment
 \return double
    ths squared euclidean distance between pt and [seg1, seg2]
*/
template<class Point2>
static inline double distance_point_segment_sq(const Point2 & pt,
                                               const Point2 & seg1,
                                               const Point2 & seg2,
                                               Point2 & proj) {
  // compute the projection of P3 on [P1, P2]
  Point2 diff (seg2.x - seg1.x, seg2.y - seg1.y);
  double u = 1.f * ((pt.x - seg1.x) * diff.x + (pt.y - seg1.y) * diff.y)
             / (diff.x * diff.x + diff.y * diff.y);

  if (u < 0) { // the projection is before seg1
    proj.x = seg1.x;
    proj.y = seg1.y;
  }
  else if (u > 1) { // the projection is after seg2
    proj.x = seg2.x;
    proj.y = seg2.y;
  }
  else { // proj on the segment
    proj.x = u * seg2.x + (1 - u) * seg1.x;
    proj.y = u * seg2.y + (1 - u) * seg1.y;
  }
  return geometry_utils::distance_points_squared(pt, proj);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   returns the min distance to the sides of the polygon
 * (euclidean projection on the sides).
 * Cf http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/
     *
 * \param   pt the test point
 * \param   poly the list of points in the polygon
 */
template<class Point2>
static inline double distance_point_polygon(const Point2 & pt,
                                            const std::vector<Point2> & poly,
                                            bool is_poly_closed = true) {
  double minDist = std::numeric_limits<double>::max();//, currDist, u;

  typename std::vector<Point2>::const_iterator p_curr = poly.begin(),
      p_next = poly.begin() + 1;

  for (unsigned int i = 1;
       i <= (is_poly_closed? poly.size() : poly.size() - 1);
       ++i) {

    minDist = fmin(minDist, distance_point_segment(pt, *p_curr, *p_next));

    ++p_curr;
    ++p_next;
    if (is_poly_closed && i == poly.size() - 1)
      p_next = poly.begin();
  } // end loop i

  return minDist;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \brief   returns the squared min distance to the sides of the polygon
 (euclidean projection on the sides).
 \param pt
    the test point
 \param poly
    the list of points in the polygon
 \param closest pt
    the closest point in the polygon to pt
 \param closest_dist_sq
    the squared euclidean distance between pt and poly
 \param is_closed
    true if poly is closed
 */
template<class Point2>
static inline bool distance_point_polygon_squared
(const Point2 & pt,
 const std::vector<Point2> & poly,
 Point2 & closest_pt,
 double & closest_dist_sq,
 bool is_poly_closed = true) {
  // do nothing if polygon empty
  if (poly.empty()) {
    maggieWeakError("distance_point_polygon_squared() should not be called with an empty polygon");
    closest_dist_sq = 0;
    closest_pt = pt;
    return false;
  }

  closest_dist_sq = std::numeric_limits<double>::max();
  closest_pt = poly.front();
  Point2 curr_closest_pt;
  unsigned int poly_size = poly.size(), max_pt_idx = poly_size - 1;
  for (unsigned int pt_idx = 0; pt_idx < max_pt_idx; ++pt_idx) {
    double curr_dist_sq = distance_point_segment_sq
                          (pt, poly[pt_idx], poly[pt_idx + 1], curr_closest_pt);
    if (closest_dist_sq > curr_dist_sq) {
      closest_dist_sq = curr_dist_sq;
      closest_pt = curr_closest_pt;
    }
  } // end loop pt_idx

  // check the last segment if the polygon is closed
  if (!is_poly_closed)
    return true;
  double curr_dist_sq = distance_point_segment_sq
                        (pt, poly.back(), poly.front(), curr_closest_pt);
  if (closest_dist_sq > curr_dist_sq) {
    closest_dist_sq = curr_dist_sq;
    closest_pt = curr_closest_pt;
  }
  return true;
} // end distance_point_polygon

////////////////////////////////////////////////////////////////////////////////

/*!
 \param A
    a vector of 2D points
 \param B
    a vector of 2D points
 \param min_dist
    a threshold distance
 \return bool
    true if there is a a pair of points closer than min_dist
*/
template<class _Pt2>
inline bool two_vectors_closer_than_threshold(const std::vector<_Pt2> & A,
                                              const std::vector<_Pt2> & B,
                                              const float min_dist) {
  float min_dist_sq = min_dist * min_dist;
  for (unsigned int A_idx = 0; A_idx < A.size(); ++A_idx) {
    for (unsigned int B_idx = 0; B_idx < B.size(); ++B_idx) {
      if (geometry_utils::distance_points_squared(A[A_idx], B[B_idx])
          < min_dist_sq)
        return true;
    } // end loop B_idx
  } // end loop A_idx
  return false;
} // end two_vectors_closer_than_threshold()

////////////////////////////////////////////////////////////////////////////////


} // end namespace geometry_utils

#endif // DISTANCES_H
