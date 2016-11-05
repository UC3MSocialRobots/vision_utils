/*!
 * \file geometry_utils.h
 *
 * A package for useful functions in geometry
 *
 * \date Dec 18, 2010
 * \author Arnaud Ramey
 */

#ifndef GEOMETRYUTILS_H_
#define GEOMETRYUTILS_H_

// maggie


#include "vision_utils/foo_point.h"
#include "vision_utils/hausdorff_distances.h"
// std
#include <math.h> // for acos()
#include <assert.h>
#include <algorithm>
#include <sstream>
#include <iostream>

namespace vision_utils {

//cut:FooRect
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

////////////////////////////////////////////////////////////////////////////////
//cut:printP2

/*!
 * print a point in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Point2>
static inline std::string printP2(const Point2 & p) {
  std::ostringstream ans;
  ans << "(" << p.x << ", " << p.y << ")";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////
//cut:printP

/*!
 * print a point in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Point3>
static inline std::string printP(const Point3 & p) {
  std::ostringstream ans;
  ans << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////
//cut:printP4

/*!
 * print a 4D point in a fancy way
 * @param p
 * @return the fancy std::string
 */
template<class Point3>
static inline std::string printP4(const Point3 & p) {
  std::ostringstream ans;
  ans << "(" << p.x << ", " << p.y << ", " << p.z << ", " << p.w << ")";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////
//cut:pose_to_string

template<class _Pose3>
inline std::string pose_to_string(const _Pose3 & pose) {
  std::ostringstream ans;
  ans << "pos:" << printP(pose.position)
      << ", orien:" << printP4(pose.orientation);
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////
//cut:pose_stamped_to_string

template<class _Pose3Stamped>
inline std::string pose_stamped_to_string(const _Pose3Stamped & pose) {
  std::ostringstream ans;
  ans << "pose: {"
      << "t:" << std::fixed << pose.header.stamp.toSec() << "s, "
      << "frame: '" << pose.header.frame_id.c_str() << "' : "
      << "pos:" << printP(pose.pose.position)
      << ", orien:" << printP4(pose.pose.orientation)
      << " }";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////
//cut:norm2

/*!
 * @param p
 * @return the norm of p
 */
template<class Vector2>
static inline float norm2(const Vector2 & p) {
  return hypot(p.x, p.y);
}


////////////////////////////////////////////////////////////////////////////////
//cut:norm

/*!
 * @param p
 * @return the norm of p
 */
template<class Vector3>
static inline float norm(const Vector3 & p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

////////////////////////////////////////////////////////////////////////////////
//cut:normalize

/*!
 * normalize a std::vector
 * @param p the std::vector to normalize
 */
template<class Vector3>
static inline void normalize(Vector3 & p) {
  float normP = norm(p);
  p.x /= normP;
  p.y /= normP;
  p.z /= normP;
}

////////////////////////////////////////////////////////////////////////////////
//cut:project_vector_on_plan

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

////////////////////////////////////////////////////////////////////////////////
//cut:angle_between_vector_and_plan
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

////////////////////////////////////////////////////////////////////////////////
//cut:mult_matrix_vector

/*!
 * computes M times V with OpenCV matrix mult routines.
 * However, this routine only takes 2 matrices as arguments,
 * so here we do a nice cast.
 * @param M a 3X3 matrix
 * @param V a 1X3 std::vector
 * @return M * V a 1X3 std::vector
 */
template<class Vector3, class Mat1f>
static inline Vector3 mult_matrix_vector(const Mat1f & M, const Vector3 & V) {
  // convert it Vec -> Mat
  //const float V_array[3][1] = { V[0], V[1], V[2] };
  //Mat1f V_matrix(V);
  Mat1f V_matrix(3, 1);
  V_matrix(0, 0) = V.x;
  V_matrix(1, 0) = V.y;
  V_matrix(2, 0) = V.z;
  //printf("size:%i x %i", V_matrix.cols, V_matrix.rows);
  //V_matrix.t();
  // make the transfo
  Mat1f M_times_V_matrix = M * V_matrix;
  // convert the result Mat -> Vec
  //    for (int comp_idx = 0; comp_idx < 3; ++comp_idx)
  //        M_times_V[comp_idx] = M_times_V_matrix(comp_idx, 0);
  Vector3 M_times_V(
        M_times_V_matrix(0, 0),
        M_times_V_matrix(1, 0),
        M_times_V_matrix(2, 0)
        );
  return M_times_V;
}

////////////////////////////////////////////////////////////////////////////////
//cut:point_inside_polygon

/*!
 * \brief   detect if a point is inside a polygon - return true or false
 *  http://en.wikipedia.org/wiki/Point_in_polygon#Winding_number_algorithm
     *
 * \param   p the point
 * \param   poly the polygon
 * \return  true if the point is in the polygon
 */
template<class Point2>
static inline bool point_inside_polygon(const Point2 & p,
                                        const std::vector<Point2> & poly) {
#if 0 // old implementation
  double sum_angles = 0;
  double angle;
  const Point2 *p_curr = &poly.at(0], *p_next = &poly.at(0];
                                                p_next++;

                         for (unsigned int i = 1; i <= poly.size(); ++i) {
    Point2 v1 (p.x - p_curr->x, p.y - p_curr->y);
    Point2 v2 (p.x - p_next->x, p.y - p_next->y);
    if ((v1.x == 0 && v1.y == 0) || (v2.x == 0 && v2.y == 0))
      return true;
    angle = acos(
              // dot product
              1.f * v1.x * v2.x + 1.f * v1.y * v2.y
              /
              (hypot(v1.x, v1.y) * hypot(v2.x, v2.y)));
    if (1.f * v1.x * v2.y - 1.f * v1.y * v2.x < 0)
      angle = -angle; // use of the vectorial product
    sum_angles += angle;

    ++p_curr;
    ++p_next;
    if (i == poly.size() - 1)
      p_next = &poly.at(0];
  }

  //cout << " p:" << p.x << "," << p.y << " - sum:" << sum_angles << endl;
  if (sum_angles < PI && sum_angles > -PI)
    return 0;
  return 1;
#else
  /*
     * algo from http://www.visibone.com/inpoly/
     */
  Point2 p_old, p_new, p1, p2;
  bool inside = false;
  int npoints = poly.size();

  if (npoints < 3) {
    return(0);
  }
  p_old = poly[npoints-1];

  for (int i=0 ; i < npoints ; i++) {
    p_new = poly[i];
    if (p_new.x > p_old.x) {
      p1 = p_old;
      p2 = p_new;
    }
    else {
      p1 = p_new;
      p2 = p_old;
    }
    if ((p_new.x < p.x) == (p.x <= p_old.x)          /* edge "open" at one end */
        && 1.f * (p.y-p1.y) * (p2.x-p1.x) < 1.f * (p2.y-p1.y) * (p.x-p1.x)) {
      inside = !inside;
    }
    p_old.x = p_new.x;
    p_old.y = p_new.y;
  } // end loop i
  return(inside);
#endif
}

////////////////////////////////////////////////////////////////////////////////
//cut:translate_polygon

/*!
 * \brief   translates a polygon
 * \param   poly_in the list of points in the polygon
 * \param   translation the vector among which we translate
 */
template<class Point2>
static inline void translate_polygon(std::vector<Point2> & poly,
                                     const Point2 & translation) {
  for(typename std::vector<Point2>::iterator pt = poly.begin();
      pt != poly.end(); ++pt) {
    pt->x += translation.x;
    pt->y += translation.y;
  } // end loop pt
}

////////////////////////////////////////////////////////////////////////////////
//cut:rotate_translate_polygon

#define ROTATE_COSSIN_X(x, y, cos_angle, sin_angle) \
  (cos_angle * (x) - sin_angle * (y))
#define ROTATE_COSSIN_Y(x, y, cos_angle, sin_angle) \
  (sin_angle * (x) + cos_angle * (y))
#define ROTATE_ANGLE_X(x, y, angle) \
  (ROTATE_COSSIN_X(x, y, cos(angle), sin(angle)) )
#define ROTATE_ANGLE_Y(x, y, angle) \
  (ROTATE_COSSIN_Y(x, y, cos(angle), sin(angle)) )


////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   rotate a polygon from the an agle
 * and then apply a translation
 * \param   poly_in the list of points in the polygon
 * \param   poly_out must be different from poly_in
 * \param   rotation the angle of rotation in radians
 * \param   translation the given translation
 */
template<class Point2>
static inline void rotate_translate_polygon(const std::vector<Point2> & poly_in,
                                            std::vector<Point2> & poly_out,
                                            const double & rotation,
                                            const Point2 & translation) {
  // handle the empty vector case
  if (poly_in.size() == 0) {
    poly_out.clear();
    return;
  }

  // check that both vectors are not identical
  assert(poly_in != poly_out);

  int npts = poly_in.size();
  double cos_angle = cos(rotation), sin_angle = sin(rotation);
  poly_out.resize(npts);

  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    poly_out[pt_idx].x = ROTATE_COSSIN_X(poly_in[pt_idx].x, poly_in[pt_idx].y,
                                         cos_angle, sin_angle)
                         + translation.x;
    poly_out[pt_idx].y = ROTATE_COSSIN_Y(poly_in[pt_idx].x, poly_in[pt_idx].y,
                                         cos_angle, sin_angle)
                         + translation.y;
  } // end loop pt_idx
} // end rotate_translate_polygon()

////////////////////////////////////////////////////////////////////////////////
//cut:rotate_dilate_polygon

/*!
 * \brief   rotate a polygon from the angle theta around its center ,
 * with optionnaly a dilatation
     *
 * \param   theta the angle of rotation in radians
 * \param   dilat the factor of dilatation of the rectangle (1=no change)
 * \param   poly the list of points in the polygon
 */
template<class Point2>
static inline void rotate_dilate_polygon(std::vector<Point2> & poly,
                                         const double & theta,
                                         const double & dilat) {
  // determine center O of the polygon
  double ox = 0, oy = 0;
  for(typename std::vector<Point2>::const_iterator curr = poly.begin();
      curr != poly.end(); ++curr) {
    ox += curr->x;
    oy += curr->y;
  }
  ox = ox / poly.size();
  oy = oy / poly.size();
  //cout << "oX:" << ox << "\t oY:" << oy << endl;

  // apply the rotation and dilation
  for(typename std::vector<Point2>::iterator curr = poly.begin();
      curr != poly.end(); ++curr) {
    curr->x = dilat * ROTATE_ANGLE_X(curr->x - ox, curr->y - oy, theta) + ox;
    curr->y = dilat * ROTATE_ANGLE_Y(curr->x - ox, curr->y - oy, theta) + oy;
  }
}

////////////////////////////////////////////////////////////////////////////////
//cut:absolute_angle_between_two_vectors

/*!
 * \brief   returns the angle formed between two vectors
 * \param   v1 the first vector
 * \param   v2 the second vector
 * \return  the angle between 0 and PI
 */
template<class Vec2>
static inline double absolute_angle_between_two_vectors(const Vec2 & v1,
                                                        const Vec2 & v2) {
  //double cos_angle =
  //printf("cos_angle:%f", cos_angle);
  return acos( fmin(1, fmax(-1,
                            // cos angle is :
                            (v1.x * v2.x + v1.y * v2.y) // scalar product
                            /
                            (norm2(v1) * norm2(v2)) // product of norms
                            )));
}

////////////////////////////////////////////////////////////////////////////////
//cut:absolute_angle_between_three_points

/*!
 * \brief   returns the angle between three points A, B, C,
 * that is the angle between the vectors BA and BC - between 0 and PI
 */
template<class Point2>
static inline double absolute_angle_between_three_points(const Point2 & A,
                                                         const Point2 & B,
                                                         const Point2 & C) {
  return absolute_angle_between_two_vectors(
        Point2 (A.x - B.x, A.y - B.y),  // BA
        Point2 (C.x - B.x, C.y - B.y) //   BC
        );
}

////////////////////////////////////////////////////////////////////////////////
//cut:oriented_angle_of_a_vector

/*!
 * \brief   returns the angle formed between a vector and the horizontal line
 * \param   v the vector
 * \return  the principal angle of v, in the interval [-pi,+pi] radians.
 */
template<class Vec2>
static inline double oriented_angle_of_a_vector(const Vec2 & v) {
  return atan2(v.y, v.x);
}

////////////////////////////////////////////////////////////////////////////////
//cut:oriented_average_angle

/*!
 * \brief   returns the average angle of a bunch of points,
            taking care of the painful case around [+-pi]
 * \param   angles some angles in the interval [-pi,+pi] radians
 * \return  the average angle , in the interval [-pi,+pi] radians.
 */
static inline double oriented_average_angle(const std::vector<double> & angles) {
  double avg = 0, best_angle = 0, curr_angle, lowest_dist;

  // iterate on all the angles
  for(std::vector<double>::const_iterator angle_it = angles.begin();
      angle_it != angles.end() ; ++ angle_it) {
    lowest_dist = INFINITY;

    // examinate angle - 2 PI, angle, angle + 2 PI
    for (int mod = -1; mod <= 1; ++mod) {
      curr_angle = *angle_it + mod * 2 * M_PI;
      if (fabs(avg - curr_angle) < lowest_dist)
        best_angle = curr_angle;
    } // end loop mod

    // add the best
    avg += best_angle;
  } // end loop angle_it

  // average it
  avg = avg / angles.size();

  // get the avg back in [-PI, PI]
  if (avg > M_PI)
    return avg - 2 * M_PI;
  else if (avg < - M_PI)
    return avg + 2 * M_PI;
  else
    return avg;
}

////////////////////////////////////////////////////////////////////////////////
//cut:oriented_average_angle_in_vector

/*!
 Computes the average angle of a point with its neighbours
 \param pts
 \param pt_pos you should not ask for the left angle of the first pt or
        right angle of the last one
 \param to_left
 \param max_window_size the nb of neighbours to take into account.
        For the points at the very beginning or end of the vector,
        less points will be used.
 \return the angle between -PI and PI
*/
template<class Point2>
static inline double oriented_average_angle_in_vector(const std::vector<Point2> & pts,
                                                      const unsigned int pt_pos,
                                                      const unsigned int max_window_size,
                                                      bool to_left = true) {
  // take care of the extreme points
  if ((!to_left && pt_pos >= pts.size()-1) ||
      (to_left && pt_pos <= 0)) {
    //printf("size:%i, pt_pos:%i and going to %s: extreme point! Average angle with its neighbours impossible.", pts.size(), pt_pos, (to_left ? "left" : "right"));
    return 0;
  }

  // determine the size of the window (existing neighbours)
  int window_size = std::min(max_window_size,
                             (to_left ?
                                pt_pos :
                                (unsigned int) (pts.size() - 1 - pt_pos)));
  //printf("window_size:%i", window_size);

  // collect all the angles in the surroundings
  typename std::vector<Point2>::const_iterator
      curr_it = pts.begin() + pt_pos,
      neigh_it = curr_it + (to_left ? -1 : 1);
  std::vector<double> angles;
  for (int neigh_count = 0; neigh_count < window_size; ++neigh_count) {
    angles.push_back(oriented_angle_of_a_vector( *neigh_it - *curr_it ));
    // move iterator
    neigh_it += (to_left ? -1 : 1);
  } // end loop neigh_count

  // average them
  return oriented_average_angle(angles);
}

////////////////////////////////////////////////////////////////////////////////
//cut:align_polygons

/*!
 Aligns two polygons(rotation and translation, but no scaling).
 \param A
 \param B
 \param B_out
    will contain rotated and translated points of B,
    such as B_out[0] = A[0],
    and the orientation of B_out is the same as A
*/
template<class Point2>
static inline void align_polygons(const std::vector<Point2> & A,
                                  const std::vector<Point2> & B,
                                  std::vector<Point2> & B_out) {
  // find angles in A and B
  double angle_A = oriented_angle_of_a_vector
                   (A.back() - A.front());
  double angle_B = oriented_angle_of_a_vector
                   (B.back() - B.front());
  //printf("angle_A:%g, angle_B:%g\n", angle_A, angle_B);

  B_out.clear();
  B_out.reserve(B.size());
  double cos_angle = cos(angle_A - angle_B), sin_angle = sin(angle_A - angle_B);

  for (unsigned int pt_idx = 0; pt_idx < B.size(); ++pt_idx) {
    B_out.push_back
        (Point2(ROTATE_COSSIN_X(B[pt_idx].x - B.front().x, B[pt_idx].y - B.front().y,
                                cos_angle, sin_angle)
                + A.front().x,
                ROTATE_COSSIN_Y(B[pt_idx].x - B.front().x, B[pt_idx].y - B.front().y,
                                cos_angle, sin_angle)
                + A.front().y
                ));
  }
} // end align_polygons()

////////////////////////////////////////////////////////////////////////////////
//cut:distance_patterns

/*!
 Compare two patterns of points.
 Aligns them beforehand (rotation and translation, but no scaling).
 \param A
 \param B
 \param max_distance
 \return double
    +double::max() if the computed distance is > max_distance
*/
template<class _Pt2>
inline float distance_patterns(const std::vector<_Pt2> & A,
                               const std::vector<_Pt2> & B,
                               const float max_distance
                               = std::numeric_limits<float>::max()
                                 ) {
  std::vector<_Pt2> B_rot;
  align_polygons(A, B, B_rot);
  //printf("B_rot:%s\n", accessible_to_string(B_rot).c_str());

  // compare A and B_rot
  float sum_dist = D22_with_min<_Pt2, std::vector<_Pt2> >
      (A, B_rot, max_distance);
  // float sum_dist = 0;
  //for (unsigned int ptA_idx = 0; ptA_idx < A.size(); ++ptA_idx)
  //sum_dist += distance_point_polygon(A[ptA_idx], B_rot, false);
  return sum_dist;
} // end distance_patterns()

////////////////////////////////////////////////////////////////////////////////
//cut:print_line_equation

/**
 * @brief  return a string representing a line equation,
 * \example 2 x + 3 y + 5 = 0
 */
template<class Point3>
static inline std::string print_line_equation(Point3 line_eq) {
  std::ostringstream out;
  if (fabs(line_eq.x) > 1E-4) {
    if (line_eq.x != 1)
      out << line_eq.x << " ";
    out << "x";
  } // end x != 0

  if (fabs(line_eq.y) > 1E-4) {
    out << (out.str().size() > 0 ? " + " : "");
    if (line_eq.y != 1)
      out << line_eq.y << " ";
    out << "y";
  } // end y != 0

  if (fabs(line_eq.z) > 1E-4)
    out << " + " << line_eq.z;

  out << " = 0";
  return out.str();

}

////////////////////////////////////////////////////////////////////////////////
//cut:line_equation

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

////////////////////////////////////////////////////////////////////////////////
//cut:angle_between_two_lines

/*!
 * \brief   returns the angle formed at the intersection between the two lines
     *
 * \param   l1 equation of the first line
 * \param   l2 equation of the second line
 * \return  the angle between 0 and PI
 */
template<class Point3>
static inline double angle_between_two_lines(const Point3 & l1,
                                             const Point3 & l2) {
  FooPoint2d vdir_l1 (-l1.y, l1.x);
  FooPoint2d vdir_l2 (-l2.y, l2.x);
  return absolute_angle_between_two_vectors(vdir_l1, vdir_l2);
}

////////////////////////////////////////////////////////////////////////////////
//cut:interLine

/*!
 * \brief   computes the intersection of two lines
 * of equations a_i* x + b_i * y + c_i = 0
 */
template<class Point2>
static inline Point2 interLine(const double a1, const double b1, const double c1,
                               const double a2, const double b2, const double c2) {
  double x = 1.f * (+c1 * b2 - c2 * b1) / (b1 * a2 - a1 * b2);
  double y = 1.f * (-c1 * a2 + c2 * a1) / (b1 * a2 - a1 * b2);
  return Point2(x, y);
}

////////////////////////////////////////////////////////////////////////////////
//cut:barycenter_of_two_points

/*!
 * \brief   returns alpha * A + (1 - alpha) * B
 * \param alpha : if 0, returns B, if 1, returns A
 */
template<class Point2>
static inline Point2 barycenter_of_two_points(const double alpha,
                                              const Point2 & A,
                                              const Point2 & B) {
  return Point2 (alpha * A.x + (1.f - alpha) * B.x,
                 alpha * A.y + (1.f - alpha) * B.y);

}

////////////////////////////////////////////////////////////////////////////////
//cut:counter_clockwise

/*!
 determining if three points are listed in a counterclockwise order.
 So say you have three points A, B and C.
 If the slope of the line AC is greater than the slope of the line AB
 then the three points are listed in a counterclockwise order.

 From http://www.bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
 \param A
 \param B
 \param C
 \return bool
*/
template<class Point2>
static inline bool counter_clockwise(const Point2 & A,
                                     const Point2 & B,
                                     const Point2 & C) {
  return (C.y-A.y)*(B.x-A.x) > (B.y-A.y)*(C.x-A.x);
}

//cut:colinear

template<class Point2>
static inline bool colinear(const Point2 & A,
                            const Point2 & B,
                            const Point2 & C) {
  return ((C.y-A.y)*(B.x-A.x) == (B.y-A.y)*(C.x-A.x));
}

////////////////////////////////////////////////////////////////////////////////
//cut:intersect_segments

/*!
 determines if two line segments AB and CD intersect.
 These intersect if and only if
 points A and B are separated by segment CD and
 points C and D are separated by segment AB.

 If points A and B are separated by segment CD
 then ACD and BCD should have opposite orientation
 meaning either ACD or BCD is counterclockwise but not both

 From http://www.bryceboe.com/2006/10/23/line-segment-intersection-algorithm/

 \return true if they intersect
*/
template<class Point2>
static inline bool intersect_segments(const Point2 & A, const Point2 & B,
                                      const Point2 & C, const Point2 & D) {
  // one common end
  if (A == C || A == D || B == C || B == D)
    return true;

  // check parallel segments
  if (colinear(A, B, C) && colinear(A, B, D)) {
    if
        (// C in AB
         (distance_points_squared(A, C) <= distance_points_squared(A, B)
          && distance_points_squared(B, C) <= distance_points_squared(A, B))
         || // D in AB
         (distance_points_squared(A, D) <= distance_points_squared(A, B)
          && distance_points_squared(B, D) <= distance_points_squared(A, B))
         || // A in CD
         (distance_points_squared(C, A) <= distance_points_squared(C, D)
          && distance_points_squared(D, A) <= distance_points_squared(C, D))
         || // B in CD
         (distance_points_squared(C, B) <= distance_points_squared(C, D)
          && distance_points_squared(D, B) <= distance_points_squared(C, D))
         )
      return true;
    else
      return false;
  }

  return (counter_clockwise(A,C,D) != counter_clockwise(B,C,D))
      && (counter_clockwise(A,B,C) != counter_clockwise(A,B,D));
}

////////////////////////////////////////////////////////////////////////////////
//cut:dist_segments

template<class Point2>
static inline double dist_segments(const Point2 & A, const Point2 & B,
                                   const Point2 & C, const Point2 & D) {
  if (intersect_segments(A, B, C, D))
    return 0;
  //    //printf("%g, %g, %g, %g",
  //                 distance_point_segment(A, C, D), distance_point_segment(B, C, D),
  //                 distance_point_segment(C, A, B), distance_point_segment(D, A, B));
  return fmin(
        fmin(distance_point_segment(A, C, D), distance_point_segment(B, C, D)),
        fmin(distance_point_segment(C, A, B), distance_point_segment(D, A, B))
        );
}


////////////////////////////////////////////////////////////////////////////////
//cut:barycenter

/*!
 * \brief   computes the barycenter of a std::vector of points
     *
 * \param   src the std::vector
 * \return  the barycenter
 */
template<class Point2>
static inline Point2 barycenter(const std::vector<Point2> & src) {
  if (src.empty())
    return Point2();
  double x = 0, y = 0;
  unsigned int npts = src.size();
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    x += src[pt_idx].x;
    y += src[pt_idx].y;
  } // end loop pt_idx
  return Point2(x / npts, y / npts);
}

////////////////////////////////////////////////////////////////////////////////

///*! a templated version of the barycenter function */
//template<class Point2>
//static inline void barycenter(const float& t,
//                              const Point2 & begin, const Point2 & end,
//                              Point2 & rep) {
//    rep.x = t * begin.x + (1.0 - t) * end.x;
//    rep.y = t * begin.y + (1.0 - t) * end.y;
//}

////////////////////////////////////////////////////////////////////////////////
//cut:barycenter4

/*!
 * \brief   returns the point as follows
     *
 *       <--tx-->
 *      A ___________________ B
 *  ^   |                     |
 *  |   |                     |
 *  ty  |                     |
 *  |   |        P            |
 *  v   |        +            |
 *      |                     |
 *      C ___________________ D
 */
template<class Point2i, class Point2f>
static inline void barycenter4(const float& tx, const float& ty,
                               const Point2i & A, const Point2i & B,
                               const Point2i & C, const Point2i & D,
                               Point2f & rep) {
  float P1x = A.x + ty * (D.x - A.x);
  float P2x = B.x + ty * (C.x - B.x);
  rep.x = P1x + tx * (P2x - P1x);

  float P1y = A.y + ty * (D.y - A.y);
  float P2y = B.y + ty * (C.y - B.y);
  rep.y = P1y + tx * (P2y - P1y);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   equation of the line between two points.
 * Cf http://fr.wikipedia.org/wiki/%C3%89quation_de_droite
 */
//template<class P> static inline Point3 line_equation(P A, P B);


////////////////////////////////////////////////////////////////////////////////
//cut:boundingBox_vec

/*!
 * \brief   get the bounding box of the points in a vector
 * \param   pts a vector of 2D points
 */
template<class Pt2Iterable, class _Bbox>
static _Bbox boundingBox_vec(const Pt2Iterable & pts) {
  //printf("boundingBox_vec()");
  if (pts.size() == 0)
    return _Bbox(-1,-1,-1, -1);
  typedef typename Pt2Iterable::value_type _Point2;
  typename _Point2::value_type
      xMin = pts.front().x, yMin = pts.front().y,
      xMax = xMin, yMax = yMin;

  for (unsigned int pt_idx = 0; pt_idx < pts.size(); ++pt_idx) {
    const _Point2* pt = &pts[pt_idx];
    if (pt->x < xMin)
      xMin = pt->x;
    else if (pt->x > xMax)
      xMax = pt->x;

    if (pt->y < yMin)
      yMin = pt->y;
    else if (pt->y > yMax)
      yMax = pt->y;
  } // end loop pt_idx

  return _Bbox(xMin, yMin, 1 + xMax - xMin, 1 + yMax - yMin);
}


////////////////////////////////////////////////////////////////////////////////
//cut:convert_coordinates

/*!
  Converts a point from a coordinate system to another
  \param bbox_src the bounding box of the source system
  \param bbox_dst the bounding box of the dest system
  \param query the point in the source system
  \return the point with coordinates in the dest system
  */
template<class _Bbox_src, class _Bbox_dst,
         class _Point2_src, class _Point2_dst>
static inline _Point2_dst
convert_coordinates(const _Bbox_src & bbox_src,
                    const _Bbox_dst & bbox_dst,
                    const _Point2_src & query
                    ) {
  //    //printf("bbox:%f+%f -> %i+%i, diff:%f",
  //                 bbox_src.x, bbox_src.width,
  //                 bbox_dst.x, bbox_dst.width,
  //                 query.x - bbox_src.x);
  _Point2_dst ans(
        bbox_dst.x +
        (query.x - bbox_src.x) * bbox_dst.width / bbox_src.width,
        bbox_dst.y +
        (query.y - bbox_src.y) * bbox_dst.height / bbox_src.height
        );
  return ans;
}

////////////////////////////////////////////////////////////////////////////////
//cut:find_closest_points_brute_force

/*!
 \fn find_closest_points_brute_force
 \param set1
 \param set2
 \param best_permutation a copy of the best permutation
 \return double the squared distance with the best matching
*/
template<class Point2>
static inline double find_closest_points_brute_force(
    const std::vector<Point2> & set1,
    const std::vector<Point2> & set2,
    Permutation & best_permutation) {

  // size check
  if (set1.size() != set2.size()) {
    throw std::invalid_argument("the two sets have a different size (%li != %li",
                set1.size(), set2.size());
  }

  // prepair the stuff
  double best_distance_squared = -1;
  Permutation current_permutation;
  create_listing(current_permutation, set1.size());
  typename std::vector<Point2>::const_iterator set1_it;
  Permutation::const_iterator perm_it;
  double current_distance_squared;

  bool next_was_found = true;
  while (next_was_found) {
    //printf("current_permutation:'%s'",
    //accessible_to_string(current_permutation).c_str());
    // do the stuff
    set1_it = set1.begin();
    perm_it = current_permutation.begin();
    current_distance_squared = 0;
    for (uint idx = 0; idx < set1.size(); ++idx) {
      const Point2* set2_pt = &set2[ *perm_it ];
      current_distance_squared += pow(set1_it->x - set2_pt->x, 2)
                                  + pow(set1_it->y - set2_pt->y, 2);
      // stop if the best distance is already reached
      if (current_distance_squared > best_distance_squared) {
        //printf("Breaking...");
        break;
      }
      // advance iterators
      ++set1_it;
      ++perm_it;
    } // end loop set1_it

    if (best_distance_squared == -1
        || current_distance_squared < best_distance_squared) {
      // keep the data
      best_distance_squared = current_distance_squared;
      best_permutation = current_permutation;

    } // end new best_distance_squared

    next_was_found =
                     permutation_find_next_lexicographical_ordering(current_permutation);
  } // end while next_was_found

  return best_distance_squared;
}

//cut

} // end geometry_utils

#endif /* GEOMETRYUTILS_H_ */

