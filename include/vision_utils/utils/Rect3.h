#ifndef RECT3_H
#define RECT3_H

#include <sstream>
#include <vision_utils/utils/debug_utils.h>

namespace geometry_utils {

/*! \class Rect3_
    A templated 3D rectangle class (cube class)
    From the nestk library
    (Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010)
*/
template <class T>
class Rect3_
{
public:
  //////////////////////////////////////////////////////////////////////////////

  //! ctor
  Rect3_() :
    x(0), y(0), z(0),
    width(-1), height(-1), depth(-1)
  {}

  //////////////////////////////////////////////////////////////////////////////

  template<class T1, class T2>
  static inline void extend_span(const T1 & new_val,
                                 T2 & old_origin, T2 & old_span) {
    if (new_val < old_origin) { // decrease the origin
      old_span = old_span + (old_origin - new_val); // 1st, update span
      old_origin = new_val;
    } else if (new_val > old_origin + old_span) { // increase the span
      // old_origin unchanged
      old_span = new_val - old_origin;
    }
  } // end extend_span()

  template<class Point3>
  void extendToInclude(const Point3& p)
  {
    if (width < T(0) || height < T(0) || depth < T(0))
    {
      x = p.x;
      y = p.y;
      z = p.z;
      width = 0;
      height = 0;
      depth = 0;
    }
    else
    {
      extend_span(p.x, x, width);
      extend_span(p.y, y, height);
      extend_span(p.z, z, depth);
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  bool isEmpty() const
  {
    return width < 0 || height < 0 || depth < 0;
  }

  //////////////////////////////////////////////////////////////////////////////

  template<class Point3>
  Point3 centroid() const
  {
    return Point3(x+(width/T(2)), y+(height/T(2)), z + (depth / T(2)));
  }

  //////////////////////////////////////////////////////////////////////////////

  //! a short-hand point creator
  template<class Point3>
  static const inline Point3 create_pt(const double & x,
                                       const double & y,
                                       const double & z) {
    Point3 ans;
    ans.x = x; ans.y = y; ans.z = z;
    return ans;
  } // end create_pt()

  /*! push all vertices  of the rectangle into the vector
         H+---------------------+G
        / |                   / |
      D+---------------------+C |
       |  |                  |  |
       |  |                  |  | height
       |  |                  |  |
       |  |                  |  |
       |  |                  |  |
       | E+ -----------------|--+F
       | /                   | / depth
      A+---------------------+B
             width
    */
  template<class Pt3, class Pt3Vector>
  inline void queue_all_vertices(Pt3Vector & pts) const {
    // create all the corners
    Pt3
        A = create_pt<Pt3>(x, y, z),
        B = create_pt<Pt3>(x + width, y, z),
        C = create_pt<Pt3>(x + width, y + height, z),
        D = create_pt<Pt3>(x, y + height, z),
        E = create_pt<Pt3>(x, y, z + depth),
        F = create_pt<Pt3>(x + width, y, z + depth),
        G = create_pt<Pt3>(x + width, y + height, z + depth),
        H = create_pt<Pt3>(x, y + height, z + depth);
    // push the vertices - front side
    pts.push_back(A); pts.push_back(B);
    pts.push_back(B); pts.push_back(C);
    pts.push_back(C); pts.push_back(D);
    pts.push_back(D); pts.push_back(A);
    // back side
    pts.push_back(E); pts.push_back(F);
    pts.push_back(F); pts.push_back(G);
    pts.push_back(G); pts.push_back(H);
    pts.push_back(H); pts.push_back(E);
    // depth vertice
    pts.push_back(A); pts.push_back(E);
    pts.push_back(B); pts.push_back(F);
    pts.push_back(C); pts.push_back(G);
    pts.push_back(D); pts.push_back(H);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! a string conversion function
  std::string to_string() {
    std::ostringstream ans;
    ans << '(' << x << ", " << y << ", " << z << ")"
        << "+(" << width << ", " << height << ", " << depth << ")"
        << " = (" << x + width << ", " << y + height << ", " << z + depth << ")";
    return ans.str();
  }

public:
  T x,y,z,width,height,depth;
};

//////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   get the bounding box of the points in a vector
 * \param   pts a vector of 3D points
 */
template<class _T, class _Pt3, class _Pt3Vector>
static Rect3_<_T>  boundingBox_vec3(const _Pt3Vector & pts) {
  maggieDebug3("boundingBox_vec3()");
  if (pts.empty()) {
    Rect3_<_T> ans;
    ans.x = ans.y = ans.z = ans.width = ans.height = ans.depth = -1;
    return ans;
  }
  _T  xMin = pts[0].x, yMin = pts[0].y, zMin = pts[0].z,
      xMax = xMin, yMax = yMin, zMax = zMin;

  for (unsigned int pt_idx = 0; pt_idx < pts.size(); ++pt_idx) {
    const _Pt3* pt = &pts[pt_idx];
    if (pt->x < xMin)
      xMin = pt->x;
    else if (pt->x > xMax)
      xMax = pt->x;

    if (pt->y < yMin)
      yMin = pt->y;
    else if (pt->y > yMax)
      yMax = pt->y;

    if (pt->z < zMin)
      zMin = pt->z;
    else if (pt->z > zMax)
      zMax = pt->z;
  } // end loop pt_idx
  Rect3_<_T> ans;
  ans.x = xMin;
  ans.width = xMax - xMin;
  ans.y = yMin;
  ans.height = yMax - yMin;
  ans.z = zMin;
  ans.depth = zMax - zMin;
  return ans;
}

typedef Rect3_<float> Rect3f;
typedef Rect3_<double> Rect3d;

} // end namespace geometry_utils

#endif // RECT3_H
