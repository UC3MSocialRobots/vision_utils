#ifndef FOO_POINT_H
#define FOO_POINT_H

#include <sstream>

namespace geometry_utils {

/*! a generic templated class for 2D points.
  It contains a x and a y field so as to be compatible
  with OpenCV cv::Point*
*/
template<class _Type>
class FooPoint2 {
public:
  typedef _Type value_type; //!< an alias for the data type (compat with OpenCV)

  //! a constructor without arguments
  FooPoint2() : x(0), y(0) {}

  //! a constructor
  FooPoint2(const _Type & x_, const _Type & y_) :
    x(x_), y(y_) {}

  //! the == operator, for some std algorithms
  bool operator== (const FooPoint2<_Type>& b) const {
    return (x == b.x) && (y == b.y);
  }

  //! the + operator, that adds field by field
  FooPoint2<_Type> operator + (const FooPoint2<_Type>& B) const {
    return FooPoint2<_Type>(x + B.x, y + B.y);
  }

  //! the - operator, that substracts field by field
  FooPoint2<_Type> operator - (const FooPoint2<_Type>& B) const {
    return FooPoint2<_Type>(x - B.x, y - B.y);
  }

  //! the * operator, that multiplies field by field
  FooPoint2<_Type> operator * (const float & alpha) const {
    return FooPoint2<_Type>(alpha * x, alpha * y);
  }

  //! the dot operator
  inline _Type dot(const FooPoint2<_Type>& B) const {
    return x * B.x + y * B.y;
  }

  //! define the output to a stream
  friend std::ostream & operator << (std::ostream & stream,
                                     const FooPoint2<_Type> & P) {
    stream << '[' << P.x << ", " << P.y << ']';
    return stream;
  }

  //! return a string representation of the point
  inline std::string to_string() const {
    std::ostringstream ans;
    ans << *this;
    return ans.str();
  }

  _Type x; //!< the first data field
  _Type y; //!< the second data field
}; // end FooPoint2

typedef FooPoint2<int> FooPoint2i;
typedef FooPoint2<float> FooPoint2f;
typedef FooPoint2<double> FooPoint2d;

////////////////////////////////////////////////////////////////////////////////

/*! a generic templated class for 3D points.
  It contains a x and a y field so as to be compatible
  with OpenCV cv::Point*
*/
template<class _Type>
class FooPoint3 {
public:
  typedef _Type value_type; //!< an alias for the data type (compat with OpenCV)

  //! a constructor without arguments
  FooPoint3() : x(0), y(0), z(0) {}

  //! a constructor
  FooPoint3(const _Type & x_, const _Type & y_, const _Type & z_) :
    x(x_), y(y_), z(z_) {}

  //! the == operator, for some std algorithms
  bool operator== (const FooPoint3<_Type>& b) const {
    return (x == b.x) && (y == b.y) && (z == b.z);
  }

  //! the + operator, that adds field by field
  FooPoint3<_Type> operator + (const FooPoint3<_Type>& B) const {
    return FooPoint3<_Type>(x + B.x, y + B.y, z + B.z);
  }

  //! the - operator, that substracts field by field
  FooPoint3<_Type> operator - (const FooPoint3<_Type>& B) const {
    return FooPoint3<_Type>(x - B.x, y - B.y, z - B.z);
  }

  //! the * operator, that multiplies field by field
  FooPoint3<_Type> operator * (const float & alpha) const {
    return FooPoint3<_Type>(alpha * x, alpha * y, alpha * z);
  }

  //! the dot operator
  inline _Type dot(const FooPoint3<_Type>& B) const {
    return x * B.x + y * B.y + z * B.z;
  }

  //! define the output to a stream
  friend std::ostream & operator << (std::ostream & stream,
                                     const FooPoint3<_Type> & P) {
    stream << '[' << P.x << ", " << P.y << ", " << P.z << ']';
    return stream;
  }

  //! return a string representation of the point
  inline std::string to_string() const {
    std::ostringstream ans;
    ans << *this;
    return ans.str();
  }

  _Type x; //!< the first data field
  _Type y; //!< the second data field
  _Type z; //!< the third data field
}; // end FooPoint3

typedef FooPoint3<int> FooPoint3i;
typedef FooPoint3<float> FooPoint3f;
typedef FooPoint3<double> FooPoint3d;

////////////////////////////////////////////////////////////////////////////////

} // end geometry_utils

#endif // FOO_POINT_H
