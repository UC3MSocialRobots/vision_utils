#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H
// ROS
#include <std_msgs/Empty.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
// vision_utils
#include <vision_utils/spline/spline.h>

namespace vision_utils {

template<class _Msg>
class Interpolator {
public:
  //! train with two vectors
  bool train(const std::vector<double> & T,
             const std::vector<_Msg> & msgs) {
    if (!Tmsgs_sanity_checks(T, msgs))
      return false;
    std::vector<double> X;
    unsigned int n = T.size();
    for (unsigned int i = 0; i < n; ++i)
      X.push_back(msgs[i].data);
    _splines.resize(1);
    _splines.front().set_points(T, X);
    return true;
  }
  //! predict once trained
  bool predict(const double & t, _Msg & out) {
    if (_splines.size() != 1) {
      ROS_WARN("Spline was not trained with the good type of data!");
      return false;
    }
    out.data = (_splines.front())(t);
    return true;
  }

protected:
  bool Tmsgs_sanity_checks(const std::vector<double> & T,
                          const std::vector<_Msg> & msgs) {
    if (msgs.size() != T.size()) {
      ROS_WARN("Inconsistent T-msgs size:%li != %li", T.size(), msgs.size());
      return false;
    }
    if (T.empty()) {
      ROS_WARN("Cannot train with an empty time vector!");
      return false;
    }
    return true;
  } // end Tmsgs_sanity_checks()

  std::vector<tk::spline> _splines;
}; // end class Interpolator

////////////////////////////////////////////////////////////////////////////////
// template specifications
template<>
bool Interpolator<std_msgs::ColorRGBA>::train(const std::vector<double> & T,
                                              const std::vector<std_msgs::ColorRGBA> & msgs) {
  if (!Tmsgs_sanity_checks(T, msgs))
    return false;
  unsigned int n = T.size(), dim = 4;
  std::vector<std::vector<double> > RGBA(dim);
  for (unsigned int i = 0; i < n; ++i) {
    RGBA[0].push_back(msgs[i].r);
    RGBA[1].push_back(msgs[i].g);
    RGBA[2].push_back(msgs[i].b);
    RGBA[3].push_back(msgs[i].a);
  }
  _splines.resize(dim);
  for (unsigned int i = 0; i < dim; ++i)
    _splines[i].set_points(T, RGBA[i]);
  return true;
}
template<>
bool Interpolator<std_msgs::ColorRGBA>::predict(const double & t,
                                                std_msgs::ColorRGBA & out) {
  if (_splines.size() != 4) {
    ROS_WARN("Spline was not trained with the good type of data!");
    return false;
  }
  out.r = (_splines[0])(t);
  out.g = (_splines[1])(t);
  out.b = (_splines[2])(t);
  out.a = (_splines[3])(t);
  return true;
}

} // end namespace vision_utils

#endif // INTERPOLATOR_H
