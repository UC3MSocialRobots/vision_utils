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
#include <vision_utils/msg2string.h>
#include <vision_utils/string2msg.h>
#include <vision_utils/spline/spline.h>

namespace vision_utils {

template<class _Msg>
class Interpolator {
public:
  Interpolator() : _msg_dim(0) {}

  //! train with two vectors
  bool train(const std::vector<double> & T,
             const std::vector<_Msg> & msgs) {
    if (!Tmsgs_sanity_checks(T, msgs))
      return false;
    // get the number of fields of the message
    if (!msg2dvec(msgs.front(), _values_buffer))
      return false;
    _msg_dim = _values_buffer.size();
    ROS_INFO("Interpolator: training %i splines...", _msg_dim);
    // for each field, get all of its values across time
    std::vector< std::vector<double> > X;
    X.resize(_msg_dim);
    unsigned int nT = T.size();
    for (unsigned int t = 0; t < nT; ++t) {
      if (!msg2dvec(msgs[t], _values_buffer))
        return false;
      for (unsigned int j = 0; j < _msg_dim; ++j)
        X[j].push_back(_values_buffer[j]);
    } // end for t
    // now train
    _splines.resize(_msg_dim);
    for (unsigned int i = 0; i < _msg_dim; ++i) {
      _splines[i].set_points(T, X[i]);
    } // end for i
    return true;
  }
  //! predict once trained
  bool predict(const double & t, _Msg & out) {
    if (!_msg_dim || _splines.size() != _msg_dim) {
      ROS_WARN("Spline was not trained with the good type of data!");
      return false;
    }
    // get each of the fields
    _values_buffer.resize(_msg_dim);
    for (unsigned int i = 0; i < _msg_dim; ++i)
      _values_buffer[i] = (_splines[i])(t);
    // now fill the message
    return vision_utils::dvec2msg(_values_buffer, out);
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

  unsigned int _msg_dim;
  std::vector<tk::spline> _splines;
  std::vector<double> _values_buffer;
}; // end class Interpolator

} // end namespace vision_utils

#endif // INTERPOLATOR_H
