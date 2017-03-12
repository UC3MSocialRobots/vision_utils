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
  Interpolator() : _msg_dim(-1) {}

  //! train with two vectors
  bool train(const std::vector<double> & T,
             const std::vector<_Msg> & msgs) {
    if (!Tmsgs_sanity_checks(T, msgs))
      return false;
    // get the number of fields of the message
    std::vector<double> values_buffer;
    if (!msg2dvec(msgs.front(), values_buffer))
      return false;
    _msg_dim = values_buffer.size();
    if (_msg_dim == 0) { // empty messages
      return true;
    }
    ROS_INFO("Interpolator: training %i splines...", _msg_dim);
    // for each field, get all of its values across time
    std::vector< std::vector<double> > X;
    X.resize(_msg_dim);
    unsigned int nT = T.size();
    for (unsigned int t = 0; t < nT; ++t) {
      if (!msg2dvec(msgs[t], values_buffer))
        return false;
      for (int j = 0; j < _msg_dim; ++j)
        X[j].push_back(values_buffer[j]);
    } // end for t
    // now train
    _splines.resize(_msg_dim);
    for (int i = 0; i < _msg_dim; ++i) {
      _splines[i].set_points(T, X[i]);
    } // end for i
    return true;
  } // end train()

  //! train with two vectors of strings
  bool train(const std::vector<double> & T,
             const std::vector<std::string> & msgs_str) {
    unsigned int nmsgs = msgs_str.size();
    std::vector<_Msg> msgs(nmsgs);
    for (unsigned int i = 0; i < nmsgs; ++i) {
      if (!vision_utils::string2msg(msgs_str[i], msgs[i]))
          ROS_WARN("string2msg('%s') failed!", msgs_str[i].c_str());
    } // end for i
    return train(T, msgs);
  } // end train()

  //! predict once trained
  bool predict(const double & t, _Msg & out) const {
    if (_msg_dim == 0) { // empty messages
      return true;
    }
    if (_msg_dim < 0 || _splines.size() != _msg_dim) {
      ROS_WARN("Spline was not trained with the good type of data!");
      return false;
    }
    // get each of the fields
    std::vector<double> values_buffer(_msg_dim);
    for (int i = 0; i < _msg_dim; ++i)
      values_buffer[i] = (_splines[i])(t);
    // now fill the message
    return vision_utils::dvec2msg(values_buffer, out);
  } // end predict()

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

  int _msg_dim;
  std::vector<tk::spline> _splines;
}; // end class Interpolator

} // end namespace vision_utils

#endif // INTERPOLATOR_H
