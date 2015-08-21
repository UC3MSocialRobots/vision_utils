/*!
  \file        atan3D.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/24

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

Clases for handling proper conversion from 3D to 2D by removing a coordinate.
For instance, for seeing 3D points into a 2D viewer like Stage.

  - \b "reprojection_mode"
        [string] (default: "xy")
        The reprojection mode, i.e. the coordinates to keep for reprojecting to 2D.
        Valid values: "xy" or "xz".
 */

#ifndef ATAN3D_H
#define ATAN3D_H

#include <ros/node_handle.h>

/*! The reprojection mode, i.e. the coordinates to keep for reprojecting to 2D.
    The value should be such as:

                  (world)

                      ^  (first dimension)
                      |
                  <---+
  (second dimension)    0

  For instance, for the laser:
                  (world)

                      ^z
                      |
                  <---+
                  y    0
 so we pass the parameter "xy", equivalent to "+x+y", or "x+y", or "+xy".

  For the Kinect:
                   (world)

                      ^z
                      |
                      +--->
                       0  x
  so we pass the parameter  "-zx", equivalent to "-z+x".
*/
class ReprojectionMode {
public:
  enum Axis {
    plusX  = 0,
    minusX = 1,
    plusY  = 2,
    minusY = 3,
    plusZ  = 4,
    minusZ = 5
  };
  ReprojectionMode() : _was_param_checked(false) {
    set_reprojection_mode_xy();
  }

  inline void check_param(bool force_recheck = false) {
    if (_was_param_checked && !force_recheck)
      return;
    _was_param_checked = true;
    ros::NodeHandle nh_private("~");
    std::string reprojection_mode = "xy";
    nh_private.param("reprojection_mode", reprojection_mode, reprojection_mode);
    unsigned int letters_first = 1, letters_second = 1;
    if (reprojection_mode.size() == 0 || reprojection_mode.size() > 4) {
      printf("ReprojectionMode: invalid param '%s'\n", reprojection_mode.c_str());
      set_reprojection_mode_xy();
      return;
    }
    if (reprojection_mode.length() == 2)
      letters_first = letters_second = 1;
    else if (reprojection_mode.length() == 4)
      letters_first = letters_second = 2;
    else if (reprojection_mode.at(0) == '+' ||reprojection_mode.at(0) == '-')
      letters_first = 2;
    else // if (reprojection_mode.at(2) == '+')
      letters_second = 2;
    std::string axis1_str = reprojection_mode.substr(0, letters_first);
    std::string axis2_str = reprojection_mode.substr(letters_first);
    std::transform(axis1_str.begin(), axis1_str.end(), axis1_str.begin(), ::tolower);
    std::transform(axis2_str.begin(), axis2_str.end(), axis2_str.begin(), ::tolower);
    //  printf("axis1_str:'%s'\n", axis1_str.c_str());
    //  printf("axis2_str:'%s'\n", axis2_str.c_str());

    if (axis1_str == "x" || axis1_str == "+x")
      _axis1 = plusX;
    else if (axis1_str == "-x")
      _axis1 = minusX;
    else if (axis1_str == "y" || axis1_str == "+y")
      _axis1 = plusY;
    else if (axis1_str == "-y")
      _axis1 = minusY;
    else if (axis1_str == "z" || axis1_str == "+z")
      _axis1 = plusZ;
    else if (axis1_str == "-z")
      _axis1 = minusZ;
    else {
      printf("ReprojectionMode: invalid param '%s'\n", reprojection_mode.c_str());
      set_reprojection_mode_xy();
      return;
    }

    if (axis2_str == "x" || axis2_str == "+x")
      _axis2 = plusX;
    else if (axis2_str == "-x")
      _axis2 = minusX;
    else if (axis2_str == "y" || axis2_str == "+y")
      _axis2 = plusY;
    else if (axis2_str == "-y")
      _axis2 = minusY;
    else if (axis2_str == "z" || axis2_str == "+z")
      _axis2 = plusZ;
    else if (axis2_str == "-z")
      _axis2 = minusZ;
    else {
      printf("ReprojectionMode: invalid param '%s'\n", reprojection_mode.c_str());
      set_reprojection_mode_xy();
      return;
    }
    // printf("ReprojectionMode:axes:%i,%i\n", _axis1, _axis2);
  } // end check_param();

  inline void set_reprojection_mode_xy() { _axis1 = plusX; _axis2 = plusY;}

  inline void convert3Dto2D(const double x, const double y, const double z,
                            double &x2D, double &y2D) {
    check_param();
    if (_axis1 == plusX)              x2D = x;
    else if (_axis1 == minusX)        x2D = -x;
    else if (_axis1 == plusY)         x2D = y;
    else if (_axis1 == minusY)        x2D = -y;
    else if (_axis1 == plusZ)         x2D = z;
    else /*if (_axis1 == minusZ)*/    x2D = -z;

    if (_axis2 == plusX)              y2D = x;
    else if (_axis2 == minusX)        y2D = -x;
    else if (_axis2 == plusY)         y2D = y;
    else if (_axis2 == minusY)        y2D = -y;
    else if (_axis2 == plusZ)         y2D = z;
    else /*if (_axis2 == minusZ)*/    y2D = -z;
  }

  inline void convert2Dto3D(const double &x2D, const double &y2D, const double& z2D,
                            double & x, double & y, double & z) {
    check_param();
    x = y = z = z2D; // default value
    //convert3Dto2D(x2D, y2D, z2D, x, y, z);
    if (_axis1 == plusX)            x = x2D;
    else if (_axis1 == minusX)      x = -x2D;
    else if (_axis1 == plusY)       y = x2D;
    else if (_axis1 == minusY)      y = -x2D;
    else if (_axis1 == plusZ)       z = x2D;
    else /*if (_axis1 == minusZ)*/  z = -x2D;

    if (_axis2 == plusX)            x = y2D;
    else if (_axis2 == minusX)      x = -y2D;
    else if (_axis2 == plusY)       y = y2D;
    else if (_axis2 == minusY)      y = -y2D;
    else if (_axis2 == plusZ)       z = y2D;
    else /*if (_axis2 == minusZ)*/  z = -y2D;
  }

private:
  Axis _axis1;
  Axis _axis2;
  bool _was_param_checked;
}; // end class ReprojectionMode

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class Atan3D {
public:
  Atan3D() {}

  inline double atan2(const double x, const double y, const double z) {
    double x2D, y2D;
    _mode.convert3Dto2D(x, y, z, x2D, y2D);
    return std::atan2(y2D, x2D);
  }

private:
  ReprojectionMode _mode;
}; // end class Atan3D

#endif // ATAN3D_H
