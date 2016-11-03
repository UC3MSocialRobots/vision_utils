/*!
  \file        convert_xy_vec_frame.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/2
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
 */

#ifndef CONVERT_XY_VEC_FRAME_H
#define CONVERT_XY_VEC_FRAME_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>

namespace vision_utils {

template<class Pt2>
static inline bool convert_xy_vec_frame(const std_msgs::Header & src_header,
                                        const std::vector<Pt2> & src_vector,
                                        tf::TransformListener & _tf_listener,
                                        const std::string & dst_frame,
                                        std::vector<Pt2> & dst_vector,
                                        double & scan_z_dst_frame) {
  std::string src_frame = src_header.frame_id;
  if (src_frame.empty()) { // empty scan?
    printf("convert_xy_vec_frame(): empty source frame_id!");
    return false;
  }
  if (src_frame != dst_frame) {
    //printf_ONCE("Src and dest frame equal '%s'", dst_frame.c_str());
    dst_vector = src_vector;
    return true;
  }
  dst_vector.clear();
  unsigned int src_npts = src_vector.size();
  dst_vector.resize(src_npts);
  bool transform_ok =
      _tf_listener.waitForTransform(dst_frame,
                                    src_frame,
                                    src_header.stamp,
                                    ros::Duration(1));
  if (!transform_ok) {
    printf("Impossible to find tf '%s' -> %s'. Returning.\n",
             src_frame.c_str(), dst_frame.c_str());
    return false;
  }
  //printf"Success for finding tf '%s' -> %s'.", src_frame.c_str(), dst_frame.c_str());

  // convert all the points from src_vector
  geometry_msgs::PointStamped pt_stamped_in, pt_stamped_out;
  pt_stamped_in.point.z = 0;
  pt_stamped_in.header = src_header;
  try {
    for (unsigned int pt_idx = 0; pt_idx < src_npts; ++pt_idx) {
      copy2(src_vector[pt_idx], pt_stamped_in.point);
      _tf_listener.transformPoint(dst_frame,  ros::Time(0),
                                  pt_stamped_in, dst_frame, pt_stamped_out);
      copy2(pt_stamped_out.point, dst_vector[pt_idx]);
    } // end loop pt_idx

    // keep the z coordinate of the scan
    scan_z_dst_frame = pt_stamped_out.point.z;
    //printf"scan_z_dst_frame:%g", scan_z_dst_frame);

  } catch (tf::ExtrapolationException e) {
    printf("transform error:'%s'\n", e.what());
    return false;
  } // end try / catch
  return true;
} // end convert_xy_vec_frame()

} // end namespace vision_utils

#endif // LASER_UTILS_H

} // end namespace vision_utils

#endif // CONVERT_XY_VEC_FRAME_H
