#ifndef LASER_UTILS_H
#define LASER_UTILS_H

// ROS
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
// utils
#include "pt_utils.h"

namespace vision_utils {

////////////////////////////////////////////////////////////////////////////////
//cut:convert_sensor_data_to_xy
//! convert from polar to xy coordinates for a laser data
template<class Pt2>
static inline void convert_sensor_data_to_xy(const sensor_msgs::LaserScan & laser_msg,
                                             std::vector<Pt2> & out_vector) {
  out_vector.clear();
  out_vector.reserve(laser_msg.ranges.size());
  const float* curr_range = &(laser_msg.ranges[0]);
  float curr_angle = laser_msg.angle_min;
  for (unsigned int idx = 0; idx < laser_msg.ranges.size(); ++idx) {
    //printf("idx:%i, curr_range:%g", idx, *curr_range);
    out_vector.push_back(Pt2(*curr_range * cos(curr_angle),
                             *curr_range * sin(curr_angle)));
    ++curr_range;
    curr_angle += laser_msg.angle_increment;
  } // end loop idx
} // end convert_sensor_data_to_xy()

////////////////////////////////////////////////////////////////////////////////
//cut:convert_xy_vec_frame
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
