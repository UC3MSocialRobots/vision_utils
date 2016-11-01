#ifndef MARKER_UTILS_H
#define MARKER_UTILS_H

#include <visualization_msgs/Marker.h>

namespace vision_utils {

inline void make_header
(visualization_msgs::Marker & msg,
 int type,
 const std::string marker_namespace,
 const float size,
 const float r, const float g, const float b, const float a = 1,
 const std::string & frame = "/base_link")
{
  msg.type = type;
  msg.header.frame_id = frame;
  msg.header.stamp = ros::Time::now();
  msg.ns = marker_namespace;
  msg.action = 0; // 0: add/modify - 2: delete
  msg.lifetime = ros::Duration(1); // 1 sec
  // color
  msg.color.r = r;
  msg.color.g = g;
  msg.color.b = b;
  msg.color.a = a;
  // size of the spheres
  msg.scale.x = size;
  msg.scale.y = size;
  msg.scale.z = size;
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
inline void list_points2_as_primitives
(visualization_msgs::Marker & msg,
 const std::vector<Pt2> & pts,
 const std::string marker_namespace,
 const float & z,
 const float size,
 const float r, const float g, const float b, const float a = 1,
 const std::string & frame = "/base_link",
 int type = visualization_msgs::Marker::SPHERE_LIST)
{
  make_header(msg, type, marker_namespace, size, r, g, b, a, frame);
  // clear pose otherwise it is used as a constant translation
  msg.pose = geometry_msgs::Pose();
  // points
  msg.points.clear();
  msg.points.reserve(pts.size());
  for (unsigned int pt_idx = 0; pt_idx < pts.size(); ++pt_idx) {
    msg.points.push_back(geometry_msgs::Point());
    msg.points.back().x = pts[pt_idx].x;
    msg.points.back().y = pts[pt_idx].y;
    msg.points.back().z = z;
  } // end loop pt_idx
}


////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
inline void list_points_as_primitives
(visualization_msgs::Marker & msg,
 const std::string marker_namespace,
 const std::vector<Pt3> & pts,
 const float size,
 const float r, const float g, const float b, const float a = 1,
 const std::string & frame = "/base_link",
 int type = visualization_msgs::Marker::SPHERE_LIST)
{
  make_header(msg, type, marker_namespace, size, r, g, b, a, frame);
  // clear pose otherwise it is used as a constant translation
  msg.pose = geometry_msgs::Pose();
  // points
  msg.points.clear();
  msg.points.reserve(pts.size());
  for (unsigned int pt_idx = 0; pt_idx < pts.size(); ++pt_idx) {
    msg.points.push_back(geometry_msgs::Point());
    msg.points.back().x = pts[pt_idx].x;
    msg.points.back().y = pts[pt_idx].y;
    msg.points.back().z = pts[pt_idx].z;
  } // end loop pt_idx
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
inline void list_point_as_primitives
(visualization_msgs::Marker & msg,
 const std::string marker_namespace,
 const Pt3 & pt,
 const float size,
 const float r, const float g, const float b, const float a = 1,
 const std::string & frame = "/base_link",
 int type = visualization_msgs::Marker::SPHERE)
{
  make_header(msg, type, marker_namespace, size, r, g, b, a, frame);
  // position
  msg.pose.position.x = pt.x;
  msg.pose.position.y = pt.y;
  msg.pose.position.z = pt.z;
  // orientation
  msg.pose.orientation.w = 1;
  msg.pose.orientation.z = 1;
}

} // end namespace vision_utils

#endif // MARKER_UTILS_H
