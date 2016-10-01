/*!
  \file        ppl2marker.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/10/19

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

\todo Description of the file

\section Parameters
  - \b "input_topic"
        [string] (default: "ppl")
        Where to get the input people list

  - \b "red", "green", "blue"
        [double] (default: 1, 0, 0)
        The red, green, blue value of the ouptut spheres

\section Subscriptions
  - \b {input_topic}
        [people_msgs_rl/PeoplePoseList]
        The people list

\section Publications
  - \b "ppl_marker"
        [visualization_msgs::Marker]
        The marker

 */

#include <ros/ros.h>
#include <people_msgs_rl/PeoplePoseList.h>
#include "vision_utils/utils/marker_utils.h"
#include "vision_utils/utils/geometry_utils.h"
#include "vision_utils/utils/pt_utils.h"

ros::Publisher marker_pub;
ros::Subscriber ppl_sub;
visualization_msgs::Marker marker;
std::vector<geometry_utils::FooPoint3f> detected_faces;
double red = 1, green = 0, blue = 0;

////////////////////////////////////////////////////////////////////////////////

//! the callback when a user is detected
void ppl_cb(const people_msgs_rl::PeoplePoseListConstPtr & ppl) {
  unsigned int n_people = ppl->poses.size();
  marker.header = ppl->header;
  //ROS_INFO_THROTTLE(5, "ppl_cb(method.'%s')", ppl->method.c_str());
  ROS_INFO_ONCE("ppl_cb(method.'%s')", ppl->method.c_str());
  if (n_people == 0) {
    return; // not much to do here
  }

  //// publish faces as circles
  // use  object ID useful in conjunction with the namespace
  // for manipulating and deleting the object later
  marker.id = 0;
  detected_faces.resize(n_people);
  for (unsigned int detec_idx = 0; detec_idx < n_people; ++detec_idx)
    pt_utils::copy3(ppl->poses[detec_idx].head_pose.position, detected_faces[detec_idx]);
  marker_utils::list_points_as_primitives
      (marker, ppl_sub.getTopic(), detected_faces, .1, red, green, blue, 1,
       ppl->header.frame_id, visualization_msgs::Marker::SPHERE_LIST);
  marker_pub.publish(marker);

  //// make a circle with a radius of "std dev"
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = .02; // scale.x is used and it controls the width of the line segments.
  for (unsigned int detec_idx = 0; detec_idx < n_people; ++detec_idx) {
    double std_dev = ppl->poses[detec_idx].std_dev;
    unsigned int n_points_circle = 10;
    marker.points.resize(n_points_circle);
    for (unsigned int pt_idx = 0; pt_idx < n_points_circle; ++pt_idx) {
      double angle = 2 * M_PI * pt_idx / (n_points_circle - 1); // -1 for finishing with 2PI
      marker.points[pt_idx].x = detected_faces[detec_idx].x + std_dev * cos(angle);
      marker.points[pt_idx].y = detected_faces[detec_idx].y + std_dev * sin(angle);
      marker.points[pt_idx].z = detected_faces[detec_idx].z;
    } // end loop pt_idx
    ++marker.id;
    marker_pub.publish(marker);
  } // end for detec_idx

  //// write method and std dev
  // use  object ID useful in conjunction with the namespace
  // for manipulating and deleting the object later
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.scale.z = .1; // scale.z specifies the height of an uppercase "A".
  for (unsigned int detec_idx = 0; detec_idx < n_people; ++detec_idx) {
    std::ostringstream caption;
    caption << ppl->method << ":";
    caption << ppl->poses[detec_idx].person_name;
    //caption << (int) (100 * list->poses[detec_idx].std_dev) / 100.f;
    caption << "(" << (int) (100 * ppl->poses[detec_idx].confidence) << "%)";
    marker.text = caption.str();
    pt_utils::copy3(detected_faces[detec_idx], marker.pose.position);
    marker.pose.position.z += .1;
    ++marker.id;
    marker_pub.publish(marker);
  } // end for detec_idx

} // end ppl_cb()

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "ppl2marker");

  // subscribers
  ros::NodeHandle nh_public, nh_private("~");
  nh_private.param("red", red, red);
  nh_private.param("green", green, green);
  nh_private.param("blue", blue, blue);
  std::string input_topic = "ppl";
  nh_private.param("input_topic", input_topic, input_topic);
  ppl_sub = nh_public.subscribe<people_msgs_rl::PeoplePoseList>
      (input_topic, 1, ppl_cb);

  marker_pub = nh_public.advertise<visualization_msgs::Marker>
      ("ppl_marker", 10); // need a queue longer than 1!

  ROS_WARN("ppl2marker: getting PeoplePoseList on '%s', "
           "publishing marker on '%s'",
           ppl_sub.getTopic().c_str(),
           marker_pub.getTopic().c_str());

  ros::spin();
  return 0;
}

