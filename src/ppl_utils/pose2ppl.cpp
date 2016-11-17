/*!
  \file        pose2ppl.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/24

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHppl ANY WARRANTY; withppl even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

A class to convert stamped poses or points into people_msgs/People.

\section Parameters
  - \b "pose_topic"
        [string] (default: "")
        If non empty, the PoseStamped topic we will convert to people_msgs/People.

  - \b "point_topic"
        [string] (default: "")
        If non empty, the PointStamped topic we will convert to people_msgs/PeoplepointList.
        Note both "pose_topic" and "point_topic" can be used at the same time.

\section Subscriptions
  - \b {pose_topic}
        [geometry_msgs/PoseStamped]
        The pose topic to convert to PPL

  - \b {point_topic}
        [geometry_msgs/PointStamped]
        The point topic to convert to PPL

\section Publications
  - \b "~ppl"
        [geometry_msgs/PoseStamped]
        The converted topic(s).
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <people_msgs/People.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <vision_utils/ppl_attributes.h>

ros::Publisher ppl_pub;
std::string method_name = "pose2ppl";

////////////////////////////////////////////////////////////////////////////////

void pose_cb(const geometry_msgs::PoseStamped & msg) {
  ROS_INFO_THROTTLE(1, "pose2ppl:point_cb()");
  people_msgs::People ppl;
  vision_utils::set_method(ppl, method_name);
  ppl.header = msg.header;
  ppl.people.resize(1);
  people_msgs::Person* pp = &(ppl.people.front());
  //pp->header = ppl.header;
  pp->name = "NOREC";
  // pose
  pp->position = msg.pose.position;
  pp->reliability = 1;
  //pp->std_dev = .1; // foo value
  ppl_pub.publish(ppl);
}

////////////////////////////////////////////////////////////////////////////////

void point_cb(const geometry_msgs::PointStamped & msg) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = msg.header;
  pose_stamped.pose.position = msg.point;
  pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  pose_cb(pose_stamped);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose2ppl");
  ros::NodeHandle nh_public, nh_private("~");
  std::string pose_topic = "", point_topic = "";
  nh_private.param("pose_topic", pose_topic, pose_topic);
  nh_private.param("point_topic", point_topic, point_topic);
  nh_private.param("method_name", method_name, method_name);
  // make subscribers and publishers
  ppl_pub = nh_private.advertise<people_msgs::People>("ppl", 1);
  ros::Subscriber point_sub, pose_sub;
  if (point_topic.size() > 0) {
    point_sub = nh_public.subscribe(point_topic, 1, point_cb);
    printf("pose2ppl: subscribing to PointStamped topic '%s', emitting PPL to '%s'\n",
           point_sub.getTopic().c_str(), ppl_pub.getTopic().c_str());
  }
  if (pose_topic.size() > 0) {
    pose_sub = nh_public.subscribe(pose_topic, 1, pose_cb);
    printf("pose2ppl: subscribing to PoseStamped topic '%s', emitting PPL to '%s'\n",
           pose_sub.getTopic().c_str(), ppl_pub.getTopic().c_str());
  }
  ros::spin();
}
