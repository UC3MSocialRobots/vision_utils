/*!
  \file        ros_camera_info_to_binary.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/11

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

A simple node that enables saving the camera info
of a given camera into a binary file.

Output: in the current folder, a file

\section Parameters
  - \b "~camera_info_topic"
        [string] (default: "/kinect_only_camera/depth_registered/camera_info")
        Where we will get the camera info.

  - \b "~filename"
        [std::string] (default: "camera_info.dat")
        The filename for the written camera info files.

\section Subscriptions
  - \b  {camera_info_topic}
        [sensor_msgs::CameraInfo]
        The input topics for the camera info.
        Such a topic can be published by the OpenNI node for instance.

\section Publications: None.

\example
rosrun vision ros_camera_info_to_binary _camera_info_topic:=/kinect_only_camera/depth_registered/camera_info  _filename:=A00365A82054035A_depth.dat
rosrun vision ros_camera_info_to_binary _camera_info_topic:=/kinect_only_camera/rgb/camera_info  _filename:=A00365A82054035A_rgb.dat

 */

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/CameraInfo.h>
// AD
#include <string/string_utils_ros.h>

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_camera_info_to_binary");
  ros::NodeHandle nh_public, nh_private("~");
  std::string filename = "camera_info.dat";
  // get params
  std::string camera_info_topic = "/kinect_only_camera/depth_registered/camera_info";
  nh_private.param("camera_info_topic", camera_info_topic, camera_info_topic);
  nh_private.param("filename", filename, filename);

  printf("Subscribing to camera_info:'%s', filename:'%s'.\n",
         nh_public.resolveName(camera_info_topic).c_str(),
         filename.c_str());

  sensor_msgs::CameraInfoConstPtr camera_info_msg
      = ros::topic::waitForMessage<sensor_msgs::CameraInfo>
      (camera_info_topic, nh_public);
  string_utils_ros::ros_object_to_file(filename, *camera_info_msg);
  printf("Succesfully wrote sensor_msgs::CameraInfo to '%s'\n", filename.c_str());
  return 0;
}
