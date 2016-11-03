/*!
  \file        read_camera_info_bag_files.h
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

#ifndef READ_CAMERA_INFO_BAG_FILES_H
#define READ_CAMERA_INFO_BAG_FILES_H
// std includes
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "vision_utils/file_exists.h"

namespace vision_utils {

/*!
 * Read the camera info objects from ROS bag files.
 * \param kinect_serial_number
 *   The serial number of the Kinect, for instance KINECT_SERIAL_ARNAUD()
 * \param depth_camera_info, rgb_camera_info
 *    Output: the depth and rgb camera info, filled by reading the bags.
 * \return true if success
 */
bool read_camera_info_bag_files(const std::string & kinect_serial_number,
                                sensor_msgs::CameraInfo & depth_camera_info,
                                sensor_msgs::CameraInfo & rgb_camera_info) {
  // load bag files
  // from http://www.ros.org/doc/api/rosbag/html/c++/
  std::ostringstream prefix;
  prefix << ros::package::getPath("kinect") << "/data/" << kinect_serial_number;

  // read the bag for depth image
  std::ostringstream depth_bag_name;
  depth_bag_name << prefix.str() << "_depth.bag";
  if (!file_exists(depth_bag_name.str())) {
    printf("The depth camera info bag '%s' does not exist!\n",
           depth_bag_name.str().c_str());
    return false;
  }
  rosbag::Bag depth_bag(depth_bag_name.str());
  rosbag::View depth_view
      (depth_bag, rosbag::TopicQuery("/kinect_only_camera/depth_registered/camera_info"));
  bool depth_success = false;
  // BOOST_FOREACH(rosbag::MessageInstance const m, depth_view) {
  //  sensor_msgs::CameraInfo::ConstPtr i =
  //      m.instantiate<sensor_msgs::CameraInfo>();
  sensor_msgs::CameraInfo::ConstPtr i =
      depth_view.begin()->instantiate<sensor_msgs::CameraInfo>();
  if (i != NULL) {
    printf("depth camera info succesfully read from '%s'\n",
           depth_bag_name.str().c_str());
    depth_camera_info = *i;
    depth_success = true;
    // break;
  }
  // } end BOOST_FOREACH
  depth_bag.close();
  if (!depth_success)
    return false;

  // read the bag for rgb image
  std::ostringstream rgb_bag_name;
  rgb_bag_name << prefix.str() << "_rgb.bag";
  if (!file_exists(rgb_bag_name.str())) {
    printf("The rgb camera info bag '%s' does not exist!\n",
           rgb_bag_name.str().c_str());
    return false;
  }
  rosbag::Bag rgb_bag(rgb_bag_name.str());
  rosbag::View rgb_view
      (rgb_bag, rosbag::TopicQuery("/kinect_only_camera/rgb/camera_info"));
  bool rgb_success = false;
  //  BOOST_FOREACH(rosbag::MessageInstance const m, rgb_view) {
  //    sensor_msgs::CameraInfo::ConstPtr i = m.instantiate<sensor_msgs::CameraInfo>();
  i = rgb_view.begin()->instantiate<sensor_msgs::CameraInfo>();
  if (i != NULL) {
    printf("rgb camera info succesfully read from '%s'\n",
           rgb_bag_name.str().c_str());
    rgb_camera_info = *i;
    rgb_success = true;
    // break;
  }
  // } end BOOST_FOREACH
  rgb_bag.close();
  if (!rgb_success)
    return false;

  // if we are here, success
  printf("read_camera_info_bag_files(): reading from '%s' and '%s' was a success.\n",
         depth_bag_name.str().c_str(), rgb_bag_name.str().c_str());
  return true;
} // end read_camera_info_bag_files()

} // end namespace vision_utils

#endif // READ_CAMERA_INFO_BAG_FILES_H
