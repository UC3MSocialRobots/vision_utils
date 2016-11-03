/*!
  \file        read_camera_info_binary_files.h
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

#ifndef READ_CAMERA_INFO_BINARY_FILES_H
#define READ_CAMERA_INFO_BINARY_FILES_H
// std includes
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include <ros/package.h>
#include "vision_utils/ros_object_from_file.h"
#include "vision_utils/file_exists.h"

namespace vision_utils {

/*!
 * Read the camera info objects from ROS binary files.
 * These binary files can be obtained for instance using
 * ros_object_to_file().
 * \param kinect_serial_number
 *   The serial number of the Kinect, for instance KINECT_SERIAL_ARNAUD()
 * \param depth_camera_info, rgb_camera_info
 *    Output: the depth and rgb camera info, filled by reading the bags.
 * \return true if success
 */
bool read_camera_info_binary_files(const std::string & kinect_serial_number,
                                   sensor_msgs::CameraInfo & depth_camera_info,
                                   sensor_msgs::CameraInfo & rgb_camera_info) {
  // load binary files
  std::ostringstream prefix;
  prefix << ros::package::getPath("kinect") << "/data/"  << kinect_serial_number;

  // read the binary for depth image
  std::ostringstream depth_binary_name;
  depth_binary_name << prefix.str() << "_depth.dat";
  if (!file_exists(depth_binary_name.str())) {
    printf("The depth camera info binary '%s' does not exist!\n",
           depth_binary_name.str().c_str());
    return false;
  }
  ros_object_from_file(depth_binary_name.str(), depth_camera_info);

  // read the binary for rgb image
  std::ostringstream rgb_binary_name;
  rgb_binary_name << prefix.str() << "_rgb.dat";
  if (!file_exists(rgb_binary_name.str())) {
    printf("The rgb camera info binary '%s' does not exist!\n",
           rgb_binary_name.str().c_str());
    return false;
  }
  ros_object_from_file(rgb_binary_name.str(), rgb_camera_info);

  // if we are here, success
  //  printf("read_camera_info_binary_files(): reading from '%s' and '%s' was a success.\n",
  //            depth_binary_name.str().c_str(), rgb_binary_name.str().c_str());
  return true;
} // end read_camera_info_binary_files()

} // end namespace vision_utils

#endif // READ_CAMERA_INFO_BINARY_FILES_H
