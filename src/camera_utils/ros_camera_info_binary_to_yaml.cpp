/*!
  \file        ros_camera_info_binary_to_yaml.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/11/21

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

Converts a ROS .dat CameraInfo file into a .yaml file.

\section Parameters
  - \b "~filename"
        [std::string] (default: "camera_info.dat")
        The filename for the input binary CameraInfo.
 */
// ROS
#include <sensor_msgs/CameraInfo.h>
// AD
#include <string/string_utils_ros.h>
#include <string/filename_handling.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_camera_info_binary_to_yaml");
  ros::NodeHandle nh_public, nh_private("~");
  std::string filename_in = "camera_info.dat";
  // get params
  nh_private.param("filename", filename_in, filename_in);
  std::string filename_out =
  StringUtils::change_filename_extension(filename_in, ".yaml");
  printf("ros_camera_info_binary_to_yaml(): Trying to convert '%s'->'%s'.\n",
         filename_in.c_str(), filename_out.c_str());

  // parse camera info .dat file and create publisher
  sensor_msgs::CameraInfo camera_info;
  if (!system_utils::file_exists(filename_in)) {
    printf("ros_camera_info_binary_to_yaml(): input file '%s' does not exist.\n",
           filename_in.c_str());
    exit(-1);
  }
  string_utils_ros::ros_object_from_file(filename_in, camera_info);
  std::string camera_info_topic = "camera_info";
  ros::Publisher pub = nh_public.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 1, true);
  pub.publish(camera_info);
  ros::spinOnce();

  // wait for camera info topic
  printf("ros_camera_info_binary_to_yaml(): waiting for CameraInfo...\n");
  std::ostringstream order;
  order << "rostopic echo -n 1 " << camera_info_topic << " >> " << filename_out;
  system_utils::exec_system(order.str().c_str());
  printf("ros_camera_info_binary_to_yaml(): succesfully wrote CameraInfo to '%s'\n",
         filename_out.c_str());
  return 0;
}

