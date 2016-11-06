/*!
  \file        read_camera_model_files.h
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

#ifndef READ_CAMERA_MODEL_FILES_H
#define READ_CAMERA_MODEL_FILES_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>
#include "image_geometry/pinhole_camera_model.h"
#include "vision_utils/read_camera_info_binary_files.h"
#include "vision_utils/read_camera_info_bag_files.h"

namespace vision_utils {

/*!
 * \brief read_camera_model_files
 * \param kinect_serial_number
 * \param depth_camera_model, rgb_camera_model
 *    Output: the depth and rgb camera model, filled by reading the bags.
 * \param is_binary_file
 *    true if the camera info are stored into binary files
 *    (\see \a read_camera_info_binary_files()),
 *    false if they are into bag files
 *    (\see \a read_camera_info_bag_files())
 * \return
 *    true if success
 */
bool read_camera_model_files(const std::string & kinect_serial_number,
                             image_geometry::PinholeCameraModel & depth_camera_model,
                             image_geometry::PinholeCameraModel & rgb_camera_model,
                             bool is_binary_file = true) {
  // read camera info
  sensor_msgs::CameraInfo depth_camera_info, rgb_camera_info;
  bool success;
  if (is_binary_file)
    success = read_camera_info_binary_files
              (kinect_serial_number, depth_camera_info, rgb_camera_info);
  else
    success = read_camera_info_bag_files
              (kinect_serial_number, depth_camera_info, rgb_camera_info);
  if (!success)
    return false;
  // convert depth info to model
  success = depth_camera_model.fromCameraInfo(depth_camera_info);
  if (!success) {
    printf("Error while converting depth_camera_info -> depth_camera_model\n");
    return false;
  }
  // convert rgb info to model
  success = rgb_camera_model.fromCameraInfo(rgb_camera_info);
  if (!success) {
    printf("Error while converting rgb_camera_info -> rgb_camera_model\n");
    return false;
  }

  //  printf("Succesfully converted depth_camera_info -> depth_camera_model "
  //         "and rgb_camera_info -> rgb_camera_model\n");
  return true;
} // end read_camera_model_files()

} // end namespace vision_utils

#endif // READ_CAMERA_MODEL_FILES_H
