/*!
  \file        get_kinect_serial_number_and_read_camera_model_files_if_needed.h
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

#ifndef GET_KINECT_SERIAL_NUMBER_AND_READ_CAMERA_MODEL_FILES_IF_NEEDED_H
#define GET_KINECT_SERIAL_NUMBER_AND_READ_CAMERA_MODEL_FILES_IF_NEEDED_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>
#include "vision_utils/try_to_get_kinect_serial_number.h"
#include "vision_utils/read_camera_model_files.h"

namespace vision_utils {

/*!
 * If the camera models are already initialized, do nothing.
 * Otherwise, try to
 *  1) get the serial number of the kinect from the parameter server
 *  2) read the corresponding binary files for both depth and rgb camera infos.
 * \param nh_public
 *   The node handle that will enable to access the
 *    serial number of the kinect
 *    (\see \a try_to_get_kinect_serial_number())
 * \param depth_camera_model, rgb_camera_model
 *    Output: the depth and rgb camera model, filled by reading the bags.
 * \return
 *    true if success (models already initialized or succesful reading)
 */
bool get_kinect_serial_number_and_read_camera_model_files_if_needed
(const ros::NodeHandle & nh_public,
 image_geometry::PinholeCameraModel & rgb_camera_model,
 image_geometry::PinholeCameraModel & depth_camera_model
 )
{
  // electric:  v1.6.5
#if ( ROS_VERSION_MINIMUM(1, 7, 0) ) // fuerte code
  if (rgb_camera_model.initialized() && depth_camera_model.initialized())
#else // electric code
  if (rgb_camera_model.fx() > 0 && depth_camera_model.fx() > 0)
#endif // USE
    return true;
  std::string kinect_serial_number;
  bool success = try_to_get_kinect_serial_number
                 (nh_public, kinect_serial_number);
  if (!success) {
    printf("Unable to get the serial number of the Kinect, returning!");
    return false;
  }
  //printf("Serial number of the Kinect:'%s'", kinect_serial_number.c_str());
  return read_camera_model_files
      (kinect_serial_number, depth_camera_model, rgb_camera_model);
} // end get_kinect_serial_number_and_read_camera_model_files_if_needed()

} // end namespace vision_utils

#endif // GET_KINECT_SERIAL_NUMBER_AND_READ_CAMERA_MODEL_FILES_IF_NEEDED_H
