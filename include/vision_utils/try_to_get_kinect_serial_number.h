/*!
  \file        try_to_get_kinect_serial_number.h
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

#ifndef TRY_TO_GET_KINECT_SERIAL_NUMBER_H
#define TRY_TO_GET_KINECT_SERIAL_NUMBER_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

/*!
 * try to obtain the serial nb of the kinect from the parameter server
 * \param nh_public
 *   a node handle to access the param server
 * \param kinect_serial_number
 *   where to store the result.
 * \return
 *   true if success.
 *   In that case, \a kinect_serial_number contains the serial number of the Kinect.
 */
bool try_to_get_kinect_serial_number(const ros::NodeHandle & nh_public,
                                     std::string & kinect_serial_number) {
  kinect_serial_number = "";
  std::string name_resolved = nh_public.resolveName("kinect_serial_number");
  bool success = nh_public.getParam(name_resolved, kinect_serial_number);
  if (!success) {
    printf("Could not get kinect_serial_number param on '%s'!\n",
           name_resolved.c_str());
  }
  return success;
}

} // end namespace vision_utils

#endif // TRY_TO_GET_KINECT_SERIAL_NUMBER_H
