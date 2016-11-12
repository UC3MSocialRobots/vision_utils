/*!
  \file        ppl_tf_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/21

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
 */

#ifndef PPL_TF_UTILS_H
#define PPL_TF_UTILS_H

// msg
#include <people_msgs/People.h>
// ROS
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

namespace vision_utils {
/*!
 * Convert a people_msgs::Person from one frame to another.
 * \param pose
 *   [IN+OUT] The pose to be converted.
 * \param target_frame
 *    The frame we want to convert into
 * \param transform_listener
 *    A listener. It contains all the transforms by subscribing to /tf.
 * \return
 *    true if success
 */
bool convert_people_pose_tf(people_msgs::Person & pose,
                            const std_msgs::Header & pose_header,
                            const std::string & target_frame,
                            const tf::TransformListener & transform_listener) {
  // do nothing if same frame
  if (pose_header.frame_id == target_frame)
    return true;

  // convert geometry_msgs::Pose -> geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped pin, pout;
  pin.header = pose_header;
  pin.pose.position = pose.position;
  pin.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  pout.header.stamp = pin.header.stamp;
  pout.header.frame_id = target_frame;
  try {
    // make transform
    transform_listener.transformPose
        (target_frame, pose_header.stamp, pin, target_frame, pout);
  } catch (tf::TransformException & e){
    printf("convert_people_pose_tf(): error in TF conversion '%s'->'%s': '%s'\n",
           pose_header.frame_id.c_str(), target_frame.c_str(), e.what());
    return false;
  }
  pose.position = pout.pose.position;
  return true;
} // end convert_people_pose_tf()

//////////////////////////////////////////////////////////////////////////////

/*!
 * Convert a list of people_msgs::Person from one frame to another.
 * \param list
 *   The list of poses to convert.
 * \param target_frame
 *    The frame we want to convert into
 * \param transform_listener
 *    A listener. It contains all the transforms by subscribing to /tf.
 * \return
 *    true if success
 */
bool convert_ppl_tf(people_msgs::People & list,
                    const std::string & target_frame,
                    tf::TransformListener & transform_listener) {
  for (unsigned int pose_idx = 0; pose_idx < list.people.size(); ++pose_idx) {
    bool success = convert_people_pose_tf
                   (list.people[pose_idx], list.header,
                    target_frame, transform_listener);
    if (!success)
      return false;
  } // end loop pose_idx
  // store the new frame for list msg
  list.header.frame_id = target_frame;
  return true;
} // end convert_ppl_tf()

////////////////////////////////////////////////////////////////////////////////

//! set a new header for both the PPL and its sons
inline void set_ppl_header(people_msgs::People & ppl,
                           const std::string & frame,
                           const ros::Time & stamp) {
  ppl.header.frame_id = frame;
  ppl.header.stamp = stamp;
}

} // end namespace vision_utils
#endif // PPL_TF_UTILS_H
