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
#include <people_msgs/PeoplePoseList.h>
// ROS
#include <tf/transform_listener.h>
#include <tf/exceptions.h>

namespace ppl_utils {
/*!
 * Convert a people_msgs::PeoplePose from one frame to another.
 * \param pose
 *   [IN+OUT] The pose to be converted.
 * \param target_frame
 *    The frame we want to convert into
 * \param transform_listener
 *    A listener. It contains all the transforms by subscribing to /tf.
 * \return
 *    true if success
 */
bool convert_people_pose_tf(people_msgs::PeoplePose & pose,
                            const std::string & target_frame,
                            const tf::TransformListener & transform_listener) {
  // do nothing if same frame
  if (pose.header.frame_id == target_frame)
    return true;

  // convert geometry_msgs::Pose -> geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped pin, pout;
  pin.header = pose.header;
  pin.pose = pose.head_pose;
  pout.header.stamp = pin.header.stamp;
  pout.header.frame_id = target_frame;
  try {
    // make transform
    transform_listener.transformPose
        (target_frame, pose.header.stamp, pin, target_frame, pout);
  } catch (tf::TransformException & e){
    printf("convert_people_pose_tf(): error in TF conversion '%s'->'%s': '%s'\n",
           pose.header.frame_id.c_str(), target_frame.c_str(), e.what());
    return false;
  }
  pose.head_pose = pout.pose;
  pose.header.frame_id = target_frame;
  return true;
} // end convert_people_pose_tf()

//////////////////////////////////////////////////////////////////////////////

/*!
 * Convert a list of people_msgs::PeoplePose from one frame to another.
 * \param list
 *   The list of poses to convert.
 * \param target_frame
 *    The frame we want to convert into
 * \param transform_listener
 *    A listener. It contains all the transforms by subscribing to /tf.
 * \return
 *    true if success
 */
bool convert_ppl_tf(people_msgs::PeoplePoseList & list,
                    const std::string & target_frame,
                    tf::TransformListener & transform_listener) {
  for (unsigned int pose_idx = 0; pose_idx < list.poses.size(); ++pose_idx) {
    bool success = convert_people_pose_tf
                   (list.poses[pose_idx], target_frame, transform_listener);
    if (!success)
      return false;
  } // end loop pose_idx
  // store the new frame for list msg
  list.header.frame_id = target_frame;
  return true;
} // end convert_ppl_tf()

////////////////////////////////////////////////////////////////////////////////

//! set a new header for both the PPL and its sons
inline void set_ppl_header(people_msgs::PeoplePoseList & ppl,
                           const std::string & frame,
                           const ros::Time & stamp) {
  ppl.header.frame_id = frame;
  ppl.header.stamp = stamp;
  unsigned int npps = ppl.poses.size();
  for (unsigned int i = 0; i < npps; ++i)
    ppl.poses[i].header = ppl.header;
}

} // end namespace ppl_utils
#endif // PPL_TF_UTILS_H
