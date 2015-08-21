/*!
  \file        draw_ppl_on_image.h
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

#ifndef DRAW_PPL_ON_IMAGE_H
#define DRAW_PPL_ON_IMAGE_H

// msg
#include <people_msgs/PeoplePoseList.h>
#include "vision_utils/drawing_utils.h"
#include "vision_utils/utils/pt_utils.h"

namespace ppl_utils {
/*!
 * \brief draw_ppl_on_image
 * \param list
 *    \warning should be already in the frame of the image
 *    (\see kinect_openni_utils::world2pixel() ).
 *    The PPL can easily be converted, for instance
 *    \see ppl_utils::convert_ppl_tf()
 * \param rgb_camera_model
 *    The model of the camera that corresponds to \arg out.
 * \param out
 * \param color
 * \param thickness
 */
void draw_ppl_on_image(const people_msgs::PeoplePoseList & list,
                       const image_geometry::PinholeCameraModel rgb_camera_model,
                       cv::Mat & out,
                       cv::Scalar color = CV_RGB(0, 255, 0),
                       int thickness = 2) {
  unsigned int npeople = list.poses.size();
  // ROS_WARN("draw_ppl_on_image(%i people)", npeople);
  for (unsigned int people_idx = 0; people_idx < npeople; ++people_idx) {
    // get face position
    cv::Point3d face_pt3d;
    pt_utils::copy3(list.poses[people_idx].head_pose.position, face_pt3d);
    cv::Point2i face_pt2d
        = kinect_openni_utils::world2pixel<cv::Point2i>(face_pt3d, rgb_camera_model);
    // printf("face_pt3d:%s\n", pt_utils::print_point(face_pt3d).c_str());
    // printf("face_pt2d:%s\n", geometry_utils::printP2(face_pt2d).c_str());
    // draw a circle for the head
    cv::circle(out, face_pt2d, 15, color, thickness);

    // get feet 2d position draw a line from feet to head
    cv::Point3d feet_pt3d(face_pt3d.x, face_pt3d.y + 1.5, face_pt3d.z);
    cv::Point2i feet_pt2d
        = kinect_openni_utils::world2pixel<cv::Point2i>(feet_pt3d, rgb_camera_model);
    cv::line(out, face_pt2d, feet_pt2d, color, thickness);

    // put text
    std::string text = list.poses[people_idx].person_name;
    cv::Point2i text_pt2d(face_pt2d.x, face_pt2d.y - 50);
    //cv::putText(out, text, text_pt2d, CV_FONT_HERSHEY_PLAIN, 2, color);
    image_utils::draw_text_centered(out, text, text_pt2d,
                                    CV_FONT_HERSHEY_PLAIN, 2, color, thickness);
  } // end loop people_idx
} // end draw_ppl_on_image();
} // end namespace ppl_utils

#endif // DRAW_PPL_ON_IMAGE_H
