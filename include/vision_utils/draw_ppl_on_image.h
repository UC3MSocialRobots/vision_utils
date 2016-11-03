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
#include <people_msgs/People.h>



namespace vision_utils {
/*!
 * \brief draw_ppl_on_image
 * \param list
 *    \warning should be already in the frame of the image
 *    (\see world2pixel() ).
 *    The PPL can easily be converted, for instance
 *    \see convert_ppl_tf()
 * \param rgb_camera_model
 *    The model of the camera that corresponds to \arg out.
 * \param out
 * \param color
 * \param thickness
 */
void draw_ppl_on_image(const people_msgs::People & list,
                       const image_geometry::PinholeCameraModel rgb_camera_model,
                       cv::Mat & out,
                       cv::Scalar color = CV_RGB(0, 255, 0),
                       int thickness = 2) {
  unsigned int npeople = list.poses.size();
  //printf("draw_ppl_on_image(%i people)", npeople);
  for (unsigned int people_idx = 0; people_idx < npeople; ++people_idx) {
    // get face position
    cv::Point3d face_pt3d;
    copy3(list.poses[people_idx].position.position, face_pt3d);
    cv::Point2i face_pt2d
        = world2pixel<cv::Point2i>(face_pt3d, rgb_camera_model);
    //printf("face_pt3d:%s\n", print_point(face_pt3d).c_str());
    //printf("face_pt2d:%s\n", printP2(face_pt2d).c_str());
    // draw a circle for the head
    cv::circle(out, face_pt2d, 15, color, thickness);

    // get feet 2d position draw a line from feet to head
    cv::Point3d feet_pt3d(face_pt3d.x, face_pt3d.y + 1.5, face_pt3d.z);
    cv::Point2i feet_pt2d
        = world2pixel<cv::Point2i>(feet_pt3d, rgb_camera_model);
    cv::line(out, face_pt2d, feet_pt2d, color, thickness);

    // put text
    std::string text = list.poses[people_idx].name;
    cv::Point2i text_pt2d(face_pt2d.x, face_pt2d.y - 50);
    //cv::putText(out, text, text_pt2d, CV_FONT_HERSHEY_PLAIN, 2, color);
    draw_text_centered(out, text, text_pt2d,
                                    CV_FONT_HERSHEY_PLAIN, 2, color, thickness);
  } // end loop people_idx
} // end draw_ppl_on_image();
} // end namespace vision_utils

#endif // DRAW_PPL_ON_IMAGE_H
