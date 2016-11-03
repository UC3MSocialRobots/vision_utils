/*!
  \file        make_opencv_interface.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/5/15

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHdst ANY WARRANTY; withdst even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file

 */

#ifndef MAKE_OPENCV_INTERFACE_H
#define MAKE_OPENCV_INTERFACE_H

#include <opencv2/highgui/highgui.hpp>
// vision


namespace vision_utils {

enum InterfacePosition {TOP = 1, BOTTOM = 2, LEFT = 3, RIGHT = 4};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline cv::Rect button_bounding_box(const int dst_cols, int dst_rows,
                                    const InterfacePosition interface_pos,
                                    const int button_size_pixels,
                                    const int nbuttons, const int button_idx) {
  int interface_x = (interface_pos == RIGHT ? dst_cols - button_size_pixels : 0);
  int interface_y = (interface_pos == BOTTOM ? dst_rows - button_size_pixels : 0);

  if (interface_pos == LEFT || interface_pos == RIGHT)
    return cv::Rect(interface_x,
                    interface_y + dst_rows * button_idx / nbuttons,
                    button_size_pixels,
                    dst_rows / nbuttons);
  else
    return cv::Rect(interface_x + dst_cols * button_idx / nbuttons,
                    interface_y,
                    dst_cols / nbuttons,
                    button_size_pixels);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void make_opencv_interface(const cv::Mat3b & src, cv::Mat3b & dst,
                           const std::vector<std::string> & button_names,
                           const std::vector<cv::Scalar> & button_colors,
                           const InterfacePosition interface_pos,
                           const int button_size_pixels) {
  // resize dst_cols to good dimensions
  int dst_cols = src.cols +
      (interface_pos == LEFT || interface_pos == RIGHT ? button_size_pixels : 0);
  int dst_rows = src.rows +
      (interface_pos == TOP || interface_pos == BOTTOM ? button_size_pixels : 0);
  dst.create(dst_rows, dst_cols);

  // paste src into dst
  paste_img(src, dst,
                         (interface_pos == LEFT ? button_size_pixels : 0),
                         (interface_pos == TOP ? button_size_pixels : 0));

  // paint the buttons
  unsigned int nbuttons = button_names.size();
  if (nbuttons != button_colors.size()) {
    printf("nbuttons:%i != button_colors.size():%i\n",
           nbuttons, button_colors.size());
    return;
  }
  for (unsigned int button_idx = 0; button_idx < nbuttons; ++button_idx) {
    // draw rectangle
    cv::Rect button_bbox = button_bounding_box
        (dst.cols, dst.rows, interface_pos, button_size_pixels, nbuttons, button_idx);
    cv::rectangle(dst, button_bbox, button_colors[button_idx], -1);
    // draw the text
    draw_text_centered
        (dst, button_names[button_idx],
         cv::Point(button_bbox.x + button_bbox.width / 2,
                   button_bbox.y + button_bbox.height / 2),
         CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0), 2);
  } // end loop button_idx
} // end make_opencv_interface()

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*!
 \return int
        -1 if not a button or the index of the button (starting at 0)
*/
int is_pixel_a_button(cv::Mat3b & dst,
                      const InterfacePosition interface_pos,
                      const unsigned int nbuttons,
                      const int button_size_pixels,
                      const int x, const int y) {
  for (unsigned int button_idx = 0; button_idx < nbuttons; ++button_idx) {
    // draw rectangle
    cv::Rect button_bbox = button_bounding_box
        (dst.cols, dst.rows, interface_pos, button_size_pixels, nbuttons, button_idx);
    if (button_bbox.contains(cv::Point(x, y)))
      return button_idx;
  } // end loop button_idx
  return -1;
} // end make_opencv_interface()

} // end namespace vision_utils
#endif // MAKE_OPENCV_INTERFACE_H
