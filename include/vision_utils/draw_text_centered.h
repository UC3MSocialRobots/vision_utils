/*!
  \file        draw_text_centered.h
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

#ifndef DRAW_TEXT_CENTERED_H
#define DRAW_TEXT_CENTERED_H
// std includes
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace vision_utils {

/*!
 \param img
    the image where the text will be put
 \param text
    The text to be put
 \param org
    Where to put the text in img
 \param fontFace, fontScale, color, thickness, linetype
    \see cv::putText()
*/
inline void draw_text_centered(cv::Mat& img, const std::string& text,
                               const cv::Point & org,  const int & fontFace,
                               const double & fontScale, const cv::Scalar & color,
                               const int thickness=1, const int linetype=8) {
  int baseline = 0;
  cv::Size txt_size =
      cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  cv::putText(img, text,
              cv::Point(org.x - txt_size.width / 2,
                        org.y + txt_size.height / 2),
              fontFace, fontScale, color, thickness, linetype);
} // end draw_text_centered();

} // end namespace vision_utils

#endif // DRAW_TEXT_CENTERED_H
