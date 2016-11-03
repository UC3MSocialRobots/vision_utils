/*!
  \file        putTextBackground.h
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

#ifndef PUTTEXTBACKGROUND_H
#define PUTTEXTBACKGROUND_H
// std includes
#include <opencv2/core/core.hpp>
#include <string>

namespace vision_utils {

/*!
 * Put a text into an image with a given background color
 * \param img
 *    the image where the text will be put
 * \param text
 *    The text to be put
 * \param org
 *    Where to put the text in img
 * \param font_color
 *    the color of the text characters
 * \param bg_color
 *    the color behind the characters
 * \param padding
 *    the size in pixels of the margin between the bounding box of the text
 *    and the rectangle filled with background color
 * \param fontFace, fontScale, thickness, linetype, bottomLeftOrigin
    \see cv::putText()
 */
inline cv::Rect putTextBackground(cv::Mat& img, const std::string& text,
                                  cv::Point org, int fontFace, double fontScale,
                                  cv::Scalar font_color, cv::Scalar bg_color,
                                  int padding = 1,
                                  int thickness=1, int linetype=8,
                                  bool bottomLeftOrigin=false ) {
  // find size of text
  int baseline = 0;
  cv::Size txt_size =
      cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  // paint where text will be with given color
  cv::Rect text_roi(org.x - padding,
                    org.y - txt_size.height - padding,
                    txt_size.width + 2 * padding,
                    txt_size.height + 2 * padding);
  cv::rectangle(img, text_roi, bg_color, -1);
  // put final text
  cv::putText(img, text, org,
              fontFace, fontScale, font_color, thickness, linetype, bottomLeftOrigin);
  return text_roi;
}

} // end namespace vision_utils

#endif // PUTTEXTBACKGROUND_H
