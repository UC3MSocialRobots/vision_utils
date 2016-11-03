/*!
  \file        paste_images_gallery.h
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

#ifndef PASTE_IMAGES_GALLERY_H
#define PASTE_IMAGES_GALLERY_H
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>

namespace vision_utils {

/*!
 * Paste a bunch of images OF THE SAME SIZE into a big collage,
 * with configurable number of images per row.
 * \param in
 *    the input images, of the same size
 * \param out
 *    the collage image.
 * \param gallerycols
 *    the desired number of images per row
 * \param background_color
 *    the color of the background, visible if the last row is not complete for instance
 * \param draw_borders
 *    true for drawing a rectangular border around each pic
 * \param border_color
 *    the color for the border. Only used if \arg draw_borders = true.
 */
template<class T>
void paste_images_gallery(const std::vector<cv::Mat_<T> > & in,
                          cv::Mat_<T> & out,
                          int gallerycols, T background_color,
                          bool draw_borders = false, cv::Scalar border_color = CV_RGB(0,0,0)) {
  if (in.size() == 0) {
    out.create(0, 0);
    return;
  }
  int cols1 = in[0].cols, rows1 = in[0].rows, nimgs = in.size();
  // prepare output
  int galleryrows = std::ceil(1. * nimgs / gallerycols);
  out.create(rows1 * galleryrows, cols1 * gallerycols);
  //printf("nimgs:%i, gallerycols:%i, galleryrows:%i\n", nimgs, gallerycols, galleryrows);
  out.setTo(background_color);
  // paste images
  for (int img_idx = 0; img_idx < nimgs; ++img_idx) {
    int galleryx = img_idx % gallerycols, galleryy = img_idx / gallerycols;
    cv::Rect roi(galleryx * cols1, galleryy * rows1, cols1, rows1);
    //printf("### out:%ix%i, roi %i:'%s'\n", out.cols, out.rows, img_idx, print_rect(roi).c_str());
    if (cols1 != in[img_idx].cols || rows1 != in[img_idx].rows) {
      printf("Image %i of size (%ix%i), different from (%ix%i), skipping it.\n",
             img_idx, in[img_idx].cols, in[img_idx].rows, cols1, rows1);
      cv::line(out, roi.tl(), roi.br(), border_color, 2);
      cv::line(out, roi.br(), roi.tl(), border_color, 2);
    }
    else
      in[img_idx].copyTo( out(roi) );
    if (draw_borders)
      cv::rectangle(out, roi, border_color, 1);
  } // end loop img_idx
} // end paste_pics<_T>

} // end namespace vision_utils

#endif // PASTE_IMAGES_GALLERY_H
