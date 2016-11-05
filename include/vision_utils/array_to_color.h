/*!
  \file        array_to_color.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/12

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

A function for converting a matrix into an array of colors.
The colormaps can be parametrized.
Optionnal header drawing.

 */

#ifndef ARRAY_TO_COLOR_H
#define ARRAY_TO_COLOR_H

#include <opencv2/imgproc/imgproc.hpp>
#include <vision_utils/rect_center.h>
#include <vision_utils/draw_text_centered.h>
#include "vision_utils/colormaps.h"
#include "vision_utils/titlemaps.h"

namespace vision_utils {

/*!
 * Compute the ROI of a given cell in the array.
 * \param col, row
 *    coordinates of the cell.
 *    if draw_headers, col=-1 correspond to a column header (same thing for a row).
 * \param width1, height1
 *    dimensions in pixels of one cell of the array
 * \param draw_headers
 *    true if the headers for rows and columns must be drawn.
 * \return
 *   the ROI of (col, row)
 */
cv::Rect array_to_color_cell_roi(const unsigned int col, const unsigned int row,
                                 const unsigned int width1, const unsigned int height1,
                                 bool draw_headers = true) {
  if (draw_headers) // leave space for the headers
    return cv::Rect((col+1) * width1, (row+1) * height1, width1, height1);
  else
    return cv::Rect(col * width1, row * height1, width1, height1);
} // end array_to_color_cell_roi()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Convert an array of doubles into a visualization image.
 * The color map can be customized.
 * \param data
 *    The input data, must be a 2-dimension array that can be accessed with [][]
 * \param ncols, nrows
 *    The dimensions of "data"
 * \param out
 *    The output image
 * \param width1, height1
 *    The wanted dimnensions of a cell, in pixels.
 * \param draw_edges
 *    true for drawing the edges of each cell.
 * \param draw_headers
 *    true for drawing the headers of each column and row.
 *    In that case, the functions column_titlemap() and row_titlemap() are used
 *    to determine the name of these rows and columns.
 * \param value_colormap
 * \param column_titlemap, row_titlemap
 *    if "draw_headers" true, functions for converting col (or row) index
 *    into a name, i.e. a string.
 *    By default, columns="A", "B", "C"... and rows="a", "b", "c",...
 */
template<class _T>
void array_to_color(const _T & data,
                    const unsigned int nrows, const unsigned int ncols,
                    cv::Mat & out,
                    const unsigned int width1, const unsigned int height1,
                    bool draw_edges = true,
                    bool draw_headers = true,
                    cv::Scalar (*value_colormap)(const double & )
                    = ratio2grey_inv,
                    std::string (*column_titlemap)(const unsigned int)
                    = &int_to_uppercase_letter,
                    std::string (*row_titlemap)(const unsigned int)
                    = &int_to_lowercase_letter,
                    const std::vector<std::string>* rows_custom_headers = NULL,
                    const std::vector<std::string>* cols_custom_headers = NULL
    ) {
  // fool-proof sizes
  if (nrows <= 0 || ncols <= 0) {
    out = cv::Mat3b(1, 1);
    return;
  }
  // resize image if needed
  int needed_rows = nrows * height1, needed_cols = ncols * width1;
  if (draw_headers) {
    needed_rows += height1;
    needed_cols += width1;
  }
  //  printf("nrows:%i, ncols:%i, height1:%i, width1:%i, needed_rows:%i, needed_cols:%i\n",
  //         nrows, ncols, height1, width1, needed_rows, needed_cols);
  if (out.cols < needed_cols || out.rows < needed_rows)
    out.create(needed_rows, needed_cols, CV_8UC3);

  // set to white
  out.setTo(255);

  // draw the value for each cell
  for (unsigned int row = 0; row < nrows; ++row) {
    for (unsigned int col = 0; col < ncols; ++col) {
      cv::Rect cell_roi = array_to_color_cell_roi(col, row, width1, height1, draw_headers);
      //printf("data[%i][%i]:%g\n", row, col, data[row][col]);
      out(cell_roi) = value_colormap(data[row][col]);
    } // end loop col
  } // end loop row

  // draw edges
  if (draw_edges) {
    // horizontal lines
    unsigned int nlines = (draw_headers ? nrows + 2 : nrows + 1);
    for (unsigned int row = 1; row < nlines; ++row) {
      // int x_begin = (draw_headers && (row <= 1 || row == nlines - 1) ? width1 : 0);
      int x_begin = (draw_headers ? width1 : 0);
      int y = row * height1 + (row == nlines - 1 ? -1 : 0);
      cv::line(out, cv::Point(x_begin, y), cv::Point(out.cols, y), CV_RGB(0, 0, 0), 1);
    } // end loop row

    // vertical lines
    nlines = (draw_headers ? ncols + 2 : ncols + 1);
    for (unsigned int col = 1; col < nlines; ++col) {
      //int y_begin = (draw_headers && (col <= 1 || col == nlines -1) ? width1 : 0);
      int y_begin = (draw_headers ? width1 : 0);
      int x = col * height1 + (col == nlines - 1 ? -1 : 0);
      cv::line(out, cv::Point(x, y_begin), cv::Point(x, out.rows), CV_RGB(0, 0, 0), 1);
    } // end loop col
  } // end if (draw_edges)

  // draw captions
  if (draw_headers) {
    bool use_custom_headers = (rows_custom_headers != NULL && rows_custom_headers->size() == nrows
                                               && cols_custom_headers != NULL && cols_custom_headers->size() == ncols);
    // horizontal labels
    for (unsigned int col = 0; col < ncols; ++col) {
      std::string header = (use_custom_headers ? (*cols_custom_headers)[col] : column_titlemap(col));
      draw_text_centered
          (out, header, rect_center<cv::Rect, cv::Point>
           (array_to_color_cell_roi(col, -1, width1, height1, true)),
           CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 0, 0));
    }

    // vertical labels
    for (unsigned int row = 0; row < nrows; ++row) {
      std::string header = (use_custom_headers ? (*rows_custom_headers)[row] : row_titlemap(row));
      draw_text_centered
          (out, header, rect_center<cv::Rect, cv::Point>
           (array_to_color_cell_roi(-1, row, width1, height1, true)),
           CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 0, 0));
    }
  } // end if (draw_headers)
} // end array_to_color()

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief colormap_to_caption_image
 * \param out
 *    where to make the drawing
 * \param width, height
 *    the wanted dimensions for out, in pixels
 * \param min, max
 *    the bounds for the caption, usually 0 --> 1
 * \param color_step
 *    the increment for the color cells
 * \param caption_step
 *    the increment for the caption (text) cells
 * \param draw_edges
 *    true for drawing a black rectangle around each color cell
 * \param padding
 *    the wanted padding (white space) in pixels around the caption
 */
bool colormap_to_caption_image(cv::Mat & out,
                               const int width, const int height,
                               cv::Scalar (*value_colormap)(const double &),
                               const double & min = 0., const double & max = 1.,
                               const double & color_step = .05, const double & caption_step = .2,
                               bool draw_edges = true,
                               const int padding = 20) {
  int width_inner = width - 2 * padding, height_inner = height - 2 * padding;
  if (width_inner < 0 || height_inner < 0) {
    printf("colormap_to_caption_image():w:%i, h:%i too small if padding=%i\n",
           width, height, padding);
    return false;
  }
  if (out.cols < width || out.rows < height)
    out.create(height, width, CV_8UC3);
  // clear image
  out.setTo(255);
  cv::Mat out_roi = out(cv::Rect(padding, padding, width_inner, height_inner));

  // draw color cells
  double value = min;
  int x_color_cell_min = width_inner / 2, x_color_cell_max = width_inner - 1;
  while (value < max - 1E-2) {
    //printf("value:%g\n", value);
    int y_lower = height_inner * (1.f - (value             - min) / max),
        y_upper = height_inner * (1.f - (value +color_step - min) / max);
    cv::rectangle(out_roi,
                  cv::Point(x_color_cell_min,  y_upper),
                  cv::Point(x_color_cell_max, y_lower),
                  value_colormap(value), -1);
    if (draw_edges)
      cv::rectangle(out, cv::Point(padding + x_color_cell_min,  padding + y_upper),
                    cv::Point(padding + x_color_cell_max, padding + y_lower),
                    CV_RGB(0, 0, 0), 1);
    value+=color_step;
  } // end while (value < max)

  // draw caption
  value = min;
  while (value <= max) {
    // Careful, y_upper corresponds to the upper bound of the COLOR cell
    int y_lower = padding + height_inner * (1.f - (value - min) / max);
    draw_text_centered
        (out, cast_to_string(value),
         cv::Point(padding + width_inner  / 4, y_lower),
         CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 0, 0));
    value+=caption_step;
  } // end while (value <= max)
  return true;
} // end colormap_to_caption_image()

} // end namespace vision_utils

#endif // ARRAY_TO_COLOR_H
