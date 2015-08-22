/*!
  \file        pie_chart_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/13

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

Some utilities for drawing pie charts and their associated captions.

 */

#ifndef PIE_CHART_UTILS_H
#define PIE_CHART_UTILS_H

#include <vector>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/colormaps.h"
#include "vision_utils/drawing_utils.h"
#include "vision_utils/utils/geometry_utils.h"

namespace pie_chart_utils {

static const int PIE_FONT_FACE = CV_FONT_HERSHEY_COMPLEX_SMALL;
static const int CAPTION_FONT_FACE = PIE_FONT_FACE;
static const unsigned int CAPTION_VERTICAL_PADDING = 5;
static const unsigned int CAPTION_HORIZONTAL_PADDING = 10;


/*!
 * Draw a nice pie chart.
 * \param values
 *    the input values (can be of any numerical type: int, double, etc.)
 *    do not need to be normalized.
 * \param out
 *    the output image
 * \param width, height
 *    the desired dimensions in pixels for out
 * \param draw_edges
 *    true for drawing the edges of each slice in black
 * \param draw_values_thres_ratio
 *    if slice value > draw_values_thres_ratio * total,
 *    draw as text the numerical value in the slice center.
 *    0 for drawing all of them, > 1 for none of them.
 * \param colormap
 *    how to determine the color of each slice
 * \param label_font_scale
 *    For drawn labels, the scale of the font to draw the values
 */
template<class _T>
void make_pie(const std::vector<_T> & values,
              cv::Mat3b & out, int width = 400, int height = 400,
              bool draw_edges = true,
              const float draw_values_thres_ratio = 0,
              colormaps::IndexColormap colormap = colormaps::index2predefined_color_washed,
              const double label_font_scale = 1) {
  // dimensions check
  if (out.cols < width || out.rows < height)
    out.create(height, width); // rows, cols
  // clear img
  out.setTo(255);

  // determine the sum of values
  double sum_values = std::accumulate(values.begin(),values.end(), 0.);
  if (sum_values <= 0) {
    printf("make_pie:%s:sum of values <= 0, impossible to draw chart!\n",
           string_utils::accessible_to_string(values).c_str());
    return;
  }

  // drawing params
  cv::Point circle_center(width / 2, height / 2);
  int circle_radius = .8 * std::min(width, height) / 2, label_radius = circle_radius * 2 / 3;
  cv::Size ellipse_axes(circle_radius, circle_radius);
  unsigned int nvalues = values.size();
  cv::Scalar black = CV_RGB(0, 0, 0);
  char txt[30];
  double min_val_to_be_drawn = sum_values * draw_values_thres_ratio;

  // iterate on all values
  double prev_angle_deg = -90;
  for (unsigned int value_idx = 0; value_idx < nvalues; ++value_idx) {
    // determine curr_angle: angular slice = [prev_angle .. curr_angle]
    _T curr_val = values[value_idx];
    double curr_angle_deg = prev_angle_deg + 360. * curr_val / sum_values;

    //    printf("sum_value:%g, prev_angle:%g, curr_angle:%g, curr_color:(%g, %g, %g, %g)\n",
    //           sum_values, prev_angle_deg, curr_angle_deg,
    //           curr_color[0], curr_color[1], curr_color[2], curr_color[3]);
    cv::ellipse(out, circle_center, ellipse_axes,
                0, prev_angle_deg, curr_angle_deg, colormap(value_idx), -1);

    if (curr_val > min_val_to_be_drawn) {
      double median_angle = DEG2RAD * (prev_angle_deg + curr_angle_deg) / 2;
      //std::ostringstream txt;
      // txt << std::setprecision(2) << values[value_idx];
      sprintf(txt, "%.2f", values[value_idx]);
      image_utils::draw_text_centered
          (out, txt,
           circle_center + cv::Point(label_radius * cos(median_angle),
                                     label_radius * sin(median_angle)),
           PIE_FONT_FACE, label_font_scale, black);
    }

    if (draw_edges)
      cv::line(out, circle_center, circle_center +
               cv::Point(circle_radius * cos(prev_angle_deg * DEG2RAD),
                         circle_radius * sin(prev_angle_deg * DEG2RAD)),
               black, 2);

    // store current values
    prev_angle_deg = curr_angle_deg;
  } // end loop value_idx

  // draw circle contour
  if (draw_edges) {
    cv::circle(out, circle_center, circle_radius, black, 2);
    cv::line(out, circle_center, circle_center + cv::Point(0, -circle_radius), black, 2);
  }

  // draw total
  if (draw_values_thres_ratio < 1) {
    sprintf(txt, "Total:%.2f", sum_values);
    int circle_lower_end = circle_center.y + circle_radius;
    image_utils::draw_text_centered
        (out, txt, cv::Point(width / 2, (height + circle_lower_end) / 2),
         PIE_FONT_FACE, label_font_scale, black);
  }
} // end make_pie();

////////////////////////////////////////////////////////////////////////////////

/*!
 * Determine the height a caption would need.
 * \param nlabels
 *    The number of labels that will be in the caption.
 * \param needed_caption_height
 *    out: the total height in pixels of the caption
 * \param caption_row_height
 *    out: the height of each row in pixels in the caption
 */
inline void compute_needed_caption_height(const unsigned int nlabels,
                                          int & needed_caption_height,
                                          int & caption_row_height) {
  int baseline = 0;
  int text_line_size =
      cv::getTextSize("M", CAPTION_FONT_FACE, 1, 1, &baseline).height;
  caption_row_height = text_line_size + CAPTION_VERTICAL_PADDING;
  needed_caption_height = nlabels * caption_row_height + CAPTION_VERTICAL_PADDING; // for the last row
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Draw a nice caption for a pie chart.
 * \param labels
 *    the vector of labels
 * \param out
 *    the output image
 * \param width
 *    the desired width for the caption.
 *    The height is automatically determined with compute_needed_caption_height()
 * \param draw_edges
 *    true for drawing black boxes around each color rectangle
 * \param colormap
 *    how to determine the color of each slice
 */
void make_caption(const std::vector<std::string> & labels,
                  cv::Mat3b & out, int width = 400,
                  bool draw_edges = true,
                  colormaps::IndexColormap colormap = colormaps::index2predefined_color_washed) {
  // determine the height of one row
  unsigned int nlabels = labels.size();
  int needed_caption_height, caption_row_height;
  compute_needed_caption_height(nlabels, needed_caption_height, caption_row_height);

  // dimensions check
  if (out.cols < width || out.rows < needed_caption_height)
    out.create(needed_caption_height, width); // rows, cols
  // clear img
  out.setTo(255);


  // iterate on all labels
  cv::Scalar black = CV_RGB(0, 0, 0);
  for (unsigned int label_idx = 0; label_idx < nlabels; ++label_idx) {
    // draw color rectangle
    cv::Rect color_rec(CAPTION_HORIZONTAL_PADDING,
                       CAPTION_VERTICAL_PADDING + label_idx * caption_row_height,
                       2 * caption_row_height,
                       caption_row_height - CAPTION_VERTICAL_PADDING);
    cv::rectangle(out, color_rec, colormap(label_idx), -1);
    if (draw_edges)
      cv::rectangle(out, color_rec, black, 1);

    // draw name
    cv::putText(out, labels[label_idx],
                color_rec.br() + cv::Point(CAPTION_HORIZONTAL_PADDING, 0),
                CAPTION_FONT_FACE, 1, black);
  } // end loop label_idx
} // end make_caption()

////////////////////////////////////////////////////////////////////////////////

/*!
 * A combination of make_pie() and make_caption() that makes both in a single image.
 * \param values, draw_edges, draw_values_thres_ratio, colormap, label_font_scale
 *    cf make_pie()
 * \param labels
 *   cf make_caption()
 * \param out
 *    the image that will contain both pie and caption.
 * \param width, height
 *    the wanted dimensions in pixels for \a out.
 *    the needed height for the caption is determined with compute_needed_caption_height()
 *    and reserved in \a height.
 *    The remaining space is used for the pie.
 *    If \a height is too small, \a out.height will be bigger than \a height.
 */
template<class _T>
void make_pie_and_caption(const std::vector<_T> & values,
                          const std::vector<std::string> & labels,
                          cv::Mat3b & out, int width = 400, int height = 400,
                          bool draw_edges = true,
                          const float draw_values_thres_ratio = 0,
                          colormaps::IndexColormap colormap = colormaps::index2predefined_color_washed,
                          const double label_font_scale = 1) {
  // determine min height
  int needed_caption_height, caption_row_height;
  compute_needed_caption_height(labels.size(), needed_caption_height, caption_row_height);
  int needed_total_height = std::max(needed_caption_height + 100, height);

  // dimensions check
  if (out.cols < width || out.rows < needed_total_height)
    out.create(needed_total_height, width); // rows, cols

  // determine ROIS
  int pie_height = needed_total_height - needed_caption_height;
  cv::Mat3b out_pie_roi = out( cv::Rect (0, 0, width, pie_height) );
  cv::Mat3b out_caption_roi = out( cv::Rect (0, pie_height, width, needed_caption_height) );
  // call subfunctions
  make_pie(values, out_pie_roi, width, pie_height, draw_edges, draw_values_thres_ratio, colormap, label_font_scale);
  make_caption(labels, out_caption_roi, width, draw_edges, colormap);
  if (draw_edges)
    cv::line(out, cv::Point(0, pie_height), cv::Point(width, pie_height), CV_RGB(0, 0, 0));
} // end make_pie_and_caption()

} // end namespace pie_chart_utils

#endif // PIE_CHART_UTILS_H
