/*!
  \file        chart_pie_utils.h
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

\todo Description of the file

 */

#ifndef CHART_PIE_UTILS_H
#define CHART_PIE_UTILS_H

#include <vector>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/colormaps.h"


namespace vision_utils {

void make_pie(const std::vector<double> & values,
              cv::Mat3b & out, int width = 400, int height = 400,
              bool draw_edges = true,
              IndexColormap colormap = index2predefined_color) {
  // dimensions check
  if (out.cols < width || out.rows < height)
    out.create(height, width); // rows, cols
  // clear img
  out.setTo(255);

  cv::Point circle_center(width / 2, height / 2);
  int circle_radius = .8 * std::min(width, height) / 2;

  // make black circle as contour
  cv::circle(out, circle_center, circle_radius, CV_RGB(0, 0, 0), 1);

  double sum_values = std::accumulate(values.begin(),values.end(),0);
  unsigned int nvalues = values.size();
  double prev_angle_deg = -90;
  // cv::Point prev_end = circle_center + cv::Point(-circle_radius, 0);
  for (unsigned int value_idx = 0; value_idx < nvalues; ++value_idx) {
    // determine curr_angle: angular slice = [prev_angle .. curr_angle]
    double curr_angle_deg = prev_angle_deg + 360. * values[value_idx] / sum_values;

    // draw right edge in curr_color
    cv::Scalar curr_color = colormap(value_idx);
    printf("prev_angle:%g, curr_angle:%g, curr_color:(%g, %g, %g, %g)\n",
           prev_angle_deg, curr_angle_deg,
           curr_color[0], curr_color[1], curr_color[2], curr_color[3]);
    cv::ellipse(out, circle_center, cv::Size(circle_radius, circle_radius),
                0, prev_angle_deg, curr_angle_deg, curr_color, -1);

    // if draw_edges, draw edges in black

    // store current values
    prev_angle_deg = curr_angle_deg;
    // prev_end = curr_end;
  } // end loop value_idx


  // remove black circle
  if (!draw_edges)
    cv::circle(out, circle_center, circle_radius, CV_RGB(255, 255, 255), 1);
} // end make_pie();

} // end namespace vision_utils

#endif // vision_utils_H
