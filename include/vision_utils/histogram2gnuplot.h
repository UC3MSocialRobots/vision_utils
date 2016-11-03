/*!
  \file        histogram2gnuplot.h
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

#ifndef HISTOGRAM2GNUPLOT_H
#define HISTOGRAM2GNUPLOT_H
// std includes
#include <sstream> // for ostringstream
#include <vector>

namespace vision_utils {

/*! plot histogram in gnuplot
  \var mean, std_dev: if wanted, draw the PDF of the histogram
                      (obtained with mean_std_dev() )
  \var xmin, xmax: only useful if you want to extend the drawing of the gaussian PDF
  \var normalize_pdf: true to set the max value of the PDF
                      to the max value of the histogram (i.e., scale the PDF)
*/
void histogram2gnuplot(const Histogram & h, const double max_value,
                       const double mean = -1, const double std_dev = -1,
                       const double xmin = -1, const double xmax = -1,
                       bool normalize_pdf = true) {
  std::vector<double> hues, freqs;
  histogram2vectors(h, max_value, hues, freqs);
  // plot!
  Gnuplot plotter;
  if (xmin != -1 && xmax != -1)
    plotter.set_xrange(xmin, xmax);
  plotter.set_style("boxes");
  plotter.plot_xy(hues, freqs);
  if (mean != -1 && std_dev != -1) {
    std::ostringstream equa;
    if (normalize_pdf) {
      double max_bin_value, max_bin_index;
      if (hist_max(h, max_bin_value, max_bin_index, false))
        equa << max_bin_value << " * ";
    }
    equa << gaussian_pdf_as_string(mean, std_dev, true); // max value = 1
    plotter.set_style("lines");
    plotter.plot_equation(equa.str());
  }
  wait_for_key();
} // end mean_std_dev2gnuplot();

} // end namespace vision_utils

#endif // HISTOGRAM2GNUPLOT_H
