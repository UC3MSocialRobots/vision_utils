/*!
  \file        gaussian_pdf2gnuplot.h
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

#ifndef GAUSSIAN_PDF2GNUPLOT_H
#define GAUSSIAN_PDF2GNUPLOT_H
// std includes
#include <sstream> // for ostringstream
// vision_utils
#include <vision_utils/exec_system.h>

namespace vision_utils {

//! call gnuplot to draw the PDF of a given gaussian
inline void gaussian_pdf2gnuplot
(const double & mean, const double & std_dev,
 bool normalize = false, const double xmin = -1, const double xmax = -1)
{
  std::ostringstream order;
  order << "gnuplot -e \"";
  if (xmin != -1 && xmax != -1)
    order << "set xr [" << xmin << ":" << xmax << "] ;";
  order << "plot " << gaussian_pdf_as_string(mean, std_dev, normalize)
        << "; pause(-1)\"";
  vision_utils::exec_system(order.str());
}

} // end namespace vision_utils

#endif // GAUSSIAN_PDF2GNUPLOT_H
