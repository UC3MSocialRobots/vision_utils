/*!
  \file        gaussian_pdf.h
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

#ifndef GAUSSIAN_PDF_H
#define GAUSSIAN_PDF_H
// std includes
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

/*!
* The probability density function of a 2D Gaussian.
* \see http://en.wikipedia.org/wiki/Multivariate_normal_distribution#Properties
* \param r, g
*    The evaluation point
* \param x_mean, y_mean, cov_xx, cov_xy, cov_yy
*    The parameters of the 2D Gaussian, \see train_fit_gaussian()
* \param normalize
*    If false, the integral sum of the PDF is 1,
*    and the peak value, in (x_mean, y_mean), is
*    1 / (2 * PI * sx * sy * sqrt(1 - rho^2)) (big)
* \return
*/
inline float gaussian_pdf
(const float & x, const float & y,
 const double & x_mean, const double & y_mean,
 const double & cov_xx, const double & cov_xy, const double & cov_yy,
 bool normalize = false)
{
  float sx = sqrt(cov_xx), sy = sqrt(cov_yy);
  float rho = cov_xy / (sx * sy);
  float one_m_rho2 = 1.f - rho * rho;
  float ydiff = (y - y_mean), xdiff = (x - x_mean);
  float val = exp( -1.f / (2 * one_m_rho2) * (
                     //pow(r - x_mean, 2) / (sx * sx)
                     xdiff * xdiff / cov_xx
                     +
                     //pow(g - y_mean, 2) / (sy * sy)
                     ydiff * ydiff / cov_yy
                     - 2 * rho / (sx * sy)
                     * xdiff * ydiff
                     ));
  if (!normalize)
    val = val * 1.f / (2 * M_PI * sx * sy * sqrt(one_m_rho2));
  return val;
}

////////////////////////////////////////////////////////////////////////////////

// from http://en.wikipedia.org/wiki/Multivariate_normal_distribution#Properties
inline std::string gaussian_pdf_as_string
(const double & x_mean, const double & y_mean,
 const double & cov_xx, const double & cov_xy, const double & cov_yy,
 bool normalize = false)
{
  float sx = sqrt(cov_xx), sy = sqrt(cov_yy);
  float rho = cov_xy / (sx * sy);
  float one_m_rho2 = 1.f - rho * rho;
  printf("rho:%g\n", rho);
  std::ostringstream equa;
  if (!normalize)
    equa << 1.f / (2 * M_PI * sx * sy * sqrt(one_m_rho2)) << " * ";
  equa << "exp(" << -1.f / (2 * one_m_rho2)<< " * ( "
       <<   "(x - " << x_mean << ")**2 / " << (sx * sx)
         <<   " + "
           <<   "(y - " << y_mean << ")**2 / " << (sy * sy)
             << " - " << 2 * rho / (sx * sy)
             << " * (x - " << x_mean << ")"
             << " * (y - " << y_mean << ")"
             << " ))";
  return equa.str();
}

////////////////////////////////////////////////////////////////////////////////

// http://en.wikipedia.org/wiki/Normal_distribution
inline std::string gaussian_pdf_as_string
(const double & mean, const double & std_dev,
 bool normalize = false)
{
  std::ostringstream equa;
  if (!normalize)
    equa << 1.f / sqrt(2 * M_PI * std_dev * std_dev) << " * ";
  equa << "exp(" << -1.f / (2 * std_dev * std_dev)
       << "*(x - " << mean << ")**2)";
  return equa.str();
}

} // end namespace vision_utils

#endif // GAUSSIAN_PDF_H
