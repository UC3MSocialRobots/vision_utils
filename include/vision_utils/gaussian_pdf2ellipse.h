/*!
  \file        gaussian_pdf2ellipse.h
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

#ifndef GAUSSIAN_PDF2ELLIPSE_H
#define GAUSSIAN_PDF2ELLIPSE_H
// std includes
#include <vector>
#include "vision_utils/cov_matrix.h"
#include "vision_utils/eigenvectors_2x2.h"

namespace vision_utils {

/** \see http://en.wikipedia.org/wiki/Multivariate_normal_distribution#Geometric_interpretation
 * The equidensity contours of a non-singular multivariate normal distribution
 * are ellipsoids (i.e. linear transformations of hyperspheres) centered at the
 * mean. The directions of the principal axes of the ellipsoids are given
 * by the eigenvectors of the covariance matrix Σ. The squared relative lengths
 * of the principal axes are given by the corresponding eigenvalues.
 * \arg radius the length of the axes, to be multiplied by sqrt(eigenvectors)
 */
template<class Pt2>
inline bool gaussian_pdf2ellipse(const double & x_mean, const double & y_mean,
                                 const double & cov_xx, const double & cov_xy, const double & cov_yy,
                                 Pt2 & center, Pt2 & end1, Pt2 & end2,
                                 double radius = 1.) {
  double eigen1, eigen2;
  bool ok = eigenvectors_2x2(cov_xx, cov_xy, cov_xy, cov_yy,
                             eigen1, eigen2, end1, end2);
  if (!ok)
    return false;
  center.x = x_mean;
  center.y = y_mean;
  // normalize vectors
  double mult = radius * sqrt(eigen1) / hypot(end1.x, end1.y);
  end1.x = x_mean + end1.x * mult;
  end1.y = y_mean + end1.y * mult;
  mult = radius * sqrt(eigen2) / hypot(end2.x, end2.y);
  end2.x = x_mean + end2.x * mult;
  end2.y = y_mean + end2.y * mult;
  return true;
}

////////////////////////////////////////////////////////////////////////////////
//! \arg radius the length of the axes, to be multiplied by sqrt(eigenvectors)
template<class T, class Pt2>
inline bool gaussian_pdf_ellipse(const std::vector<T> & X, const std::vector<T> & Y,
                                 Pt2 & center, Pt2 & end1, Pt2 & end2,
                                 double radius = 1.) {
  double meanX = -1, meanY = -1, c11 = -1, c12 = -1, c22 = -1;
  cov_matrix2_vec(X, Y, meanX, meanY, c11, c12, c22);
  bool ok = gaussian_pdf2ellipse(meanX, meanY, c11, c12, c22,
                                 center, end1, end2, radius);
  return ok;
}

} // end namespace vision_utils

#endif // GAUSSIAN_PDF2ELLIPSE_H
