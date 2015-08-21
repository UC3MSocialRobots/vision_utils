/*!
  \file        stats_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/8/6
  
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

Some useful statistics functions

 */

#ifndef STATS_UTILS_H
#define STATS_UTILS_H

#include <math.h>
#include <vector>
#include <numeric> // accumulate
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf = debugging
#include "vision_utils/utils/clamp.h"
#include <vision_utils/utils/system_utils.h>

/*! from http://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation/1719155#1719155
  Return the median of a sequence of numbers defined by the random
  access iterators begin and end.  The sequence must not be empty
  median is undefined for an empty set).
  The numbers must be convertible to double. */
template<class RandAccessIter>
double median(RandAccessIter begin, RandAccessIter end) {
  if(begin == end){
    printf("median():empty list!\n");
    return -1;
  }
  std::size_t size = end - begin;
  std::size_t middleIdx = size/2;
  RandAccessIter target = begin + middleIdx;
  std::nth_element(begin, target, end);

  if(size % 2 != 0){ //Odd number of elements
    return *target;
  }
  //Even number of elements
  double a = *target;
  RandAccessIter targetNeighbor= target-1;
  std::nth_element(begin, targetNeighbor, end);
  return (a+*targetNeighbor)/2.0;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline double mean
(const T* data, unsigned int data_size)
{
  if (data_size == 0) {
    printf("mean(): empty data\n");
    return 0;
  }
  return 1.f * std::accumulate(data, data + data_size, (T) 0) / data_size;
} // end mean()

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline void mean_std_dev
(const T* data, unsigned int data_size, double & mean, double & std_dev)
{
  if (data_size == 0) {
    printf("mean_std_dev(): empty data\n");
    mean = 0;
    std_dev = 0;
    return;
  }
  // mean
  mean = 1.f * std::accumulate(data, data + data_size, (T) 0) / data_size;

  // std dev
  double sum_squared = 0;
  for (unsigned int i = 0; i < data_size; ++i)
    sum_squared += 1.f * (data[i] - mean) * (data[i] - mean);
  std_dev =sqrt(sum_squared / (data_size - 1));
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline void mean_std_dev
(const std::vector<T> & data, double & mean, double & std_dev)
{
  mean_std_dev(data.data(), data.size(), mean, std_dev);
}

////////////////////////////////////////////////////////////////////////////////

// from http://www.angelfire.com/blues/michaelyang/ive/dms/chapter_05/5_6_StaDev.html
template<class T>
inline void mean_std_dev_grouped_data
(const T* mid_pts, const T* occurs, unsigned int data_size,
 double & mean, double & std_dev)
{
  if (data_size == 0) {
    printf("mean_std_dev_grouped_data(): empty mid_pts\n");
    mean = 0;
    std_dev = 0;
    return;
  }
  // mean
  double sum_occur = std::accumulate(occurs, occurs + data_size, (T) 0),
      sum_occur_inv = 1. / sum_occur;
  // printf("sum_occur:%g\n", sum_occur);
  if (fabs(sum_occur) < 1E-10) {
    printf("mean_std_dev_grouped_data(): sum_occur=0\n");
    mean = 0;
    std_dev = 0;
    return;
  }
  double sum_crossed = 0;
  for (unsigned int i = 0; i < data_size; ++i)
    sum_crossed += 1.f * mid_pts[i] * occurs[i];
  mean = sum_crossed * sum_occur_inv;
  // std_dev
  double sum_squared = 0;
  for (unsigned int i = 0; i < data_size; ++i)
    sum_squared += 1.f * mid_pts[i] * mid_pts[i] * occurs[i];
  if (sum_occur > data_size) // use sample variance (s^2)
    std_dev = sqrt((sum_squared - sum_crossed * sum_crossed * sum_occur_inv) / (sum_occur - 1));
  else // use population variance (s^2)
    std_dev = sqrt((sum_squared - sum_crossed * sum_crossed * sum_occur_inv) * sum_occur_inv);
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline void mean_std_dev_grouped_data
(const std::vector<T> & mid_pts, const std::vector<T>& occurs,
 double & mean, double & std_dev)
{
  if (mid_pts.size() != occurs.size()) {
    printf("mean_std_dev_grouped_data(): mid_pts.size()=%i != occurs.size()=%i\n",
           mid_pts.size(), occurs.size());
    return;
  }
  mean_std_dev_grouped_data(mid_pts.data(), occurs.data(), mid_pts.size(), mean, std_dev);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief mean_std_dev_grouped_data_modulo
 * \param mid_pts
 * \param occurs
 * \param min_pt, max_pt
 *    the pts of mid_pts are such as min_pt = max_pt
 *    (makes sense for instance for the hue component, where 0 = 360)
 * \param data_size
 * \param mean
 * \param std_dev
 */
template<class T>
inline void mean_std_dev_grouped_data_modulo
(const std::vector<T> & mid_pts, const std::vector<T> & occurs,
 T min_pt, T max_pt, std::vector<T> & mid_pts_shifted,
 double & mean, double & std_dev)
{
  if (mid_pts.size() != occurs.size()) {
    printf("mean_std_dev_grouped_data(): mid_pts.size()=%i != occurs.size()=%i\n",
           mid_pts.size(), occurs.size());
    return;
  }
  unsigned int data_size = mid_pts.size();
  if (data_size == 0) {
    printf("mean_std_dev_grouped_data(): empty mid_pts\n");
    mean = 0;
    std_dev = 0;
    return;
  }
  // first find peak
  T peak_pt = 0, max_occur = 0;
  for (unsigned int i = 0; i < data_size; ++i) {
    if (occurs[i] > max_occur) {
      max_occur = occurs[i];
      peak_pt = mid_pts[i];
    }
  } // end loop i
  // determine the needed shift: center the peak, i.e. peak -> (min_pt + max_pt)/2
  double pt_shift = .5 * ((double) min_pt + (double) max_pt) - peak_pt;
  // printf("pt_shift:%g\n", pt_shift);

  // then shift the pt values
  mid_pts_shifted.resize(data_size);
  for (unsigned int i = 0; i < data_size; ++i) {
    mid_pts_shifted[i] = modulo_real((T) (mid_pts[i] + pt_shift), min_pt, max_pt);
    // printf("data:%g:%g\n", (double) mid_pts_shifted[i], (double) occurs[i]);
  } // end loop i

  mean_std_dev_grouped_data(mid_pts_shifted, occurs, mean, std_dev);

  // now shift back mean
  mean = modulo_real((T) (mean - pt_shift), min_pt, max_pt);
}

////////////////////////////////////////////////////////////////////////////////

// from http://en.wikipedia.org/wiki/Estimation_of_covariance_matrices
template<class T>
inline void cov_matrix2
(const T* X, const T* Y, unsigned int data_size,
 double & meanX, double & meanY,
 double & c11, double & c12, double & c22)
{
  double data_size_inv = 1. / data_size, data_size_m_inv = 1. / (data_size - 1);
  // NaN check
  //  for (unsigned int data_idx = 0; data_idx < data_size; ++data_idx)
  //    if (isnan(X[data_idx]))
  //      printf("data_idx:%i is nan!\n", data_idx);

  c11 = c12 = c22 = 0;
  meanX = 1.f * std::accumulate(X, X + data_size, (T) 0) * data_size_inv;
  meanY = 1.f * std::accumulate(Y, Y + data_size, (T) 0) * data_size_inv;
  for (unsigned int i = 0; i < data_size; ++i) {
    double xd = X[i] - meanX, yd = Y[i] - meanY;
    c11 += xd * xd;
    c12 += xd * yd;
    c22 += yd * yd;
  } // end loop i
  c11 *= data_size_m_inv;
  c12 *= data_size_m_inv;
  c22 *= data_size_m_inv;
}

////////////////////////////////////////////////////////////////////////////////

// from http://en.wikipedia.org/wiki/Estimation_of_covariance_matrices
template<class T>
inline void cov_matrix3
(const T* X, const T* Y, const T* Z, unsigned int data_size,
 double & meanX, double & meanY, double & meanZ,
 double & c11, double & c12, double & c13,
 double & c22, double & c23,
 double & c33) {
  c11 = c12 = c13 = c22 = c23 = c33 = 0;
  double data_size_inv = 1. / data_size, data_size_m_inv = 1. / (data_size - 1);
  meanX = 1.f * std::accumulate(X, X + data_size, (T) 0) * data_size_inv;
  meanY = 1.f * std::accumulate(Y, Y + data_size, (T) 0) * data_size_inv;
  meanZ = 1.f * std::accumulate(Z, Z + data_size, (T) 0) * data_size_inv;
  for (unsigned int i = 0; i < data_size; ++i) {
    double xd = X[i] - meanX, yd = Y[i] - meanY, zd = Z[i] - meanZ;
    c11 += xd * xd;
    c12 += xd * yd;
    c13 += xd * zd;
    c22 += yd * yd;
    c23 += yd * zd;
    c33 += zd * zd;
  } // end loop i
  c11 *= data_size_m_inv;
  c12 *= data_size_m_inv;
  c13 *= data_size_m_inv;
  c22 *= data_size_m_inv;
  c23 *= data_size_m_inv;
  c33 *= data_size_m_inv;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline void cov_matrix2_vec(const std::vector<T> & X, const std::vector<T> & Y,
                            double & meanX, double & meanY,
                            double & c11, double & c12, double & c22) {
  if (X.size() != Y.size()) {
    printf("cov_matrix(): X.size()=%i != Y.size()=%i\n",
           X.size(), Y.size());
    c11 = c12 = c22 = 0;
    return;
  }
  cov_matrix2(X.data(), Y.data(), X.size(), meanX, meanY, c11, c12, c22);
}

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

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
  system_utils::exec_system(order.str());
}

////////////////////////////////////////////////////////////////////////////////

/*!  Computes eigenvectors for a 2x2 matrix,
 * ( a b )
 * ( c d)
 *\see http://www.math.harvard.edu/archive/21b_fall_04/exhibits/2dmatrices/index.html
   http://en.wikipedia.org/wiki/Eigenvalue_algorithm#2.C3.972_matrices */
template<class Vec2>
bool eigenvectors_2x2(const double & a, const double & b,
                      const double & c, const double & d,
                      double & eigen1, double & eigen2,
                      Vec2 & V1, Vec2 & V2) {
  double T = a + d,
      D = a * d - b *c,
      t = .25 * T * T - D;
  if (t < 0)
    return false;
  eigen1 = .5 * T + sqrt(t);
  eigen2 = -eigen1 + T;
  V1 = Vec2(a - eigen2, c);
  if (fabs(V1.x) < 1E-2 && fabs(V1.y) < 1E-2) // avoid null vectors
    V1 = Vec2(b, d - eigen2);
  V2 = Vec2(a - eigen1, c);
  if (fabs(V2.x) < 1E-2 && fabs(V2.y) < 1E-2) // avoid null vectors
    V2 = Vec2(b, d - eigen1);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////

//! scale and shift \arg data such as its mean  = 0 and variance = 1
// http://www.quora.com/Support-Vector-Machines/SVM-performance-depends-on-scaling-and-normalization-Is-this-considered-a-drawback
// http://stackoverflow.com/questions/8717139/zero-mean-and-unit-variance-of-a-signal
template<class T>
inline void normalize(std::vector<T> & data) {
  double mean, std_dev;
  mean_std_dev(data, mean, std_dev);
  if (fabs(std_dev) < 1E-10) {
    std::fill(data.begin(), data.end(), 0);
    return;
  }
  double std_dev_inv = 1. / std_dev;
  for (unsigned int data_idx = 0; data_idx < data.size(); ++data_idx)
    data[data_idx] = (data[data_idx] - mean) * std_dev_inv;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
inline std::string stats2string(std::vector<T> & data) {
  T min_value = *(std::min_element(data.begin(), data.end()));
  T max_value = *(std::max_element(data.begin(), data.end()));
  double mean, std_dev;
  mean_std_dev(data, mean, std_dev);
  // convert to string
  std::ostringstream ans;
  ans << data.size() << " elts"
      << ", min:" << min_value
      << ", max:" << max_value
      << ", mean:" << mean
      << ", std_dev:" << std_dev;
  return ans.str();
}

#endif // STATS_UTILS_H
