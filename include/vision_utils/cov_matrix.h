/*!
  \file        cov_matrix.h
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

#ifndef COV_MATRIX_H
#define COV_MATRIX_H
// std includes
#include <stdio.h> // for printf(), etc
#include <vector>
#include <numeric> // for accumulate

namespace vision_utils {

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
    printf("cov_matrix(): X.size()=%li != Y.size()=%li\n",
           X.size(), Y.size());
    c11 = c12 = c22 = 0;
    return;
  }
  cov_matrix2(X.data(), Y.data(), X.size(), meanX, meanY, c11, c12, c22);
}

} // end namespace vision_utils

#endif // COV_MATRIX_H
