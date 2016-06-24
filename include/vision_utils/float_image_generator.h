/*!
 * compressed_rounded_image_transport is an image_transport plugin for float images.
 * \author Arnaud Ramey ( arnaud.a.ramey@gmail.com )
            -- Robotics Lab, University Carlos III of Madrid
 * \date Jan. 2012
  */

#ifndef FLOAT_IMAGE_GENERATOR_H
#define FLOAT_IMAGE_GENERATOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/nan_handling.h"


/*!
 Generates dst(i, j) = i + j + const_value
 \param dst
    the matrix to be generated
 \param width
    the wanted width
 \param height
    the wanted height
 \param const_value
    the const value to add to each pixel
*/
template<class _T>
inline void generate_sum_row_col(cv::Mat & dst,
                                 const int & width, const int & height,
                                 const double const_value = 0) {
  dst = cv::Mat_<_T>(height, width);
  for (int row = 0; row < dst.rows; ++row) {
    for (int col = 0; col < dst.cols; ++col) {
      cv::line(dst, cv::Point(col, row), cv::Point(col, row),
               cv::Scalar::all(const_value + row + col));
    } // end loop col
  } // end loop row
}

////////////////////////////////////////////////////////////////////////////////

/*!
  generate random values in [min_value .. max_value[
 \param dst
    the matrix to be generated
 \param width
    the wanted width
 \param height
    the wanted height
 \param min_value
    the minimum value for the rand values
 \param max_value
    the maximum value for the rand values
 \param put_nans
    if true, will put some NaNs at random positions
 \param rate_nans
    the percentage of NaNs, ex 0.1 will put more or less 10% of NaNs
*/
template<class _T>
inline void generate_full_rand(cv::Mat & dst,
                               const int & width, const int & height,
                               const double min_value = 1,
                               const double max_value = 10,
                               bool put_nans = false,
                               double rate_nans = 0.1) {
  dst = cv::Mat_<_T>(height, width);
#if 0
  for (int row = 0; row < dst.rows; ++row) {
    for (int col = 0; col < dst.cols; ++col) {
      if (put_nans && rand() % 10 == 0)
        cv::line(dst, cv::Point(col, row), cv::Point(col, row),
                 cv::Scalar::all(0);
      else
        cv::line(dst, cv::Point(col, row), cv::Point(col, row),
                 cv::Scalar(min_value + drand48() * (max_value - min_value),
                            min_value + drand48() * (max_value - min_value),
                            min_value + drand48() * (max_value - min_value),
                            min_value + drand48() * (max_value - min_value)));
    } // end loop col
  } // end loop row
#else
  cv::randu(dst, cv::Scalar::all(min_value), cv::Scalar::all(max_value));
  if (!put_nans)
    return;
  _T nan_v = _T(image_utils::NAN_DEPTH);
  unsigned int nb_nans = width * height * rate_nans;
  for (unsigned int nan_idx = 0; nan_idx < nb_nans; ++nan_idx) {
    int row = rand() % dst.rows, col = rand() % dst.cols;
    dst.at<_T>(row, col) = nan_v;
  }
#endif
}

#endif // FLOAT_IMAGE_GENERATOR_H
