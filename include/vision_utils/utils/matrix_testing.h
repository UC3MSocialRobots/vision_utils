/*!
  \file        matrix_testing.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/29

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

Some useful functions for testing if matrices are equal

 */

#ifndef MATRIX_TESTING_H
#define MATRIX_TESTING_H

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

namespace matrix_testing {

inline std::string infosImage(const cv::Mat & i) {
  std::ostringstream concl;
  concl << "size:(" << i.cols << "x" << i.rows << "), ";
  concl << i.channels() << " channels";
  concl << ", depth:" << i.depth();
  concl << ", type:" << i.type();
  concl << ", isContinuous:" << i.isContinuous();
  return concl.str();
}

////////////////////////////////////////////////////////////////////////////////

inline bool matrices_near(const cv::Mat & A, const cv::Mat & B,
                          float max_diff) {
  if (A.size() != B.size() || A.type() != B.type() || A.channels() != B.channels()) {
    printf("matrices_near(): shape mismatch: [%s], [%s]\n",
           infosImage(A).c_str(), infosImage(B).c_str());
    return false;
  }
  cv::Mat Areshape = A.reshape(1), Breshape = B.reshape(1),
      diff = cv::abs(Areshape - Breshape);
  cv::Point minLoc, maxLoc;
  double minVal, maxVal;
  cv::minMaxLoc(diff, &minVal, &maxVal, &minLoc, &maxLoc);
  if (maxVal > max_diff) {
    if (A.cols * A.rows < 500)
      std::cout << "matrices_near(): A:" << std::endl << A << std::endl
                << ", B:" << std::endl << B << std::endl;
    std::cout << "matrices_near(): " << maxLoc << ": diff:" << maxVal << std::endl;
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////

inline bool matrices_equal(const cv::Mat & A, const cv::Mat & B) {
  return matrices_near(A, B, 1E-12);
}

////////////////////////////////////////////////////////////////////////////////

inline bool matrice_size_equal(const cv::Mat & A,
                               const int cols, const int rows,
                               const int channels = -1, const int type = -1) {
  if (A.cols != cols || A.rows != rows ||
      (channels >= 0 && A.channels() != channels) || (type >= 0 && A.type() != type)) {
    printf("matrices_near(): shape mismatch: [%s], (%ix%i), channels:%i, type:%i\n",
           infosImage(A).c_str(), cols, rows, channels, type);
    return false;
  }
  return true;
}

inline bool matrice_size_equal(const cv::Mat & A,
                               const cv::Mat & B) {
  return matrice_size_equal(A, B.cols, B.rows, B.channels(), B.type());
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   compute the rate of change between two images
 * both have to be the same size and ROI than frame
 *
 * \param   i1 the first image
 * \param   i2 the second image
 * \return  the rate of change (between 0 and 1)
 * \param frame_diff
 *   A buffer that will be used to make the absolute difference between
 *   \a i1 and \a i2.
 * \param frame_diff
 *   A buffer that will contain a thresholded version of \a frame_diff
 * \param threshold_pixel_diff_
 *   The threshold between two pixel values in i1/i2 to
 *   consider them as different values.
 * \return
 *   0 if both images are exactly the same
 *   1 if they are completely different
 *   (each pair of corresponding pixels differs from more than
 *   \a threshold_pixel_diff_).
 */
template<class _T>
static double rate_of_changes_between_two_images
(const cv::Mat_<_T> &i1,
 const cv::Mat_<_T> &i2,
 cv::Mat_<_T> & frame_diff,
 double threshold_pixel_diff)
{
  if (i1.empty() || i2.empty()) {
    printf("rate_of_changes_between_two_images():"
           "At least one of the two images is empty\n");
    return 1;
  }
  if (i1.size() != i2.size() || i1.type() != i2.type()) {
    printf("rate_of_changes_between_two_images():"
           "Images must have the same size and type.\n");
    return 1;
  }
  //    printf("rate_of_changes_between_two_images([%s], [%s])\n",
  //                 image_utils::infosImage(i1).c_str(),
  //                 image_utils::infosImage(i2).c_str());
  cv::absdiff(i1, i2, frame_diff);
  cv::threshold(frame_diff, frame_diff, threshold_pixel_diff, 255,
                CV_THRESH_BINARY);

  /* count the proportion of non zeros */
  double rate_of_moving_px = 1.f * cv::countNonZero(frame_diff)
      / (i1.cols * i1.rows);
  return rate_of_moving_px;
}


} // end namespace matrix_testing

#endif // MATRIX_TESTING_H
