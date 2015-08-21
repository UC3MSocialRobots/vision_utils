#ifndef BORDER_REMOVER_H
#define BORDER_REMOVER_H

#include "vision_utils/utils/debug_utils.h"
#include <opencv2/core/core.hpp>

namespace image_utils {

typedef short int Coord;

////////////////////////////////////////////////////////////////////////////////

/*! a static (hidden) function.
 * It return the median element in a vector.
 * Careful, it changes the order of the elements in the vector
 * \arg ratio 0: the min border, 0.5: its mean, 1: its max
 */
template<class _ElemType>
static _ElemType vector_median(std::vector<_ElemType> &v,
                               const double & ratio = 0.5) {
  int n = (int) (ratio * v.size());
  nth_element(v.begin(), v.begin()+n, v.end());
  return v[n];
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Compute the average border of the image
 * \arg ratio
      0: the min border, 0.5: its mean, 1: its max
 * \return the ROI of the input image
 */
template<class _T>
inline void evaluate_mean_border(const cv::Mat_<_T> src,
                                 Coord & left, Coord & right, Coord & up, Coord & down,
                                 const _T border_value = _T(0),
                                 const double & ratio = 0.5) {
  // prepair the border values
  std::vector<Coord> left_vec, right_vec, up_vec, down_vec;
  left_vec.reserve(src.rows);
  right_vec.reserve(src.rows);
  up_vec.reserve(src.cols);
  down_vec.reserve(src.cols);
  // a header conversion
  IplImage src_ipl = src;

  // iterate on the rows
  for (Coord row = 0; row < src.rows; ++row) {
    // search left border
    for (Coord col = 0; col < src.cols; ++col) {
      if (CV_IMAGE_ELEM(&src_ipl, _T, row, col) != border_value) {
        left_vec.push_back(col);
        break;
      }
    } // end loop col

    // search right border
    for (Coord col = src.cols - 1; col >= 0; --col) {
      if (CV_IMAGE_ELEM(&src_ipl, _T, row, col) != border_value) {
        right_vec.push_back(col);
        break;
      }
    } // end loop col
  } // end loop row

  // iterate on the columns
  for (Coord col = 0; col < src.rows; ++col) {
    // search up border
    for (Coord row = 0; row < src.rows; ++row) {
      if (CV_IMAGE_ELEM(&src_ipl, _T, row, col) != border_value) {
        up_vec.push_back(row);
        break;
      }
    } // end loop row

    // search down border
    for (Coord row = src.rows - 1; row >= 0; --row) {
      if (CV_IMAGE_ELEM(&src_ipl, _T, row, col) != border_value) {
        down_vec.push_back(row);
        break;
      }
    } // end loop row
  } // end loop col

  // get the median
  left = vector_median<Coord>(left_vec, ratio);
  right = vector_median<Coord>(right_vec, ratio);
  up = vector_median<Coord>(up_vec, ratio);
  down = vector_median<Coord>(down_vec, ratio);
} // end evaluate_mean_border

////////////////////////////////////////////////////////////////////////////////

/*!
 * \arg ratio 0: the min border, 0.5: its mean, 1: its max
 * \return the ROI of the input image
 */
template<class _T>
inline cv::Rect remove_border(const cv::Mat_<_T> & src,
                              cv::Mat_<_T> & dst,
                              const _T & border_value = _T(0),
                              const double & ratio = 0.5) {
  //maggieDebug2("remove_border()");
  Coord left, right, up, down;
  evaluate_mean_border(src, left, right, up, down, border_value, ratio);
  maggieDebug3("left:%i, right:%i, up:%i, down:%i",
               left, right, up, down);

  // resize dst
  //dst.create(height, width); // rows, cols

  // copy the center
  cv::Rect roi(left, up,
               right - left + 1, // width
               down - up + 1 // height
               );
  src(roi).copyTo(dst);
  //maggiePrint("dst.rows:%i, dst.cols:%i", dst.rows, dst.cols);
  return roi;
}

////////////////////////////////////////////////////////////////////////////////

/*!
  Paint a precomputed border (i.e frame) on a given image.
 \param dst
 \param left, right, up, down
    Can for instance be obtained from evaluate_mean_border()
 \param color
    The color that will be used to draw the border on dst
*/
inline void paint_average_border_into_other_image
(cv::Mat & dst,
 const Coord & left, const Coord & right, const Coord & up, const Coord & down,
 const cv::Scalar & color) {
  // left rectangle
  cv::rectangle(dst, cv::Point(0, 0), cv::Point(left, dst.rows), color, -1);
  // right rectangle
  cv::rectangle(dst, cv::Point(right, 0), cv::Point(dst.cols, dst.rows), color, -1);
  // up rectangle
  cv::rectangle(dst, cv::Point(0, 0), cv::Point(dst.cols, up), color, -1);
  // down rectangle
  cv::rectangle(dst, cv::Point(0, down), cv::Point(dst.cols, dst.rows), color, -1);
} // end paint_average_border_into_other_image();

////////////////////////////////////////////////////////////////////////////////

/*!
 Find the average border on an image, and paint rectangles in the output image
 that match this average border
 \param src
 \param dst
 \param color
    The color that will be used to draw the border on dst
 \param border_value
 \param ratio
      0: the min border, 0.5: its mean, 1: its max
*/
template<class _T>
inline void compute_average_border_and_paint_into_other_image
(const cv::Mat_<_T> src,
 cv::Mat & dst,
 const cv::Scalar & color,
 const _T border_value = _T(0),
 const double & ratio = 0.5) {
  Coord left, right, up, down;
  image_utils::evaluate_mean_border(src, left, right, up, down, border_value, ratio);
  paint_average_border_into_other_image(dst, left, right, up, down, color);
} // end compute_average_border_and_paint_into_other_image()

} // end namespace image_utils

#endif // BORDER_REMOVER_H
