/*!
  \file        depth_canny.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/30

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

\class DepthCanny
A class to apply Canny filters on depth images.

 */

#ifndef DEPTH_CANNY_H
#define DEPTH_CANNY_H

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
// AD
#include "vision_utils/value_remover.h"
#include "vision_utils/convert_float_to_uchar.h"
//#define CHART_TIMER_ON
//#define TIMER_ON

#ifdef CHART_TIMER_ON // chart timer
#include "vision_utils/timer_chart.h"

#elif defined TIMER_ON // chart timer
#define TIMER_CREATE(t)                     vision_utils::Timer t;
#define TIMER_RESET(t)                      t.reset();
#define TIMER_PRINT_RESET(t, msg)           t.printTime(msg); t.reset();
#define TIMER_DISPLAY_CHART(t, step)        // empty

#else // do nothing
#define TIMER_CREATE(t)                     // empty
#define TIMER_RESET(t)                      // empty
#define TIMER_PRINT_RESET(t, msg)           // empty
#define TIMER_DISPLAY_CHART(t, step)        // empty
#endif // CHART_TIMER_ON

namespace vision_utils {

class DepthCanny {
public:
  //! the size of the kernel used to try to close contours (pixels)
  static const int MORPH_OPEN_KERNEL_SIZE = 3;
  //  static const double DEFAULT_CANNY_THRES1 = .88; // m
  //  static const double DEFAULT_CANNY_THRES2 = 1.48; // m
  //! decrease to make more edges appear
  static const double DEFAULT_CANNY_THRES1 = 1; // m
  static const double DEFAULT_CANNY_THRES2 = 1.6; // m

  DepthCanny() {
    set_canny_thresholds(DEFAULT_CANNY_THRES1, DEFAULT_CANNY_THRES2);
    set_nan_removal_method(VALUE_REMOVAL_METHOD_DO_NOTHING);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! set Canny parameters, in meters
  inline void set_canny_thresholds(const double canny_thres1,
                                   const double canny_thres2) {
    _canny_thres1 = canny_thres1;
    _canny_thres2 = canny_thres2;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_nan_removal_method(const NaNRemovalMethod m) {
    _nan_removal_method = m;
  }

  //////////////////////////////////////////////////////////////////////////////

  void thresh(const cv::Mat & depth_img) {
    if (depth_img.empty())
      return;

    TIMER_RESET(timer);
    convert_float_to_uchar(depth_img, _img_uchar, src_float_clean_buffer,
                                        _alpha_trans, _beta_trans);

    TIMER_PRINT_RESET(timer, "thresh(): remapping depth float->uchar");

    /*
     *remove NaN from input image
     */
    cv::Rect roi = remove_value
        (_img_uchar, _img_uchar_with_no_nan, NAN_UCHAR,
         _inpaint_mask, _nan_removal_method);
    TIMER_PRINT_RESET(timer, "thresh(): remove_value(NAN_UCHAR)");

    /*
     *edge detection
     */
    // canny
    //printf("canny_thres1:%g, canny_thres2:%g, alpha_trans:%g", _canny_thres1, _canny_thres2, _alpha_trans);
    cv::Canny(_img_uchar_with_no_nan, _edges,
              _alpha_trans *_canny_thres1, _alpha_trans *_canny_thres2);

    std::ostringstream sentence;
    sentence << "thresh(): Canny (param1:" <<_canny_thres1
             << ", param2: " << _canny_thres2 << ")";
    TIMER_PRINT_RESET(timer, sentence.str().c_str());

    /*invert the edges */
#if 1 // faster
    cv::threshold(_edges, _edges_inverted, 128, 255, cv::THRESH_BINARY_INV);
#else
    _edges_inverted = (_edges == 0);
#endif
    TIMER_PRINT_RESET(timer, "thresh(): cv::threshold(_edges) -> _edges_inverted");

    // close borders
    //close_borders(edges_inverted_with_nan, (uchar) 255);
    //    cv::morphologyEx(edges_inverted, edges_inverted_opened,
    //                     cv::MORPH_OPEN,
    //                     cv::Mat(MORPH_OPEN_KERNEL_SIZE, MORPH_OPEN_KERNEL_SIZE, CV_8U, 255));
    cv::erode(_edges_inverted, _edges_inverted_opened,
              cv::Mat(MORPH_OPEN_KERNEL_SIZE, MORPH_OPEN_KERNEL_SIZE, CV_8U, 255));
    TIMER_PRINT_RESET(timer, "thresh(): cv::erode()");

    /*
     *combine canny with nan
     */
#if 1 // faster
    _edges_inverted_opened.copyTo(_edges_inverted_opened_with_nan);
    _img_uchar(roi).copyTo(_edges_inverted_opened_with_nan,
                           (_img_uchar(roi) == NAN_UCHAR));
#else
    cv::min(_edges_inverted_opened, _img_uchar(roi), _edges_inverted_opened_with_nan);
    cv::threshold(_edges_inverted_opened_with_nan, _edges_inverted_opened_with_nan,
                  NAN_UCHAR, 255, cv::THRESH_BINARY);
#endif
    TIMER_PRINT_RESET(timer, "thresh(): combining Canny edges and NAN of depth");
  }

  //////////////////////////////////////////////////////////////////////////////

  inline const cv::Mat1b & get_thresholded_image() const {
    return _edges_inverted_opened_with_nan;
  }
  inline       cv::Mat1b & get_thresholded_image() {
    return _edges_inverted_opened_with_nan;
  }

  //////////////////////////////////////////////////////////////////////////////

  // private:
  TIMER_CREATE(timer)

  // float -> uchar
  ScaleFactorType _alpha_trans, _beta_trans;
  cv::Mat1b _img_uchar;
  cv::Mat src_float_clean_buffer;

  // nan removal
  cv::Mat1b _img_uchar_with_no_nan;
  NaNRemovalMethod _nan_removal_method;
  cv::Mat1b _inpaint_mask;

  // edge detection
  double _canny_thres1, _canny_thres2;
  //  int canny_tb1_value, canny_tb2_value;

  cv::Mat1b _edges;
  cv::Mat1b _edges_inverted;
  cv::Mat1b _edges_inverted_opened_with_nan;
  cv::Mat1b _edges_inverted_opened;
  //cv::Mat1b harrisCorners;
}; // end class DepthCanny

} // end namespace vision_utils

#endif // DEPTH_CANNY_H
