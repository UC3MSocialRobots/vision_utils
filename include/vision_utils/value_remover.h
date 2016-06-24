#ifndef VALUE_REMOVER_H
#define VALUE_REMOVER_H

// OpenCV
#include <opencv2/core/core.hpp>
#if CV_MAJOR_VERSION > 2 || (CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >= 4)
#include "opencv2/photo/photo.hpp" // for versions 2.4 and +
#endif // CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >= 4
// AD
#include "vision_utils/border_remover.h"
#include "vision_utils/utils/debug_utils.h"

namespace image_utils {

/*! replace the pixel with forbidden value by their left neighbours value
*/
template<class _T>
inline void remove_value_left_propagation(cv::Mat_<_T> & src,
                                          const _T & value_to_remove) {
  // make a left sweep to clean the image from its nan
  cv::Mat src_mat = src;

  for (int row = 0; row < src.rows; ++row) {
    // get the address of row
    _T *row_ptr = src_mat.ptr<_T>(row),
        *curr_pt = row_ptr;
    bool only_zeros_before_in_row = true;
    _T last_value_in_row = 0;
    for (int col = 0; col < src.cols; ++col) {
      if (*curr_pt != value_to_remove) {
        if (only_zeros_before_in_row == true) {
          //maggiePrint("Row %i: cleaning from 0 to %i", row, col);
          // sweep the left nans
          for (int col_neigh = 0; col_neigh < col; ++col_neigh)
            row_ptr[col_neigh] = *curr_pt;
        } // end if only_zeros_before_in_row
        last_value_in_row = *curr_pt;
        only_zeros_before_in_row = false;
      } // end if non zero
      else { // it is a zero
        if (only_zeros_before_in_row == false) { // affect the last value
          *curr_pt = last_value_in_row;
        } // end if only_zeros_before_in_row
      }  // end if it is zero
      ++curr_pt;
    } // end loop col
  } // end loop row
} // end remove_value_left_propagation();

////////////////////////////////////////////////////////////////////////////////

enum NaNRemovalMethod {
  VALUE_REMOVAL_METHOD_DO_NOTHING = 0,
  VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION = 1,
  VALUE_REMOVAL_METHOD_INPAINT = 2,
  VALUE_REMOVAL_METHOD_AVERAGE_BORDER = 3
}; // end NaNRemovalMethod

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline cv::Rect remove_value(const cv::Mat_<_T> & img_uchar,
                             cv::Mat_<_T> & img_uchar_with_no_nan,
                             const _T & value_to_remove,
                             cv::Mat1b & inpaint_mask,
                             const NaNRemovalMethod nan_removal_method) {
  cv::Rect roi(0, 0, img_uchar.cols, img_uchar.rows);
  if (nan_removal_method == VALUE_REMOVAL_METHOD_DO_NOTHING)
    img_uchar.copyTo(img_uchar_with_no_nan);

  else if (nan_removal_method == VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION) {
    // left value propagation
    img_uchar.copyTo(img_uchar_with_no_nan);
    image_utils::remove_value_left_propagation<_T>
        (img_uchar_with_no_nan, value_to_remove);
  } // end if VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION

  else if (nan_removal_method == VALUE_REMOVAL_METHOD_INPAINT) {
    // make inpaint
    inpaint_mask.create(img_uchar.size());
    inpaint_mask = 0;
    inpaint_mask.setTo((uchar) 255, (img_uchar == value_to_remove));
    // cv::threshold(img_uchar, inpaint_mask, 0, 255, cv::THRESH_BINARY_INV);
    cv::inpaint(img_uchar, inpaint_mask, img_uchar_with_no_nan, 5, cv::INPAINT_NS);
  } // en if VALUE_REMOVAL_METHOD_INPAINT

  else if (nan_removal_method == VALUE_REMOVAL_METHOD_AVERAGE_BORDER) {
    // remove border
    roi = image_utils::remove_border(img_uchar, img_uchar_with_no_nan,
                                     value_to_remove, 0.5);
    img_uchar(roi).copyTo(img_uchar_with_no_nan);
  } // end if VALUE_REMOVAL_METHOD_AVERAGE_BORDER

  else {
    printf("remove_value(): Unknown NAN removal method %i\n",
           nan_removal_method);
  }
  return roi;
}



} // end namespace image_utils

#endif // VALUE_REMOVER_H
