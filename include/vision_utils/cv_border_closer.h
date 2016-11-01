#ifndef CV_BORDER_CLOSER_H
#define CV_BORDER_CLOSER_H

#include <opencv2/core/core.hpp>

namespace vision_utils {

template<class _T>
/*! Use the following pattern for points indexing
                 [curr]  [10]  [20]
   [m21]  [m11]   [01]   [11]  [21]
   [m22]  [m12]   [02]   [12]  [22]
 \param src
 \param non_border_value
*/
inline void close_borders(cv::Mat_<_T> src,
                          const _T & non_border_value) {
  _T* p01 = NULL, *p02 = NULL;
  for (int row_idx = 0; row_idx < src.rows; ++row_idx) {
    // get the address of rows
    _T* pcurr = src.ptr(row_idx);
    if (row_idx < src.rows - 2) {
      p01 = src.ptr(row_idx + 1);
      p02 = src.ptr(row_idx + 2);
    }

    // iterate on the columns
    _T *p10 = pcurr + 1, *p20 = p10 + 1,
        *pm21 = p01, *pm11 = p01, *p11 = p01 + 1, *p21 = p11 + 1,
        *pm22 = p01, *pm12 = p01, *p12 = p02 + 1, *p22 = p12 + 1;
    for (int col_idx = 0; col_idx < src.cols; ++col_idx) {
      // X-X      XXX
      // ...  --> ...
      // ...      ...
      if (col_idx < src.cols - 2
          && *pcurr != non_border_value
          && *p10 == non_border_value
          && *p20 != non_border_value ) { // close the border
        *p10 = *pcurr;
      }
      // X..       X..
      // -..  -->  X..
      // X..       X..
      if (row_idx < src.rows - 2
          && *pcurr != non_border_value
          && *p01 == non_border_value
          && *p02 != non_border_value ) { // close the border
        *p01 = *pcurr;
      }
      // X--       XXX
      // ..X  -->  ..X
      // ...       ...

      // increment everybody
      ++pcurr; ++p10; ++p20;
      if (col_idx > 1) {
        ++pm21;
        ++pm22;
      }
      if (col_idx > 0) {
        ++pm11;
        ++pm12;
      }
      ++p01; ++p11; ++p21;
      ++p02; ++p12; ++p22;
    } // end loop col
  } // end loop row
}

} // end namespace vision_utils

#endif // CV_BORDER_CLOSER_H
