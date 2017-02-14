/*!
  \file        drawListOfPoints.h
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

#ifndef DRAWLISTOFPOINTS_H
#define DRAWLISTOFPOINTS_H
// std includes
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace vision_utils {

/*!
* \brief   draw a list of points
*/
template<class _T>
inline void drawListOfPoints(cv::Mat_<_T> & img,
                             const std::vector<cv::Point> & pts,
                             const _T & color) {
  unsigned int npts = pts.size();
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
    img(pts[pt_idx]) = color;
}

//! for each point of a list, draw it only if it visible
template<class _T>
inline void drawListOfPoints_offset(cv::Mat_<_T> & img,
                                    const std::vector<cv::Point> & pts,
                                    const _T & color,
                                    const cv::Point & offset) {
  unsigned int npts = pts.size();
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
    img(pts[pt_idx]-offset) = color;
}

//! for each point of a list, draw it only if it visible
template<class _T>
inline void drawListOfPoints_safe(cv::Mat_<_T> & img,
                                  const std::vector<cv::Point> & pts,
                                  const _T & color) {
  unsigned int npts = pts.size();
  cv::Rect bbox(0, 0, img.cols, img.rows);
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
    if (bbox.contains(pts[pt_idx]))
      img(pts[pt_idx]) = color;
}

//! for each point of a list, draw it only if it visible
template<class _T>
inline void drawListOfPoints_offset_safe(cv::Mat_<_T> & img,
                                         const std::vector<cv::Point> & pts,
                                         const _T & color,
                                         const cv::Point & offset) {
  unsigned int npts = pts.size();
  cv::Rect bbox(offset.x, offset.y, img.cols, img.rows);
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
    if (bbox.contains(pts[pt_idx]))
      img(pts[pt_idx]-offset) = color;
}

template<class _T>
inline void drawListOfPoints_safe2(cv::Mat_<_T> & img,
                                   const std::vector<cv::Point> & pts,
                                   const cv::Scalar & color, const int thickness) {
  unsigned int npts = pts.size();
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
    cv::circle(img, pts[pt_idx], thickness, color, -1);
}

} // end namespace vision_utils

#endif // DRAWLISTOFPOINTS_H
