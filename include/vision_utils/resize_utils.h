/*!
  \file        resize_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/5/21

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

\todo Description of the file

 */
#ifndef RESIZE_UTILS_H
#define RESIZE_UTILS_H

#include <limits>
#include <opencv2/imgproc/imgproc.hpp>

namespace vision_utils {
//cut:resize_if_bigger
/*!
  Resize an image if it is bigger than the given limit dimensions
 \param big_img
    The image to resize
 \param small_img
 \param max_width , max_height
    The maximum dimensions for \a small_img
 \param force_resize
    If true, big_img will be always be resized into small_img
    even though it is small enough (in that case, we perform a simple deep copy).
 \return scale
    The scaling factor
    in ]0, 1] if the image was resized
    < 0 else
*/
template<class Img>
inline float resize_if_bigger(const Img & big_img,
                              Img & small_img,
                              const int & max_width ,
                              const int & max_height,
                              int interpolation = cv::INTER_NEAREST,
                              bool resize_if_smaller = false,
                              bool force_resize = false) {
  //printf("resize_if_bigger()");

  // foolproof for empty images
  if (big_img.empty() || big_img.cols <= 0 || big_img.rows <= 0) {
    printf("resize_if_bigger():empty input image!\n");
    small_img = Img();
    return 0;
  }

  // determine the scale
  bool need_resize = false;
  float scale = std::numeric_limits<float>::max();
  if (big_img.cols > max_width || resize_if_smaller) {
    need_resize = true;
    scale = 1.f * max_width / big_img.cols;
    //printf("scale:%g\n", scale);
  }
  if (big_img.rows > max_height || resize_if_smaller) {
    need_resize = true;
    scale = std::min(scale, 1.f * max_height / big_img.rows);
    //printf("scale:%g\n", scale);
  }

  // resize if needed or wanted
  if (need_resize) {
    cv::resize(big_img, small_img, cv::Size(0, 0), scale, scale, interpolation);
    return scale;
  }
  if (!need_resize && force_resize) {
    big_img.copyTo(small_img);
    return 1;
  }

  // no resize done -> return a negative value
  return -1;
} // end resize_if_bigger()

////////////////////////////////////////////////////////////////////////////////

//cut:resize_constrain_proportions
/*!
 Just a ROI of the image will be kept,
 such as it fits in (dst_width, dst_height)
 with the same aspect.
 \param big_img
 \param small_img
 \param width
 \param height
*/
template<class Img>
inline void resize_constrain_proportions(const Img & src,
                                         Img & dst,
                                         const int & dst_width ,
                                         const int & dst_height,
                                         int interpolation = cv::INTER_NEAREST) {
  float ratio_src = 1.f * src.cols / src.rows;
  float ratio_dst = 1.f * dst_width / dst_height;
  /*        width    case 1 +-----+   case2 +-----+-----+-----+
     ratio= ------          | src |         | src | dst | src |
            height          +-----+         +-----+-----+-----+
                            | dst |
                            +-----+
                            | src |
                            +-----+      */
  cv::Rect ROI;
  if (ratio_src < ratio_dst) { // case 1
    //printf("case 1\n");
    ROI.width = src.cols;
    ROI.height = ROI.width / ratio_dst;
    ROI.x = 0;
    ROI.y = (src.rows - ROI.height) / 2;
  }
  else {
    //printf("case 2\n");
    ROI.height = src.rows;
    ROI.width = ROI.height * ratio_dst;
    ROI.y = 0;
    ROI.x = (src.cols - ROI.width) / 2;
  }
  //  printf("src:%i x %i, dst:%i x %i, dst_width:%i, dst_height:%i\n",
  //         src.cols, src.rows, dst.cols, dst.rows, dst_width, dst_height);
  cv::resize(src(ROI), dst, cv::Size(dst_width, dst_height), 0, 0, interpolation);
} // end resize_if_bigger()

////////////////////////////////////////////////////////////////////////////////
//cut:scale_img_forward_backward
template<class _T>
void scale_img_forward_backward(const cv::Mat_<_T> & in, cv::Mat_<_T> & out,
                                const float & xscale = 1, const int & xoffset = 0,
                                const float & yscale = 1, const int & yoffset = 0,
                                bool forward = false) {
  out.create(in.size());
  out.setTo(0);
  if (xscale == 0 || yscale == 0)
    return;
  if (xscale == 1 && yscale == 1) {
    cv::Rect r1(xoffset, yoffset, in.cols, in.rows);
    cv::Rect r1_inter_out = rectangle_intersection_img(out, r1);
    cv::Rect r1_inter_in (r1_inter_out.x - xoffset, r1_inter_out.y - yoffset,
                          r1_inter_out.width, r1_inter_out.height);
    //    printf("r1:%s, r1_inter_in:%s, r1_inter_out:%s\n",
    //           print_rect(r1).c_str(),
    //           print_rect(r1_inter_in).c_str(),
    //           print_rect(r1_inter_out).c_str());
    in(r1_inter_in).copyTo(out(r1_inter_out));
    return;
  }

  int rows = in.rows, cols = in.cols;
  if (forward) {
    for (int row = 0; row < rows; ++row) {
      _T* data_in = (_T*) in.ptr(row);
      for (int col = 0; col < cols; ++col) {
        cv::Scalar color = CV_RGB(data_in[col][2], data_in[col][1], data_in[col][0]);
        cv::Point pt_out(col * xscale + xoffset, row * yscale + yoffset);
        if (pt_out.x >= 0 && pt_out.x < cols && pt_out.y >= 0 && pt_out.y < rows)
          cv::circle(out, pt_out, 2, color, -1);
      } // end loop col
    } // end loop row
  } // end if (forward)
  else { // backward
    int mincol = std::max(0,        (int) (0 * xscale + xoffset)),
        maxcol = std::min(cols - 1, (int) (cols * xscale + xoffset)),
        minrow = std::max(0,        (int) (0 * yscale + yoffset)),
        maxrow = std::min(rows - 1, (int) (rows * yscale + yoffset));
    float xscaleinv = 1.f / xscale, yscaleinv = 1.f / yscale,
        xoffsetinv = -xscaleinv * xoffset, yoffsetinv = -yscaleinv * yoffset;
    _T* data_in = (_T*) in.ptr(0);
    _T* data_out = (_T*) out.ptr(0);
    for (int row = minrow; row <= maxrow; ++row) {
      int in_row = yscaleinv * row + yoffsetinv;
      for (int col = mincol; col <= maxcol; ++col) {
        data_out[row * cols + col] = data_in[in_row * cols + (int) (xscaleinv * col + xoffsetinv)];
      } // end loop col
    } // end loop row
  } // end if (backward)
}
//cut:scale_img_warp
template<class _T>
void scale_img_warp(const cv::Mat_<_T> & in, cv::Mat_<_T> & out,
                    const float & xscale = 1, const int & xoffset = 0,
                    const float & yscale = 1, const int & yoffset = 0) {
  if (xscale == 0 || yscale == 0)
    return;
  cv::Mat1d M(2, 3, (float) 0); // rows, cols
  M(0, 0) = xscale;
  M(1, 1) = yscale;
  M(0, 2) = xoffset;
  M(1, 2) = yoffset;
  cv::warpAffine(in, out, M, in.size(), cv::INTER_NEAREST);
}
//cut
} // end namespace vision_utils

#endif // RESIZE_UTILS_H
