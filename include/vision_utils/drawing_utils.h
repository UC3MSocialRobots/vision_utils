#ifndef DRAWING_UTILS_H
#define DRAWING_UTILS_H

#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/geometry_utils.h"
#include "vision_utils/resize_utils.h"
#include "vision_utils/titlemaps.h"
#include "vision_utils/draw_arrow.h"

namespace vision_utils {
//cut:drawLine

////////////////////////////////////////////////////////////////////////////////

inline void drawLine(cv::Mat & img,
                     const double & a,
                     const double & b,
                     const double & c,
                     const cv::Scalar & color,
                     const int thickness = 2) {
  //cout << "line : 0," << (int) (-eqn.val[2] / eqn.val[1]) << " - 100, " << (int) ((-100 * eqn.val[0] - eqn.val[2]) / eqn.val[1]) << endl;
  cv::line(img,
           cv::Point(0, (int) (-c / b)),
           cv::Point(1000, (int) ((-1000 * a - c) / b)),
           color, thickness);
}

////////////////////////////////////////////////////////////////////////////////
//cut:draw_segment

inline void draw_segment(cv::Mat & img,
                         const cv::Vec4i & segment_ends,
                         const cv::Scalar & color,
                         const int thickness = 2) {
  cv::line(img,
           cv::Point2i(segment_ends[0], segment_ends[1]),
      cv::Point2i(segment_ends[2], segment_ends[3]),
      color, thickness);
}

////////////////////////////////////////////////////////////////////////////////
//cut:drawPoint

inline void drawPoint(cv::Mat & img, const cv::Point & p,
                      const cv::Scalar & color, const int thickness = 1) {
  cv::line(img, p, p, color, thickness);
}

////////////////////////////////////////////////////////////////////////////////
//cut:drawRectangle

inline void drawRectangle(cv::Mat & img, const cv::Rect & r, const cv::Scalar & color,
                          const int thickness =1,
                          const int line_type =8,
                          const int shift =0) {
  cv::rectangle(img, cv::Point(r.x, r.y),
                cv::Point(r.x + r.width, r.y + r.height),
                color, thickness, line_type, shift);
}

////////////////////////////////////////////////////////////////////////////////
//cut:drawCross

inline void drawCross(cv::Mat & img, const int x, const int y, const int radius,
                      const cv::Scalar & color,  const int thickness =1,
                      const int line_type =8, const int shift =0) {
  cv::line(img, cv::Point(x - radius, y), cv::Point(x + radius, y), color, thickness,
           line_type, shift);
  cv::line(img, cv::Point(x, y - radius), cv::Point(x, y + radius), color, thickness,
           line_type, shift);
}

////////////////////////////////////////////////////////////////////////////////

inline void drawCross(cv::Mat & img, const cv::Point & p,
                      const int radius, const cv::Scalar & color,
                      const int thickness =1, const int line_type =8,
                      const int shift =0) {
  drawCross(img, p.x, p.y, radius, color, thickness, line_type, shift);
}

////////////////////////////////////////////////////////////////////////////////
//cut:drawPolygon

inline void drawPolygon(cv::Mat & img,
                        const std::vector<cv::Point> & poly,
                        bool is_closed,
                        const cv::Scalar & color,
                        const int thickness =1,
                        const int line_type = 8,
                        const int shift = 0) {
  if (poly.size() == 0)
    return;
  if (poly.size() == 1) {
    cv::circle(img, poly[0], thickness, color, -1, line_type, shift);
    return;
  }
  for (unsigned int pt_idx = 0; pt_idx < poly.size() - 1; ++pt_idx)
    cv::line(img, poly[pt_idx], poly[pt_idx + 1], color, thickness,line_type, shift);
  if (is_closed)
    cv::line(img, poly[poly.size() - 1], poly[0], color, thickness,line_type, shift);
}

////////////////////////////////////////////////////////////////////////////////
//cut:drawListOfPoints

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

////////////////////////////////////////////////////////////////////////////////
//cut:paste_img_compute_rois

/*!
 Determine the ROIs for pasting an image onto another.
 \param topaste
 \param dst
 \param topaste_x, topaste_y
    The coordinates where to paste "topaste" into "dst"
 \param topaste_roi (out)
   If width = 0, means "topaste" would be out of "dst" (= nothing to do).
   Otherwise, the ROI of "topaste" that needs to be copied.
 \param dst_roi (out)
   If width = 0, means "topaste" would be out of "dst" (= nothing to do).
   Otherwise, the ROI of "dst" where the ROI of "topaste" needs to be pasted.
*/
template<class Image>
inline void paste_img_compute_rois(const Image & topaste, const Image & dst,
                                   int topaste_x, int topaste_y,
                                   cv::Rect & topaste_roi, cv::Rect & dst_roi) {
  dst_roi = cv::Rect(0, 0, dst.cols, dst.rows);
  topaste_roi = cv::Rect(topaste_x, topaste_y, topaste.cols, topaste.rows);

  // sizes check
  if (topaste.cols == 0 || topaste.rows == 0 || dst.rows == 0 || dst.cols == 0) {
    dst_roi.width = dst_roi.height = topaste_roi.width = topaste_roi.height = 0;
    return;
  }

  cv::Rect inter_roi = rectangle_intersection(dst_roi, topaste_roi);
  // disjoint rectangles => do nothing
  if (inter_roi.width <= 0 || inter_roi.height <= 0) {
    dst_roi.width = dst_roi.height = topaste_roi.width = topaste_roi.height = 0;
    return;
  }
  // the corresponding width and height in input images will be the one of the inter
  dst_roi.width = inter_roi.width;
  dst_roi.height = inter_roi.height;
  topaste_roi.width = inter_roi.width;
  topaste_roi.height = inter_roi.height;
  // adjust x, depending on which side of the left edge of "dst" we paste
  if (topaste_x >= 0) { // we paste on the right side of the left edge of "dst"
    dst_roi.x = inter_roi.x;
    topaste_roi.x = 0;
  }
  else { // we paste on the left side of "dst"
    dst_roi.x = 0;
    topaste_roi.x = -topaste_x;
  }
  // same thing with y
  if (topaste_y >= 0) {
    dst_roi.y = inter_roi.y;
    topaste_roi.y = 0;
  }
  else {
    dst_roi.y = 0;
    topaste_roi.y = -topaste_y;
  }
} // end paste_img_compute_rois()

/*! copies to \arg dst
    matrix elements of \arg topaste that are
    marked with non-zero \arg mask elements. */
template<class Image>
inline void paste_img(const Image & topaste, Image & dst,
                      int topaste_x, int topaste_y,
                      const cv::Mat* mask = NULL,
                      const std::string title = "",
                      const cv::Scalar title_color = cv::Scalar(0, 0, 255)) {
  //  printf("paste_img()\n");

  cv::Rect topaste_roi, dst_roi;
  paste_img_compute_rois(topaste, dst, topaste_x, topaste_y, topaste_roi, dst_roi);
  //  printf("topaste_roi:(%ix%i)+(%ix%i), dst_roi:(%ix%i)+(%ix%i)\n",
  //         topaste_roi.x, topaste_roi.y, topaste_roi.width, topaste_roi.height,
  //         dst_roi.x, dst_roi.y, dst_roi.width, dst_roi.height);
  if (topaste_roi.width == 0 || topaste_roi.height == 0
      || dst_roi.width == 0 || dst_roi.height == 0)
    return;

  // make the proper pasting
  Image topaste_sub = topaste(topaste_roi),
      dst_sub = dst(dst_roi);
  bool use_mask = (mask != NULL);
  if (use_mask && !bbox_included_image(topaste_roi, *mask)) {
    printf("paste_img(): incorrect dims of mask (%i,%i), topaste_roi:%s\n",
           mask->cols, mask->rows, print_rect(topaste_roi).c_str());
    use_mask = false;
  }
  if (use_mask) {
    const cv::Mat mask_sub = (*mask)(topaste_roi);
    topaste_sub.copyTo(dst_sub, mask_sub);
  }
  else
    topaste_sub.copyTo(dst_sub);

  // put the title if needed
  if (title != "")
    cv::putText(dst, title,
                cv::Point(dst_roi.x + 5, dst_roi.y + dst_roi.height - 10),
                cv::FONT_HERSHEY_PLAIN, 1, title_color);
}

////////////////////////////////////////////////////////////////////////////////
//cut:draw_text_centered

/*!
 \param img
    the image where the text will be put
 \param text
    The text to be put
 \param org
    Where to put the text in img
 \param fontFace, fontScale, color, thickness, linetype
    \see cv::putText()
*/
inline void draw_text_centered(cv::Mat& img, const std::string& text,
                               const cv::Point & org,  const int & fontFace,
                               const double & fontScale, const cv::Scalar & color,
                               const int thickness=1, const int linetype=8) {
  int baseline = 0;
  cv::Size txt_size =
      cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  cv::putText(img, text,
              cv::Point(org.x - txt_size.width / 2,
                        org.y + txt_size.height / 2),
              fontFace, fontScale, color, thickness, linetype);
} // end draw_text_centered();

////////////////////////////////////////////////////////////////////////////////
//cut:putTextBackground

/*!
 * Put a text into an image with a given background color
 * \param img
 *    the image where the text will be put
 * \param text
 *    The text to be put
 * \param org
 *    Where to put the text in img
 * \param font_color
 *    the color of the text characters
 * \param bg_color
 *    the color behind the characters
 * \param padding
 *    the size in pixels of the margin between the bounding box of the text
 *    and the rectangle filled with background color
 * \param fontFace, fontScale, thickness, linetype, bottomLeftOrigin
    \see cv::putText()
 */
inline cv::Rect putTextBackground(cv::Mat& img, const std::string& text,
                                  cv::Point org, int fontFace, double fontScale,
                                  cv::Scalar font_color, cv::Scalar bg_color,
                                  int padding = 1,
                                  int thickness=1, int linetype=8,
                                  bool bottomLeftOrigin=false ) {
  // find size of text
  int baseline = 0;
  cv::Size txt_size =
      cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  // paint where text will be with given color
  cv::Rect text_roi(org.x - padding,
                    org.y - txt_size.height - padding,
                    txt_size.width + 2 * padding,
                    txt_size.height + 2 * padding);
  cv::rectangle(img, text_roi, bg_color, -1);
  // put final text
  cv::putText(img, text, org,
              fontFace, fontScale, font_color, thickness, linetype, bottomLeftOrigin);
  return text_roi;
}

////////////////////////////////////////////////////////////////////////////////
//cut:rotate_image

/*!
  Roate an image by a given angle.
 \param src
    The image to be rotated
 \param dst
    The resulting rotated image
 \param angle_rad
    The angle of rotation, in radians
 \param rotation_center
    The center of rotation. Must be in "src" bounds.
 \param flags, borderMode, borderValue
    \see cv::warpAffine()
    http://docs.opencv.org/modules/imgproc/doc/geometric_transformations.html?highlight=warpaffine#warpaffine
*/
inline void rotate_image(const cv::Mat & src, cv::Mat & dst,
                         const double & angle_rad, const cv::Point & rotation_center,
                         int flags=cv::INTER_LINEAR,
                         int borderMode=cv::BORDER_CONSTANT,
                         const cv::Scalar& borderValue=cv::Scalar()) {
  cv::Mat rot_mat = cv::getRotationMatrix2D
                    (rotation_center, angle_rad * RAD2DEG, 1.0);
  cv::warpAffine(src, dst, rot_mat, src.size(), flags, borderMode, borderValue);
} // end rotate_image();

////////////////////////////////////////////////////////////////////////////////
//cut:draw_text_centered

/*!
 \param img
    the image where the text will be put
 \param buffer1, buffer2
    Two buffers for rendering. Memory allocation is made inside the function.
    the horizontal text will be put in buffer1.
    buffer2 is the rotated version of buffer1
    (cv::warpAffine() can not operate in-place)
 \param text
    The text to be put
 \param org
    Where to put the text in img
 \param angle_rad
    The angle of rotation for the text, in radians.
 \param fontFace, fontScale, color, thickness, linetype
    \see cv::putText()
*/
inline void draw_text_rotated
(cv::Mat3b& img, cv::Mat1b& buffer1, cv::Mat1b& buffer2,
 const std::string& text,
 const cv::Point & org, const double & angle_rad,
 const int & fontFace, const double & fontScale, const cv::Scalar & color,
 const int thickness=1, const int linetype=8) {
  // do nothing if angle almost 0
  if (cos(angle_rad) > .995) {
    cv::putText(img, text, org, fontFace, fontScale, color, thickness, linetype);
    buffer1.create(1, 1);
    return;
  }

  // get size of text if not rotated
  int baseline = 0;
  cv::Size txt_size =
      cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);
  int buffer_dim = 2 * std::max(txt_size.width, txt_size.height);

  // create buffer if not big enough
  if (buffer1.cols < buffer_dim || buffer1.rows < buffer_dim) {
    //printf("resize\n");
    buffer1.create(buffer_dim, buffer_dim);
  }
  // buffer2 alloc made by rotate_image();

  // write text with no rotation in buffer
  cv::Point text_pos(buffer_dim / 2, buffer_dim / 2);
  buffer1 = 0;
  cv::putText(buffer1, text, text_pos, fontFace, fontScale,
              cv::Scalar::all(255), thickness, linetype);

  // find the ROI where buffer2 would be pasted
  cv::Rect topaste_roi, dst_roi;
  paste_img_compute_rois((cv::Mat) buffer1, (cv::Mat) img,
                                      org.x - text_pos.x, org.y - text_pos.y,
                                      topaste_roi, dst_roi);
  // do nothing if out of bounds
  if (topaste_roi.width == 0)
    return;

#if 1
  // rotate buffer1 into buffer2
  rotate_image(buffer1, buffer2, angle_rad, text_pos,
               cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar::all(0));
  // find a background color different from text color
  // cv::Vec3b bg_color = (color[0] == 0 ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0));

  // now paint img with color, use buffer2 as mask
  //  paste_img((cv::Mat) buffer3, img, org.x - text_pos.x, org.y - text_pos.y,
  //                         &buffer2);
  img(dst_roi).setTo(color, buffer2(topaste_roi));
#else
  // make a reverse lookup to know if pixels were activated
  IplImage buffer1_ipl = (IplImage) buffer1;
  double cos_a = cos(angle_rad), sin_a = sin(angle_rad);
  int min_col = topaste_roi.x, max_col = topaste_roi.x + topaste_roi.width;
  int min_row = topaste_roi.y, max_row = topaste_roi.y + topaste_roi.height;
  //  printf("min_col:%i, max_col:%i, min_row:%i, max_row:%i\n",
  //           min_col, max_col, min_row, max_row);
  // iterate on the window of buffer1 that needs to be pasted
  for (int row = min_row; row < max_row; ++row) {
    // get the address of row
    uchar* img_data = img.ptr<uchar>(dst_roi.y + row);
    for (int col = min_col; col < max_col; ++col) {
      // compute where would be the coordinates of the rotated pixel in buffer1
      int buffer1_col = text_pos.x
                        + ROTATE_COSSIN_X(col - text_pos.x, row - text_pos.y, cos_a, sin_a);
      int buffer1_row = text_pos.y
                        + ROTATE_COSSIN_Y(col - text_pos.x, row - text_pos.y, cos_a, sin_a);
      //  printf"col:%i, row:%i, buffer1_col:%i, buffer1_row:%i",
      //                    col, row, buffer1_col, buffer1_row);

      // if coordinates are inside bounds and buffer1 at this pixel is white,
      // paint it
      if (buffer1_col >= 0 && buffer1_col < buffer1.cols
          && buffer1_row >= 0 && buffer1_row < buffer1.rows
          && CV_IMAGE_ELEM(&buffer1_ipl, uchar, buffer1_row, buffer1_col) == 255) {
        img_data[3 * (dst_roi.x + col)    ] = color[0];
        img_data[3 * (dst_roi.x + col) + 1] = color[1];
        img_data[3 * (dst_roi.x + col) + 2] = color[2];
      }
    } // end loop col
  } // end loop row

#endif
} // end draw_text_rotated();

////////////////////////////////////////////////////////////////////////////////
//cut:paste_images

static const unsigned int HEADER_SIZE = 30;

inline cv::Rect paste_images_image_roi
(unsigned int img_idx,
 bool horizontal_pasting = true,
 const int width1 = 50,
 const int height1 = 50,
 bool draw_headers = true)
{
  return cv::Rect(
        width1 * (horizontal_pasting ? img_idx : 0),
        height1 * (horizontal_pasting ? 0 : img_idx),
        width1 + (!horizontal_pasting && draw_headers ? HEADER_SIZE : 0),
        height1 + (horizontal_pasting && draw_headers ? HEADER_SIZE : 0)
        );
} // end paste_images_image_roi()

////////////////////////////////////////////////////////////////////////////////

inline int paste_images_pixel_belong_to_image
(int pixel_x, int pixel_y,
 bool horizontal_pasting = true,
 const int width1 = 50,
 const int height1 = 50,
 bool draw_headers = true)
{
  int img_idx = 0;
  cv::Point pixel(pixel_x, pixel_y);
  while (img_idx < 200) { // foolproof
    cv::Rect img_roi = paste_images_image_roi
                       (img_idx, horizontal_pasting, width1, height1, draw_headers);
    if (img_roi.contains(pixel)) // pixel in ROI? we found it!
      return img_idx;
    if (img_roi.x > pixel_x || img_roi.y > pixel_y) // we went over the pixel
      return -1;
    ++img_idx;
  }
  // should not be reached - ever
  return -1;
} // end paste_images_pixel_belong_to_image()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Paste a list of images horizontally into a collage image.
 * \param imgs
 *  The series of input images
 * \param out
 *  The image where all images of "imgs" are pasted.
 * \param width1
 *  The wanted width of a column, in pixels.
 * \param height
 *  The wanted total height of "out", in pixels
 * \param colpadding
 *  The white space between columns, in pixels.
 *  Not taken into acount if "do_not_constrain_width" is true.
 * \param draw_headers
 *  true for drawing column headers
 * \param column_titlemap
 *  the function for convertnig column indices into header titles (strings)
 * \param masks
 *  if not empty, must be the same size than imgs.
 *  Contains the mask for each input image.
 * \param do_not_constrain_width
 *  if true, only constrain images height, and not width.
 *  Thus, pasted images can be wider than "width1" and overlap each other.
 */
template<class _T>
void paste_images(const std::vector<cv::Mat_<_T> > & imgs,
                  cv::Mat_<_T> & out,
                  bool horizontal_pasting = true,
                  const int width1 = 50,
                  const int height1 = 50,
                  const int itempadding = 5,
                  bool draw_headers = true,
                  Map column_titlemap = int_to_uppercase_letter,
                  const std::vector<cv::Mat1b> & masks = std::vector<cv::Mat1b>(),
                  bool do_not_constrain_width = false)
{
  //  printf("paste_images: %i imgs, width1:%i, height1:%i, draw_headers:%i, "
  //         "do_not_constrain_width:%i\n",
  //         imgs.size(), width1, height1, draw_headers, do_not_constrain_width);
  unsigned int nimgs = imgs.size();
  // fool-proof sizes
  if (nimgs == 0) {
    out = cv::Mat_<_T>(1, 1);
    return;
  }

  bool use_masks = (masks.size() > 0);
  if (use_masks && nimgs != masks.size()) {
    printf("paste_images(): Wrong number of masks: %i != %li\n", nimgs, masks.size());
    return;
  }

  // check size big enough
  int header_size = (draw_headers ? HEADER_SIZE : 0);
  int width_final = (horizontal_pasting ? width1 * nimgs : width1 + header_size);
  int height_final = (horizontal_pasting ? height1 + header_size : height1 * nimgs);
  if (out.cols < width_final || out.rows < height_final)
    out.create(height_final, width_final);
  int allowed_width1 = (do_not_constrain_width ?
                          1E5
                        : width1 - (horizontal_pasting ? itempadding: 0));
  int allowed_height1 = height1 - (horizontal_pasting ? 0: itempadding);

  // clear background
  out.setTo(255);

  // draw ROIS - DEBUG
  //for (uint img_idx = 0; img_idx < nimgs; ++img_idx)
  //  cv::rectangle(out, paste_images_image_roi(img_idx, horizontal_pasting, width1, height1, draw_headers),
  //                cv::Scalar::all(rand() % 200), -1);

  // paste all images
  for (unsigned int img_idx = 0; img_idx < nimgs; ++img_idx) {
    // resize image if bigger than allowed space
    cv::Mat_<_T> curr_img_resized;
    cv::Mat1b curr_mask_resized;
    resize_if_bigger
        (imgs[img_idx], curr_img_resized, allowed_width1, allowed_height1,
         cv::INTER_NEAREST, true, true);
    if (use_masks) {
      resize_if_bigger
          (masks[img_idx], curr_mask_resized, allowed_width1, allowed_height1,
           cv::INTER_NEAREST, true, false);
    }

    // cv::imshow("curr_img_resized", curr_img_resized); cv::waitKey(0);
    int paste_x = (width1 - curr_img_resized.cols) / 2
                  + (horizontal_pasting ? img_idx * width1 : header_size);
    int paste_y = (height1 - curr_img_resized.rows) / 2
                  + (horizontal_pasting ? 0 : img_idx * height1);
    paste_img(curr_img_resized, out, paste_x, paste_y,
                           (use_masks ? &curr_mask_resized : NULL));
    //    printf("curr_img_resized:%i x %i, paste_x:%i, paste_y:%i\n",
    //           curr_img_resized.cols, curr_img_resized.rows, paste_x, paste_y);
  } // end loop img_idx

  // draw headers
  if (draw_headers) {
    for (uint img_idx = 0; img_idx < nimgs; ++img_idx) {
      int header_x = (horizontal_pasting ? width1 * (.5 + img_idx)
                                         : header_size / 2);
      int header_y = (horizontal_pasting ? height1 + header_size / 2
                                         : height1 * (.5 + img_idx));
      draw_text_centered
          (out, column_titlemap(img_idx), cv::Point(header_x, header_y),
           CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 0, 0));
    } // end for img_idx
  } // end if (draw_headers)

} // end if paste_images();

////////////////////////////////////////////////////////////////////////////////
//cut:paste_images_gallery

/*!
 * Paste a bunch of images OF THE SAME SIZE into a big collage,
 * with configurable number of images per row.
 * \param in
 *    the input images, of the same size
 * \param out
 *    the collage image.
 * \param gallerycols
 *    the desired number of images per row
 * \param background_color
 *    the color of the background, visible if the last row is not complete for instance
 * \param draw_borders
 *    true for drawing a rectangular border around each pic
 * \param border_color
 *    the color for the border. Only used if \arg draw_borders = true.
 */
template<class T>
void paste_images_gallery(const std::vector<cv::Mat_<T> > & in,
                          cv::Mat_<T> & out,
                          int gallerycols, T background_color,
                          bool draw_borders = false, cv::Scalar border_color = CV_RGB(0,0,0)) {
  if (in.size() == 0) {
    out.create(0, 0);
    return;
  }
  int cols1 = in[0].cols, rows1 = in[0].rows, nimgs = in.size();
  // prepare output
  int galleryrows = std::ceil(1. * nimgs / gallerycols);
  out.create(rows1 * galleryrows, cols1 * gallerycols);
  //printf("nimgs:%i, gallerycols:%i, galleryrows:%i\n", nimgs, gallerycols, galleryrows);
  out.setTo(background_color);
  // paste images
  for (int img_idx = 0; img_idx < nimgs; ++img_idx) {
    int galleryx = img_idx % gallerycols, galleryy = img_idx / gallerycols;
    cv::Rect roi(galleryx * cols1, galleryy * rows1, cols1, rows1);
    //printf("### out:%ix%i, roi %i:'%s'\n", out.cols, out.rows, img_idx, print_rect(roi).c_str());
    if (cols1 != in[img_idx].cols || rows1 != in[img_idx].rows) {
      printf("Image %i of size (%ix%i), different from (%ix%i), skipping it.\n",
             img_idx, in[img_idx].cols, in[img_idx].rows, cols1, rows1);
      cv::line(out, roi.tl(), roi.br(), border_color, 2);
      cv::line(out, roi.br(), roi.tl(), border_color, 2);
    }
    else
      in[img_idx].copyTo( out(roi) );
    if (draw_borders)
      cv::rectangle(out, roi, border_color, 1);
  } // end loop img_idx
} // end paste_pics<_T>

////////////////////////////////////////////////////////////////////////////////
// cut

} // end namespace vision_utils

#endif // DRAWING_UTILS_H
