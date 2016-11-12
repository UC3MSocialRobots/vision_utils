/*!
  \file        propagative_floodfill.h
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

#ifndef PROPAGATIVE_FLOODFILL_H
#define PROPAGATIVE_FLOODFILL_H
// vision_utils
#include <vision_utils/find_top_point_centered.h>
#include <vision_utils/depth_image_to_vizualisation_color_image.h>
// std includes
#include <opencv2/core/core.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>
#include <queue>

namespace vision_utils {

typedef short BufferElem;
//#define PROPAGATE_FLOODFILL_USE_STDVEC

#ifdef PROPAGATE_FLOODFILL_USE_STDVEC
#define SEEN_BUFFER_TYPE std::vector<short>
#define SEEN_BUFFER_ACCESS(row, col) queued_short[(row) * cols + (col)]
#else // no CMatrix
#define SEEN_BUFFER_TYPE cv::Mat_<short>
#define SEEN_BUFFER_ACCESS(row, col) queued_short_data[(row) * cols + (col)]
#endif // PROPAGATE_FLOODFILL_USE_STDVEC

/*!
 * Make a propagative floodfill from a given seed
 * and store the distances in a buffer.
 * \param img
 *  Where to make the floodfill from.
 *  The propatation will be done on non-zero pixels (like a mask).
 * \param seed
 *  Where to start the propagation from.
 * \param queued_short
 *  Where we store the propagation distance in pixel:
 *  the value at (x, y) is the length of the shortest C1 path from \a seed to (x, y).
 * \param search_top_point_centered
 *  if true, instead of using \a seed, use the point obtained by
 *  calling find_top_point_centered(img, seed)
 * \param lookup_function, lookup_result, lookup_cookie
 *  optionnaly, a lookup function can be called.
 *  For each pixel of the propagation , it converts the distance from the seed,
 *  in pixels, into whatever you want.
 *  The result of this lookup is then stored into \a lookup_result.
 *  Optionnaly, some data can be passed to \a lookup_function()
 *  thanks to the \a lookup_cookie.
 */
void propagative_floodfill(const cv::Mat1b & img,
                           cv::Point & seed,
                           SEEN_BUFFER_TYPE & queued_short,
                           bool search_top_point_centered = false,
                           float (*lookup_function)
                           (const int propagation_pixel_dist, const int row, const int col, void* cookie) = NULL,
                           cv::Mat1f* lookup_result = NULL,
                           void* lookup_cookie = NULL,
                           bool propagate_along_diagonals = true) {
  if (!img.isContinuous()) {
    printf("floodfill() only works with continous images!\n");
    return;
  }
  IplImage img_ipl = img;
  int cols = img.cols, rows = img.rows;
  int max_col = cols - 1, max_row = rows - 1;

  // search for top point if wanted
  // cv::Point seed = seed;
  if (search_top_point_centered)
    seed = find_top_point_centered(img, .4);

  // will we call the lookup_function
  bool want_lookup_function = (lookup_function != NULL);
  IplImage lookup_result_ipl;
  if (want_lookup_function) {
    lookup_result->create(img.size());
    lookup_result->setTo(0);
    lookup_result_ipl = *lookup_result;
  }

  // init queue with seed
  std::queue<int> queue_row, queue_col;
  queue_col.push(seed.x);
  queue_row.push(seed.y);
  // set the image as unseen except in seed
#ifdef PROPAGATE_FLOODFILL_USE_STDVEC
  queued_short.resize(rows * cols);
  std::fill(queued_short.begin(), queued_short.end(), 0);
#else // no CMatrix
  queued_short.create(img.size());
  queued_short.setTo(0);
  BufferElem* queued_short_data = queued_short.ptr<BufferElem>(0);
#endif // PROPAGATE_FLOODFILL_USE_STDVEC
  SEEN_BUFFER_ACCESS(seed.y, seed.x) = 1;

  while (queue_row.size() > 0) {
    // get next point in queue
    int curr_row = queue_row.front();
    queue_row.pop();
    int curr_col = queue_col.front();
    queue_col.pop();
    BufferElem curr_buff_val = SEEN_BUFFER_ACCESS(curr_row, curr_col);
    // lookup
    if (want_lookup_function) {
      CV_IMAGE_ELEM(&lookup_result_ipl, float, curr_row, curr_col) =
          lookup_function(curr_buff_val, curr_row, curr_col, lookup_cookie);
    } // end if (want_lookup_function)

    //printf("queue front:(%i, %i), curr_buff_val:%i\n", curr_col, curr_row, curr_buff_val);

    // check each of the neigbours
    // up
    if (curr_row > 0
        && SEEN_BUFFER_ACCESS(curr_row - 1, curr_col) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row - 1, curr_col) != 0) {
      SEEN_BUFFER_ACCESS(curr_row - 1, curr_col) = curr_buff_val + 1;
      queue_row.push(curr_row - 1);
      queue_col.push(curr_col);
    }
    // down
    if (curr_row < max_row
        && SEEN_BUFFER_ACCESS(curr_row + 1, curr_col) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row + 1, curr_col) != 0) {
      SEEN_BUFFER_ACCESS(curr_row + 1, curr_col) = curr_buff_val + 1;
      queue_row.push(curr_row + 1);
      queue_col.push(curr_col);
    }
    // left
    if (curr_col > 0
        && SEEN_BUFFER_ACCESS(curr_row, curr_col - 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row, curr_col - 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row, curr_col - 1) = curr_buff_val + 1;
      queue_row.push(curr_row);
      queue_col.push(curr_col - 1);
    }
    // right
    if (curr_col < max_col
        && SEEN_BUFFER_ACCESS(curr_row, curr_col + 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row, curr_col + 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row, curr_col + 1) = curr_buff_val + 1;
      queue_row.push(curr_row);
      queue_col.push(curr_col + 1);
    }
    if (!propagate_along_diagonals)
      continue;
    // up left
    if (curr_row > 0
        && curr_col > 0
        && SEEN_BUFFER_ACCESS(curr_row - 1, curr_col - 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row - 1, curr_col - 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row - 1, curr_col - 1) = curr_buff_val + 1;
      queue_row.push(curr_row - 1);
      queue_col.push(curr_col - 1);
    }
    // up right
    if (curr_row > 0
        && curr_col < max_col
        && SEEN_BUFFER_ACCESS(curr_row - 1, curr_col + 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row - 1, curr_col + 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row - 1, curr_col + 1) = curr_buff_val + 1;
      queue_row.push(curr_row - 1);
      queue_col.push(curr_col + 1);
    }
    // down left
    if (curr_row < max_row
        && curr_col > 0
        && SEEN_BUFFER_ACCESS(curr_row + 1, curr_col - 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row + 1, curr_col - 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row + 1, curr_col - 1) = curr_buff_val + 1;
      queue_row.push(curr_row + 1);
      queue_col.push(curr_col - 1);
    }
    // down right
    if (curr_row < max_row
        && curr_col < max_col
        && SEEN_BUFFER_ACCESS(curr_row + 1, curr_col + 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row + 1, curr_col + 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row + 1, curr_col + 1) = curr_buff_val + 1;
      queue_row.push(curr_row + 1);
      queue_col.push(curr_col + 1);
    }
  } // end while queue not empty
} // end propagative_floodfill();

////////////////////////////////////////////////////////////////////////////////

/*!
 * Convert the queued modified by propagative_floodfill()
 * into a neat image for debugging.
 * \param queued_short
 *   the buffer, once modified by propagative_floodfill()
 * \param queued_float_buffer
 *   a buffer. It will contain the same values as queued_short,
 *   but as floats
 * \param greyscale_buffer
 *   Needed for depth_image_to_vizualisation_color_image()
 * \param queued_illus
 *   The output image.
 */
void propagative_floodfill_seen_buffer_to_viz_image(const SEEN_BUFFER_TYPE & queued_short,
                                                    cv::Mat1f & queued_float_buffer,
                                                    cv::Mat3b & queued_illus) {
#ifdef PROPAGATE_FLOODFILL_USE_STDVEC
  // Mat_(int _rows, int _cols, _Tp* _data, size_t _step=AUTO_STEP);
  cv::Mat1s queued_short_cv(img.rows, img.cols, queued_short.data());
  queued_float_buffer.convertTo(queued_float, CV_32F);
#else // no CMatrix
  queued_short.convertTo(queued_float_buffer, CV_32F);
#endif // PROPAGATE_FLOODFILL_USE_STDVEC
  // convert seen buffer into a visualisation image
  depth_image_to_vizualisation_color_image
      (queued_float_buffer, queued_illus, FULL_RGB_STRETCHED);
} // end propagative_floodfill_seen_buffer_to_viz_image()

} // end namespace vision_utils

#endif // PROPAGATIVE_FLOODFILL_H
