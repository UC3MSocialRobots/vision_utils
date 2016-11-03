/*!
  \file        depth_image_to_vizualisation_color_image.h
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

#ifndef DEPTH_IMAGE_TO_VIZUALISATION_COLOR_IMAGE_H
#define DEPTH_IMAGE_TO_VIZUALISATION_COLOR_IMAGE_H
// std includes
#include <algorithm> // for std::min(), std::max()...
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h> // for printf(), etc
#include <vector>
// vision_utils
#include "vision_utils/convert_float_to_uchar.h"
#include "vision_utils/hue2rgb_make_lookup_table.h"
#include "vision_utils/is_nan_depth.h"
#include "vision_utils/min_max.h"

namespace vision_utils {

/*!
 * convert a meter distance into something into [0; 255].
 * Ideally, the max value of the sensor (+- 10m) should correspond to 255.
 */
static const int METER2UCHAR_FACTOR = 20;

//! the different color modes
enum DepthViewerColorMode {
  // these modes multiply the depth value (in meter) by METER2UCHAR_FACTOR
  // to send it into a [0, 255] range.
  GREYSCALE_SCALED = 0, REDSCALE_SCALED = 1, FULL_RGB_SCALED = 2,
  // these modes find the min and the max of the depth image,
  // and match the range [min, max] -> [0, 255]
  // the contrast is thus maximised,
  // but they are slightly slower
  GREYSCALE_STRETCHED = 3, REDSCALE_STRETCHED = 4, FULL_RGB_STRETCHED = 5
};
//! the number of color modes. Must be updated if some new are implemented.
static const unsigned int DEPTH_VIEWER_COLOR_NMODES = 6;

/*!
 Convert a float image to a color image for viewing
 \param float_in
    The floating image
 \param uchar_greyscale_buffer
    A greyscale buffer
 \param uchar_rgb_out
    The output image, same size as float_in
 \param mode
    The wanted vizualisation mode, between
    GREYSCALE_SCALED, REDSCALE_SCALED, FULL_RGB_SCALED,
    GREYSCALE_STRETCHED, REDSCALE_STRETCHED, FULL_RGB_STRETCHED
 \param min_value, max_value
    For SCALED modes, the min and max values in meters of the sensor
    (can be actually smaller than the sensor for putting more contrast for a
     zone of interest, for instance 3 to 5 meters)
*/
inline void depth_image_to_vizualisation_color_image
(const cv::Mat & float_in,
 cv::Mat3b & uchar_rgb_out,
 const DepthViewerColorMode mode = FULL_RGB_STRETCHED,
 float min_value = 0, float max_value = 10) {
  unsigned int ncols = float_in.cols, nrows = float_in.rows;
  if (mode == GREYSCALE_SCALED) {
    float a = 255. / (max_value - min_value), b = -a * min_value;
    // one line conversion (slow)
    //float_in.convertTo(uchar_greyscale_buffer, CV_8U, 20, 0);
    //cv::cvtColor(uchar_greyscale_buffer, uchar_rgb_out, CV_GRAY2BGR);
    // iterate (faster)
    uchar_rgb_out.create(nrows, ncols);
    for (unsigned int row = 0; row < nrows; ++row) {
      const float* float_img_ptr = float_in.ptr<float>(row);
      cv::Vec3b* out_ptr = uchar_rgb_out.ptr<cv::Vec3b>(row);
      for (unsigned int col = 0; col < ncols; ++col) {
        //if (float_img_ptr[col] == NAN_DEPTH)
        if (is_nan_depth(float_img_ptr[col]))
          (out_ptr[col])[0] = (out_ptr[col])[1] = (out_ptr[col])[2] = 0;
        else
          (out_ptr[col])[0] = (out_ptr[col])[1] = (out_ptr[col])[2] =
            std::max(0, std::min((int) (float_img_ptr[col] * a + b), 255));
      } // end loop col
    } // end loop row
    return;
  } // end if (mode == GREYSCALE_SCALED)

  if (mode == REDSCALE_SCALED) {
    float a = 255. / (max_value - min_value), b = -a * min_value;
    uchar_rgb_out.create(nrows, ncols);
    // uchar_rgb_out.setTo(0);
    for (unsigned int row = 0; row < nrows; ++row) {
      const float* float_img_ptr = float_in.ptr<float>(row);
      cv::Vec3b* out_ptr = uchar_rgb_out.ptr<cv::Vec3b>(row);
      for (unsigned int col = 0; col < ncols; ++col) {
        (out_ptr[col])[0] = (out_ptr[col])[1] = 0; // B, G
        //if (float_img_ptr[col] == NAN_DEPTH)
        if (is_nan_depth(float_img_ptr[col]))
          (out_ptr[col])[2] = 0; // R
        else
          (out_ptr[col])[2] = // R
                              std::max(0, std::min((int) (float_img_ptr[col] * a + b), 255));
      } // end loop col
    } // end loop row
    return;
  } // end if (mode == REDSCALE_SCALED)

  if (mode == FULL_RGB_SCALED) {
    float a = 255. / (max_value - min_value), b = -a * min_value;
    static std::vector<cv::Vec3b> hue_lut;
    hue2rgb_make_lookup_table(hue_lut, 256);
    uchar_rgb_out.create(nrows, ncols);
    for (unsigned int row = 0; row < nrows; ++row) {
      const float* float_img_ptr = float_in.ptr<float>(row);
      cv::Vec3b* out_ptr = uchar_rgb_out.ptr<cv::Vec3b>(row);
      for (unsigned int col = 0; col < ncols; ++col) {
        //if (float_img_ptr[col] == NAN_DEPTH)
        if (is_nan_depth(float_img_ptr[col]))
          (out_ptr[col])[0] = (out_ptr[col])[1] = (out_ptr[col])[2] = 0;
        else
          out_ptr[col] =
              hue_lut[std::max(0, std::min((int) (float_img_ptr[col] * a + b), 255))];
        //if (rand()%100000 == 0) printf("value:%g->%i\n", float_img_ptr[col],(int) (float_img_ptr[col] * a + b));
      } // end loop col
    } // end loop row
    return;
  } // end if (mode == FULL_RGB_SCALED)

  float minVal, maxVal;
  min_max_loc_nans(float_in, minVal, maxVal, NAN_DEPTH);
  //cv::minMaxLoc(float_in, &minVal, &maxVal);
  //printf("minVal:%g, maxVal:%g\n", minVal, maxVal);

  ScaleFactorType alpha_trans, beta_trans;
  compute_alpha_beta(minVal, maxVal, alpha_trans, beta_trans);
  if (mode == GREYSCALE_STRETCHED) {
    //  cv::convertScaleAbs(float_in, uchar_greyscale_buffer, alpha_trans, beta_trans);
    //  cv::cvtColor(uchar_greyscale_buffer, uchar_rgb_out, cv::COLOR_GRAY2RGB);
    uchar_rgb_out.create(nrows, ncols);
    for (unsigned int row = 0; row < nrows; ++row) {
      const float* float_img_ptr = float_in.ptr<float>(row);
      cv::Vec3b* out_ptr = uchar_rgb_out.ptr<cv::Vec3b>(row);
      for (unsigned int col = 0; col < ncols; ++col) {
        //if (float_img_ptr[col] != NAN_DEPTH) {
        (out_ptr[col])[0] = (out_ptr[col])[1] = (out_ptr[col])[2] =
            dist_to_image_val(float_img_ptr[col], alpha_trans, beta_trans);
        //} // end if not NAN_UCHAR
      } // end loop col
    } // end loop row
  } // end if GREYSCALE_STRETCHED

  else if (mode == REDSCALE_STRETCHED) {
    uchar_rgb_out.create(nrows, ncols);
    uchar_rgb_out.setTo(0);
    for (unsigned int row = 0; row < nrows; ++row) {
      const float* float_img_ptr = float_in.ptr<float>(row);
      cv::Vec3b* out_ptr = uchar_rgb_out.ptr<cv::Vec3b>(row);
      for (unsigned int col = 0; col < ncols; ++col) {
        // change red channel
        //if (float_img_ptr[col] != NAN_DEPTH) {
        (out_ptr[col])[2] = dist_to_image_val
                            (float_img_ptr[col], alpha_trans, beta_trans);
        //} // end if not NAN_UCHAR
      } // end loop col
    } // end loop row
  } // end if REDSCALE_STRETCHED

  else /*if (mode == FULL_RGB_STRETCHED)*/ {
    static std::vector<cv::Vec3b> hue_lut;
    hue2rgb_make_lookup_table(hue_lut, 256);
    uchar_rgb_out.create(nrows, ncols);
    uchar_rgb_out.setTo(0);
    for (unsigned int row = 0; row < nrows; ++row) {
      const float* float_img_ptr = float_in.ptr<float>(row);
      cv::Vec3b* out_ptr = uchar_rgb_out.ptr<cv::Vec3b>(row);
      for (unsigned int col = 0; col < ncols; ++col) {
        uchar val = dist_to_image_val(float_img_ptr[col], alpha_trans, beta_trans);
        if (val != NAN_UCHAR)
          out_ptr[col] = hue_lut[val];
      } // end loop col
    } // end loop row
  } // end if FULL_RGB_STRETCHED
} // end depth_image_to_vizualisation_color_image

////////////////////////////////////////////////////////////////////////////////

//! a short version for the other depth_image_to_vizualisation_color_image()
inline cv::Mat3b depth_image_to_vizualisation_color_image
(const cv::Mat & float_in,
 const DepthViewerColorMode mode = FULL_RGB_STRETCHED,
 double scale = 1) {
  cv::Mat3b float_out_color;
  depth_image_to_vizualisation_color_image
      (float_in, float_out_color, mode);
  if (scale == 1)
    return float_out_color;
  cv::resize(float_out_color, float_out_color, cv::Size(), scale, scale, cv::INTER_NEAREST);
  return float_out_color;
}

//! a short alias
inline cv::Mat3b depth2viz
(const cv::Mat & float_in, const DepthViewerColorMode mode = FULL_RGB_STRETCHED, double scale = 1) {
  return depth_image_to_vizualisation_color_image(float_in, mode, scale);
}

} // end namespace vision_utils

#endif // DEPTH_IMAGE_TO_VIZUALISATION_COLOR_IMAGE_H
