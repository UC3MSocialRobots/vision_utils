/*!
 * compressed_rounded_image_transport is an image_transport plugin for float images.
 * \author Arnaud Ramey ( arnaud.a.ramey@gmail.com )
            -- Robotics Lab, University Carlos III of Madrid
 * \date Jan. 2012
  */

#ifndef CV_CONVERSION_FLOAT_UCHAR_H
#define CV_CONVERSION_FLOAT_UCHAR_H

#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include "hue_utils.h"
//#include "time/timer.h"
#include "nan_handling.h"
#include "min_max.h"

namespace image_utils {

typedef double ScaleFactorType;

////////////////////////////////////////////////////////////////////////////////

/*! from http://www.cs.rit.edu/~ncs/color/t_convert.html#RGB%20to%20HSV%20&%20HSV%20to%20RGB
  \param The hue value H runs from 0 to 360º.
  \param The saturation S is the degree of strength or purity and is from 0 to 1.
  Purity is how much white is added to the color, so S=1 makes the purest color (no white).
  \param Brightness V also ranges from 0 to 1, where 0 is the black.
*/
template<class Float>
inline void HSVtoRGB( const Float h, const Float s, const Float v,
                      Float & r, Float & g, Float & b) {
  if( s == 0 ) {
    // achromatic (grey)
    r = g = b = v;
    return;
  }
  float h2 = h / 60;   // sector 0 to 5
  int i = floor( h2 );
  float f = h2 - i;   // factorial part of h
  float p = v * ( 1 - s );
  float q = v * ( 1 - s * f );
  float t = v * ( 1 - s * ( 1 - f ) );
  switch( i ) {
  case 0:
    r = v; g = t; b = p;
    break;
  case 1:
    r = q; g = v; b = p;
    break;
  case 2:
    r = p; g = v; b = t;
    break;
  case 3:
    r = p; g = q; b = v;
    break;
  case 4:
    r = t; g = p; b = v;
    break;
  default:  // case 5:
    r = v; g = p; b = q;
    break;
  } // end switch i
} // end HSVtoRGB

/*!
 * \brief   color conversion function
 * \param hue
 *  in [0..180] (the way it comes with OpenCV hue conversion)
 *  traditionally, hue is in [0..360], so please divide it by 2
   */
inline cv::Vec3b hue2rgb(const unsigned char hue) {
  float r, g, b, h_filter = (hue >= 179 ? 179 : hue);
  HSVtoRGB(h_filter * 2.f, 1.f, 1.f, r, g, b);
  return cv::Vec3b(b * 255, g * 255, r * 255);
}

inline void hue2rgb_make_lookup_table(std::vector<cv::Vec3b> & lut,
                                      const unsigned int lut_size = 255) {
  lut.reserve(lut_size);
  lut.clear();
  for (unsigned int hue_idx = 0; hue_idx < lut_size; ++hue_idx) {
    unsigned char hue =  hue_idx * 180.f / lut_size;
    lut.push_back(hue2rgb(hue));
  }
} // end hue2rgb_make_lookup_table()

////////////////////////////////////////////////////////////////////////////////

/*! compute the transform :
   dest(i, j) = alpha * src(i, j) + beta
   find alpha, beta.
   NAN_VALUE = 0, so we want
   { min_cv_img_reshape -> 1
   { max_cv_img_reshape -> 255
   \arg alpha, beta
      computed values
*/
inline void compute_alpha_beta(const double & minVal, const double & maxVal,
                               ScaleFactorType & alpha, ScaleFactorType & beta) {
  if (maxVal == minVal)
    alpha = 1;
  else
    alpha = 254 / (maxVal - minVal);
  beta = 255 - alpha * maxVal;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param distance
    the value of the float image
 \param alpha_trans
    the scaling factor, obtained with convert_float_to_uchar()
 \param beta_trans
    the offset factor, obtained with convert_float_to_uchar()
 \return uchar
    the converted value, as an uchar
*/
inline uchar dist_to_image_val(const float & distance,
                               const ScaleFactorType & alpha_trans,
                               const ScaleFactorType & beta_trans) {
  return (is_nan_depth(distance) ?
            NAN_UCHAR
          : cv::saturate_cast<uchar>(alpha_trans * distance + beta_trans));
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param val
    the uchar value, as an uchar
 \param alpha_trans
    the scaling factor, obtained with convert_float_to_uchar()
 \param beta_trans
    the offset factor, obtained with convert_float_to_uchar()
 \return float
    the value of the float image
*/
inline float image_val_to_dist(const uchar & val,
                               const ScaleFactorType & alpha_trans,
                               const ScaleFactorType & beta_trans) {
  return (val == NAN_UCHAR ? NAN_DEPTH
                           : 1.f * (val - beta_trans) / alpha_trans);
}

////////////////////////////////////////////////////////////////////////////////

/*!
  Compresses a float matrix to a uchar one.
 \param src_float
    a float matrix, any number of channels <= 4
 \param dst_uchar (out)
    where the converted uchar matrix will be stored
 \param alpha_trans (out)
    the scaling factor
 \param beta_trans (out)
    the offset factor
 \param src_nan_indices (out)
    will contain the indice of all the points that are equal to NAN.
    It is sorted from smaller to bigger.
    It can be useful if the image is compressed in a lossy way afterwoards.
*/
inline void convert_float_to_uchar(const cv::Mat & src_float, cv::Mat & dst_uchar,
                                   cv::Mat & src_float_clean_buffer,
                                   ScaleFactorType & alpha_trans,
                                   ScaleFactorType & beta_trans,
                                   std::vector<unsigned int>* src_nan_indices = NULL) {
  // Timer timer;
  dst_uchar.create(src_float.size(), CV_8UC(src_float.channels()));
  // timer.printTime("create");

  bool store_indices = (src_nan_indices != NULL);
  if (store_indices) { // clear and reserve some big space
    src_nan_indices->clear();
    src_nan_indices->reserve((src_float.cols * src_float.rows * src_float.channels()) / 3);
  }
  cv::Mat* src_float_clean_ptr;

  // find the max value
  float minVal, maxVal;
#if 1 // find at the same time minVal, maxVal and clean NaNs
  // cv::Mat src_float_clean_buffer = src_float.clone();
  src_float.copyTo(src_float_clean_buffer);
  remove_nans_and_minmax<float>(src_float_clean_buffer, minVal, maxVal, NAN_DEPTH);
  src_float_clean_ptr = &src_float_clean_buffer;

#elif 1 // clean NaNs, then find minVal, maxVal with minmax_nans
  // cv::Mat src_float_clean_buffer = src_float.clone();
  src_float.copyTo(src_float_clean_buffer);
  remove_nans<float>(src_float_clean_buffer, NAN_DEPTH);
  min_max_loc_nans(src_float_clean_buffer, minVal, maxVal, NAN_DEPTH);
  src_float_clean_ptr = &src_float_clean_buffer;

#elif 1
  // cv::minMaxLoc() does not work with multi-channel arrays.
  // If you need to find minimum or maximum elements across all the channels,
  // use reshape() first to reinterpret the array as single-channel.
  // reshape(int cn, int rows=0) const :
  //    Changes the 2D matrix’s shape and/or the number of channels without copying the data.
  cv::Mat src_float_reshape = src_float.reshape(1);
  min_max_loc(src_float_reshape, minVal, maxVal);
  src_float_clean_ptr = &src_float; // no cleaning

#elif 1
  // cv::minMaxLoc() does not work with multi-channel arrays.
  // If you need to find minimum or maximum elements across all the channels,
  // use reshape() first to reinterpret the array as single-channel.
  // reshape(int cn, int rows=0) const :
  //    Changes the 2D matrix’s shape and/or the number of channels without copying the data.
  cv::Mat src_float_reshape = src_float.reshape(1);
  // timer.printTime("reshape");
  double minVal_double, maxVal_double;
  cv::minMaxLoc(src_float_reshape, &minVal_double, &maxVal_double);
  minVal = minVal_double;
  maxVal = maxVal_double;
  src_float_clean_ptr = &src_float; // no cleaning

#else // find at the same time minVal, maxVal and store NaNs
  boost::unordered_set<unsigned int> src_nan_indices_set;
  store_nans_and_minmax_set<float>(src_float, minVal, maxVal, src_nan_indices_set);
  src_float_clean_ptr = &src_float; // no cleaning
#endif

  // printf("minVal:%g, maxVal:%g\n", minVal, maxVal);
  // timer.printTime("min_max_loc");
  compute_alpha_beta(minVal, maxVal, alpha_trans, beta_trans);
  // timer.printTime("compute_alpha_beta");

  // convert the image
#if 1 // simple iterations
  //  int rows = dst_uchar.rows;
  //  int values_per_row = dst_uchar.cols * dst_uchar.channels();
  int rows = src_float_clean_ptr->rows;
  int values_per_row = src_float_clean_ptr->cols * src_float_clean_ptr->channels();
  if (src_float_clean_ptr->isContinuous() && dst_uchar.isContinuous()) {
    rows = 1;
    values_per_row =src_float_clean_ptr->rows * src_float_clean_ptr->cols * src_float_clean_ptr->channels();
  }
  for (int row = 0; row < rows; ++row) {
    // get the address of row
    const float* src_ptr = src_float_clean_ptr->ptr<float>(row);
    uchar* dst_ptr = dst_uchar.ptr<uchar>(row);
    // change each value
    for (int col = 0; col < values_per_row; ++col) {
#if 0 // use src_nan_indices_set
      if (src_nan_indices_set.count(col + row * values_per_row))
        *dst_ptr = NAN_UCHAR;
      else
#endif
        // convert float distance to uchar
        *dst_ptr = dist_to_image_val(*src_ptr, alpha_trans, beta_trans);
      // store the indice if we found a NaN
      if (store_indices && *dst_ptr == NAN_UCHAR)
        src_nan_indices->push_back(col + row * values_per_row);
      ++src_ptr;
      ++dst_ptr;
    } // end loop col
  } // end loop row

#elif 1 // use the evil ptr<>() and iterate, does not work for non-continuous?
  // IplImage src_float_ipl = src_float, dst_uchar_ipl = dst_uchar;
  int nvals = dst_uchar.cols * dst_uchar.rows * dst_uchar.channels();
  const float* src_ptr = src_float_clean_ptr->ptr<float>();
  uchar* dst_ptr = dst_uchar.ptr<uchar>();
  for (int col = 0; col < nvals; ++col) {
    // convert float distance to uchar
    *dst_ptr = dist_to_image_val(*src_ptr, alpha_trans, beta_trans);
    // store the indice if we found a NaN
    if (store_indices && *dst_ptr == NAN_UCHAR)
      src_nan_indices->push_back(col);
    ++src_ptr;
    ++dst_ptr;
  } // end loop col

#elif 1
  cv::convertScaleAbs(*src_float_clean_buffer, dst_uchar, alpha_trans, beta_trans);

#elif 1
  // a bit faster than cv::convertScaleAbs()
  dst_uchar = alpha_trans * (*src_float_clean_buffer) + beta_trans;

#elif 1
  // use CV convertTo() -> very slow!
  cv::Mat dst_uchar_reshape;
  src_float_reshape.convertTo(dst_uchar_reshape, CV_8U, alpha_trans, beta_trans);
  // cv::Mat mask_reshape = (src_float_reshape == NAN_DEPTH);
  //  printf("mask_reshape: %i x %i (%i channels, type:%i, step1:%i)\n",
  //         mask_reshape.cols, mask_reshape.rows, mask_reshape.channels(), mask_reshape.type(), mask_reshape.step1());
  dst_uchar_reshape.setTo(NAN_UCHAR, /*mask_reshape*/ (src_float_reshape == NAN_DEPTH));
  dst_uchar = dst_uchar_reshape.reshape(src_float.channels());
#endif
  // timer.printTime("after cv::convertScaleAbs()");
} // end convert_float_to_uchar();

//////////////////////////////////////////////////////////////////////////////


/*!
  Restores the compressed rounded image to the approximate float image
 \param src_uchar
    the compressed image
 \param dst_float (out)
    the approximate float image
 \param alpha_trans
    the scaling factor, obtained with convert_float_to_uchar()
 \param beta_trans
    the offset factor, obtained with convert_float_to_uchar()
 \param src_nan_indices
    should contain the indice of all the points that are equal to NaN.
    It should be sorted from smaller to bigger.
    It can be useful to restore the NaNs
    if the image was compressed in a lossy way afterwoards.
*/
inline void convert_uchar_to_float(const cv::Mat & src_uchar, cv::Mat & dst_float,
                                   const ScaleFactorType & alpha_trans,
                                   const ScaleFactorType & beta_trans,
                                   const std::vector<unsigned int>* src_nan_indices = NULL) {
  dst_float.create(src_uchar.size(), CV_32FC(src_uchar.channels()));
  // parameters for NaN restoring
  bool src_nan_indices_given = (src_nan_indices != NULL);
  unsigned int src_nan_indices_pos = 0, curr_pos_total = 0;
  const unsigned int* src_nan_current_indice =
      (src_nan_indices_given ? &(*src_nan_indices)[0] : NULL);
  //ROS_INFO("src_nan_indices_given:%i", src_nan_indices_given);

  // make a lookup table for faster conversion
  float lookup_table[256];
  for (int col = 0; col <= 255; ++col)
    lookup_table[col] = image_val_to_dist(col, alpha_trans, beta_trans);

  // convert the image
  // int values_per_row = src_uchar.cols * src_uchar.channels();
  int rows = src_uchar.rows;
  int values_per_row = src_uchar.cols * src_uchar.channels();
  if (src_uchar.isContinuous() && dst_float.isContinuous()) {
    rows = 1;
    values_per_row =src_uchar.rows * src_uchar.cols * src_uchar.channels();
  }
  for (int row = 0; row < rows; ++row) {
    // get the address of row
    const uchar* src_ptr = src_uchar.ptr<uchar>(row);
    float* dst_ptr = dst_float.ptr<float>(row);
    // change each value
    for (int col = 0; col < values_per_row; ++col) {
      // restore NaN if needed
      if (src_nan_indices_given && curr_pos_total == *src_nan_current_indice) {
        *dst_ptr = NAN_DEPTH;
        if (src_nan_indices_pos < src_nan_indices->size() - 1)
          ++src_nan_current_indice;
      }
      else {
        //*dst_ptr = image_val_to_dist(*src_ptr, alpha_trans, beta_trans);
        *dst_ptr = lookup_table[*src_ptr];
      }
      ++src_ptr;
      ++dst_ptr;
      if (src_nan_indices_given)
        ++ curr_pos_total;
    } // end loop col
  } // end loop row
} // end convert_uchar_to_float();

////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////


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
        //if (float_img_ptr[col] == image_utils::NAN_DEPTH)
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
        //if (float_img_ptr[col] == image_utils::NAN_DEPTH)
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
        //if (float_img_ptr[col] == image_utils::NAN_DEPTH)
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
        //if (float_img_ptr[col] != image_utils::NAN_DEPTH) {
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
        //if (float_img_ptr[col] != image_utils::NAN_DEPTH) {
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
  image_utils::depth_image_to_vizualisation_color_image
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

enum FileFormat {
  FILE_PNG = 0,
  FILE_BMP = 1,
  FILE_PPM_BINARY = 2,
  FILE_JPG = 3 // lossy, shouldnt be used
};

typedef std::vector<int> ParamsVec;

inline ParamsVec format2params(FileFormat format) {
  ParamsVec ans;
  switch (format) {
  case FILE_PNG:
    ans.push_back(CV_IMWRITE_PNG_COMPRESSION);
    ans.push_back(9);
  case FILE_PPM_BINARY:
    ans.push_back(CV_IMWRITE_PXM_BINARY);
    ans.push_back(1);
  case FILE_JPG:
    ans.push_back(CV_IMWRITE_JPEG_QUALITY);
    ans.push_back(85);
  case FILE_BMP:
  default:
  {}
  }
  return ans;
}

inline std::string format2extension(FileFormat format) {
  switch (format) {
  case FILE_BMP:
    return ".bmp";
  case FILE_PPM_BINARY:
    return ".ppm";
  default:
  case FILE_PNG:
    return ".png";
  }
  return ""; // never reached
}

////////////////////////////////////////////////////////////////////////////////

/*!
  write depth and rgb files to PNG images.
 \param filename_prefix
    Where to write the files, for instance "/tmp/test"
 \param rgb_img
    The 3 channels RGB image.
    If NULL, no RGB output will be written.
 \param depth_img_as_uchar, alpha, beta
    The data of the float image converted to uchar,
    using convert_float_to_uchar()
    If depth_img_as_uchar = NULL, no depth output will be written.
 \example filename_prefix = "/tmp/test"
    will generate "/tmp/test_depth.png"; "/tmp/test_depth_params.png";
    "/tmp/test_rgb.png";
*/
inline bool write_rgb_and_depth_image_as_uchar_to_image_file
(const std::string & filename_prefix,
 const cv::Mat * rgb_img = NULL,
 const cv::Mat * depth_img_as_uchar = NULL,
 const ScaleFactorType * alpha = NULL, const ScaleFactorType * beta = NULL,
 FileFormat format = FILE_PNG,
 bool debug_info = true)
{
  std::string extension = format2extension(format);
  ParamsVec params = format2params(format);
  // depth file
  if (depth_img_as_uchar != NULL) {
    std::ostringstream depth_img_filename;
    depth_img_filename << filename_prefix << "_depth" << extension;
    if (!cv::imwrite(depth_img_filename.str(), *depth_img_as_uchar, params)) {
      printf("write_rgb_and_depth_image_as_uchar_to_image_file(): "
             "could not write depth image '%s'\n", depth_img_filename.str().c_str());
      return false;
    }

    // depth params file
    std::ostringstream params_textfile_filename;
    params_textfile_filename << filename_prefix << "_depth_params.yaml";
    cv::FileStorage fs(params_textfile_filename.str(), cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
      printf("write_rgb_and_depth_image_as_uchar_to_image_file(): "
             "could not open depth params file '%s'\n", params_textfile_filename.str().c_str());
      return false;
    }
    fs << "alpha" << *alpha;
    fs << "beta" << *beta;
    fs.release();
    if (debug_info)
      printf("Written depth files '%s' and '%s'.\n",
             depth_img_filename.str().c_str(), params_textfile_filename.str().c_str());
  } // end (depth_img_as_uchar != NULL)

  // RGB file
  if (rgb_img != NULL) {
    std::ostringstream rgb_img_filename;
    rgb_img_filename << filename_prefix << "_rgb"  << extension;
    if (!cv::imwrite(rgb_img_filename.str(), *rgb_img, params)) {
      printf("write_rgb_and_depth_image_as_uchar_to_image_file(): "
             "could not write rgb image '%s'\n", rgb_img_filename.str().c_str());
      return false;
    }
    if (debug_info)
      printf("Written RGB file '%s'.\n", rgb_img_filename.str().c_str());
  } // end (rgb_img != NULL)
  return true;
} // end write_rgb_and_depth_image_as_uchar_to_image_file();

////////////////////////////////////////////////////////////////////////////////

/*!
  Converts the float image to uchar, then write depth and rgb files to PNG images.
  If rgb_img == NULL, no RGB output is written.
  If depth_img == NULL, no depth output is written.
  \see write_rgb_and_depth_image_as_uchar_to_image_file(),
       convert_float_to_uchar()
*/
inline bool write_rgb_and_depth_image_to_image_file
(const std::string & filename_prefix,
 const cv::Mat * rgb_img = NULL,
 const cv::Mat * depth_img = NULL,
 FileFormat format = FILE_PNG,
 bool debug_info = true)
{
  if (depth_img != NULL) {
    cv::Mat depth_img_as_uchar, src_float_clean_buffer;
    ScaleFactorType alpha, beta;
    convert_float_to_uchar(*depth_img, depth_img_as_uchar, src_float_clean_buffer,
                           alpha, beta);
    return write_rgb_and_depth_image_as_uchar_to_image_file
        (filename_prefix, rgb_img, &depth_img_as_uchar, &alpha, &beta,
         format, debug_info);
  } // end (depth_img != NULL)
  else
    return write_rgb_and_depth_image_as_uchar_to_image_file
        (filename_prefix, rgb_img, NULL, NULL, NULL, format, debug_info);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param filename
    a relative or absolute filename
 \return true if the file with given filename exists
*/
inline bool file_exists(const std::string & filename) {
  std::ifstream my_file(filename.c_str());
  return (my_file.good());
}

////////////////////////////////////////////////////////////////////////////////

/*!
  read depth and rgb files from PNG images.
 \param filename_prefix
    Where to read the files, for instance "/tmp/test"
 \param rgb_img
    The 3 channels RGB image.
    If NULL, no RGB input will be read.
 \param depth_img_as_uchar, alpha, beta
    The data of the float-as-uchar image,
    which can be used for convert_uchar_to_float();
    If depth_img_as_uchar = NULL, no depth input will be read.
 \example filename_prefix = "/tmp/test"
    will generate "/tmp/test_depth.png"; "/tmp/test_depth_params.png";
    "/tmp/test_rgb.png";
*/
inline bool read_rgb_and_depth_image_as_uchar_from_image_file
(const std::string & filename_prefix,
 cv::Mat * rgb_img = NULL,
 cv::Mat * depth_img_as_uchar = NULL,
 ScaleFactorType * alpha = NULL, ScaleFactorType * beta = NULL,
 FileFormat format = FILE_PNG)
{
  std::string extension = format2extension(format);
  if (depth_img_as_uchar != NULL) {
    // depth img
    std::ostringstream depth_img_filename;
    depth_img_filename << filename_prefix << "_depth" << extension;
    if (!file_exists(depth_img_filename.str())) {
      printf("depth img file '%s' does not exist, cannot read it!\n",
             depth_img_filename.str().c_str());
      return false;
    }
    *depth_img_as_uchar = cv::imread(depth_img_filename.str(), CV_LOAD_IMAGE_GRAYSCALE);
    if (depth_img_as_uchar->empty()) {
      printf("depth_img_as_uchar '%s' is corrupted!\n", depth_img_filename.str().c_str());
      return false;
    }
    // params file
    std::ostringstream params_textfile_filename;
    params_textfile_filename << filename_prefix << "_depth_params.yaml";
    if (!file_exists(params_textfile_filename.str())) {
      printf("params_textfile img file '%s' does not exist, cannot read it!\n",
             params_textfile_filename.str().c_str());
      return false;
    }
    cv::FileStorage fs(params_textfile_filename.str(), cv::FileStorage::READ);
    fs["alpha"] >> *alpha;
    fs["beta"] >> *beta;
    fs.release();
    //    printf("Read depth file '%s' (params_textfile:'%s').\n",
    //           depth_img_filename.str().c_str(),
    //           params_textfile_filename.str().c_str());
  } // end (depth_img_as_uchar != NULL)

  // rgb img
  if (rgb_img != NULL) {
    std::ostringstream rgb_img_filename;
    rgb_img_filename << filename_prefix << "_rgb" << extension;
    if (file_exists(rgb_img_filename.str()))
      *rgb_img = cv::imread(rgb_img_filename.str());
    else { // try another format
      std::ostringstream rgb_jpg_img_filename;
      rgb_jpg_img_filename << filename_prefix << "_rgb.jpg";
      if (!file_exists(rgb_jpg_img_filename.str())) {
        printf("rgb img file '%s' does not exist, cannot read it!\n",
               rgb_img_filename.str().c_str());
        return false;
      }
      *rgb_img = cv::imread(rgb_jpg_img_filename.str());
    }
    // printf("Read rgb file '%s'.\n", rgb_img_filename.str().c_str());
    if (rgb_img->empty()) {
      printf("rgb_img '%s' is corrupted!\n", rgb_img_filename.str().c_str());
      return false;
    }
  } // end (rgb_img != NULL)
  return true;
} // end read_rgb_and_depth_image_as_uchar_from_image_file();

////////////////////////////////////////////////////////////////////////////////

/*!
  Read RGB and depth-as-uchar images from PNG,
  then converts depth-as-uchar to depth-as-float.
  If rgb_img == NULL, no RGB input is read.
  If depth_img == NULL, no depth input is read.
  \see read_rgb_and_depth_image_as_uchar_from_image_file(),
       convert_float_to_uchar()
*/
inline bool read_rgb_and_depth_image_from_image_file
(const std::string & filename_prefix,
 cv::Mat * rgb_img = NULL,
 cv::Mat * depth_img = NULL,
 FileFormat format = FILE_PNG)
{
  if (depth_img != NULL) {
    cv::Mat depth_img_as_uchar;
    ScaleFactorType alpha = 1, beta = 0;
    bool ok = read_rgb_and_depth_image_as_uchar_from_image_file
              (filename_prefix, rgb_img, &depth_img_as_uchar, &alpha, &beta, format);
    if (!ok)
      return false;
    convert_uchar_to_float(depth_img_as_uchar, *depth_img, alpha, beta);
    return true;
  } // end (depth_img != NULL)
  return read_rgb_and_depth_image_as_uchar_from_image_file
      (filename_prefix, rgb_img, NULL, NULL, NULL, format);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
} // end namespace image_utils

#endif // CV_CONVERSION_FLOAT_UCHAR_H
