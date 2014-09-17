#ifndef LAYER_UTILS_H
#define LAYER_UTILS_H

#include "vision_utils/color/color_utils.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace image_utils {

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   extract one of the layer from an image
     *
 *\param   src the source
 *\param   dest the destination
 *\param   layer_idx the number of the layer (between 0 and 2)
 */
inline void extractLayer(const cv::Mat3b & src, cv::Mat1b & dest,
                         const int layer_idx) {
  // TODO check there is a copy done
#if 1
  std::vector<cv::Mat> planes;
  cv::split(src, planes);
  dest = planes.at(layer_idx);
#else
  IplImage src_ipl = src, dest_ipl = dest;
  if (layer == 0)
    cvSplit(&src_ipl, &dest_ipl, 0, 0, 0); // hue
  if (layer == 1)
    cvSplit(&src_ipl, 0, &dest_ipl, 0, 0); // saturation
  if (layer == 2)
    cvSplit(&src_ipl, 0, 0, &dest_ipl, 0); // value
#endif
}

} // end namespace image_utils

namespace color_utils {

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   save in "dest" one HSV component of "src"
 */
inline void rgb2hsv_layer(const cv::Mat3b & src_bgr, cv::Mat3b & temp_hsv,
                          cv::Mat1b & dest_layer, const int layer_idx) {
  cv::cvtColor(src_bgr, temp_hsv, CV_BGR2HSV); // conversion in HSV
  image_utils::extractLayer(temp_hsv, dest_layer, layer_idx);
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   save in "dest" the hue component of "src"
 */
inline void rgb2hue(const cv::Mat3b & src_bgr, cv::Mat3b & temp_hsv,
                    cv::Mat1b & dest_hue) {
  rgb2hsv_layer(src_bgr, temp_hsv, dest_hue, 0);
}

////////////////////////////////////////////////////////////////////////////////

/*! short version for rgb2hue().
 *  Careful, the src image is modified: converted to BGR -> HSV.
 */
inline cv::Mat1b rgb2hue(cv::Mat3b & src_bgr) {
  cv::Mat1b hue;
  rgb2hsv_layer(src_bgr, src_bgr, hue, 0);
  return hue;
}

////////////////////////////////////////////////////////////////////////////////

//! read an image from file and return its
inline cv::Mat1b rgb_file2hue(const std::string & filename) {
  cv::Mat3b img_rgb = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  return rgb2hue(img_rgb);
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   save in "dest" the saturation component of "src"
 */
inline void rgb2saturation(const cv::Mat3b & src_bgr, cv::Mat3b & temp_hsv,
                           cv::Mat1b & dest_saturation) {
  rgb2hsv_layer(src_bgr, temp_hsv, dest_saturation, 1);
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   save in "dest" the value component of "src"
 */
inline void rgb2value(const cv::Mat3b & src_bgr, cv::Mat3b & temp_hsv,
                      cv::Mat1b & dest_value) {
  rgb2hsv_layer(src_bgr, temp_hsv, dest_value, 2);
}

////////////////////////////////////////////////////////////////////////////

inline void hue2rgb(const cv::Mat1b & hue, cv::Mat3b & rgb) {
  // make lookup table - hue goes in 0..180
  static std::vector<cv::Vec3b> hue_lut;
  color_utils::hue2rgb_make_lookup_table(hue_lut, 180);
  // use it
  rgb.create(hue.size());
  for (int row = 0; row < hue.rows; ++row) {
    // get the address of row
    const uchar* hue_data = hue.ptr<uchar>(row);
    cv::Vec3b* rgb_data = rgb.ptr<cv::Vec3b>(row);
    for (int col = 0; col < hue.cols; ++col) {
      rgb_data[col] = hue_lut[ hue_data[col] ];
    } // end loop col
  } // end loop row
}

////////////////////////////////////////////////////////////////////////////

inline cv::Mat3b hue2rgb(const cv::Mat1b & hue) {
  cv::Mat3b rgb;
  hue2rgb(hue, rgb);
  return rgb;
}

////////////////////////////////////////////////////////////////////////////////

void saturate_saturation_value(const cv::Mat3b & src_bgr,
                               std::vector<cv::Mat> & layers,
                               cv::Mat3b & out_bgr) {
  cv::cvtColor(src_bgr, out_bgr, CV_BGR2HSV); // convert BGR -> HSV
  cv::split(out_bgr, layers);
  layers[1].setTo(255); // set saturation to 255
  layers[2].setTo(255); // set value to 255
  cv::merge(layers, out_bgr); // recombine HSV
  cv::cvtColor(out_bgr, out_bgr, CV_HSV2BGR); // now convert HSV -> BGR
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   create the rgb image which illustrate the hue
 *\param   src a bgr image
 *\param   dest a bgr image
 */
inline void rgb_saturate_saturation_value_slow(const cv::Mat3b & src_bgr, cv::Mat3b & dest_bgr)  {
  std::vector<cv::Mat> layers;
  saturate_saturation_value(src_bgr, layers, dest_bgr);
}

inline void rgb_saturate_saturation_value(const cv::Mat3b & src_bgr, cv::Mat3b & dest_bgr)  {
  cv::Mat1b hue;
  rgb2hue(src_bgr, dest_bgr, hue);
  hue2rgb(hue, dest_bgr);
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   create the rgb image which illustrate the hue component
 *of the image "filename"
 */
inline void rgb2hue_and_showRGB(const std::string & filename) {
  cv::Mat3b original_h = cv::imread(filename, CV_LOAD_IMAGE_COLOR), res;
  rgb_saturate_saturation_value(original_h, res);
  const char* window_hueImage = "Hue Previewer";
  cv::namedWindow(window_hueImage, CV_WINDOW_AUTOSIZE);
  cv::imshow(window_hueImage, res);
  cv::waitKey(0);
} // end rgb2hue_and_showRGB();

////////////////////////////////////////////////////////////////////////////

inline void HSVfilter(const cv::Mat3b & srcHSV,
                      const int outValue, cv::Mat1b & result,
                      const int Hmin, const int Hmax,
                      const int Smin, const int Smax,
                      const int Vmin, const int Vmax,
                      cv::Mat1b & Hbuffer, cv::Mat1b & Sbuffer, cv::Mat1b & Vbuffer) {
  /*
     * init the images if needed
     */
  Hbuffer.create(srcHSV.size());
  Sbuffer.create(srcHSV.size());
  Vbuffer.create(srcHSV.size());
  result.create(srcHSV.size());

  /*
     * split
     */
  std::vector< cv::Mat1b > layers;
  layers.push_back(Hbuffer);
  layers.push_back(Sbuffer);
  layers.push_back(Vbuffer);
  cv::split(srcHSV, layers);

  /*
     * filter each channel
     */
  cv::threshold(Hbuffer, Hbuffer, Hmax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Hbuffer, Hbuffer, Hmin, outValue, cv::THRESH_BINARY);
  cv::threshold(Sbuffer, Sbuffer, Smax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Sbuffer, Sbuffer, Smin, outValue, cv::THRESH_BINARY);
  cv::threshold(Vbuffer, Vbuffer, Vmax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Vbuffer, Vbuffer, Vmin, outValue, cv::THRESH_BINARY);

  /*
     * combine
     */
  cv::min( (cv::Mat) Hbuffer, Sbuffer, result);
  cv::min( (cv::Mat) result, Vbuffer, result);
}

} // end namespace color_utils

#endif // LAYER_UTILS_H
