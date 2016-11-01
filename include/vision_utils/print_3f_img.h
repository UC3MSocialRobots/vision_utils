/*!
 * compressed_rounded_image_transport is an image_transport plugin for float images.
 * \author Arnaud Ramey ( arnaud.a.ramey@gmail.com )
            -- Robotics Lab, University Carlos III of Madrid
 * \date Jan. 2012
  */

#ifndef PRINT_3F_IMG_H
#define PRINT_3F_IMG_H

#include <sstream>
#include <sensor_msgs/Image.h>
#include "vision_utils/nan_handling.h"

namespace vision_utils {

//! uncomment for printing image info
//#define DEBUG_PRINT

////////////////////////////////////////////////////////////////////////////////

//! when flag DEBUG_PRINT is on, print some infos about the img size and channels
inline void print_cv_img_info(const cv::Mat & img, const std::string & img_name) {
#ifdef DEBUG_PRINT
  //printf("%s: %i x %i (%i channels, step1:%i)",
           img_name.c_str(), img.cols, img.rows, img.channels(),
           img.step1());
#endif // DEBUG_PRINT
}

////////////////////////////////////////////////////////////////////////////////

//! when flag DEBUG_PRINT is on, print some infos about the img size and encoding
inline void print_sensor_img_info(const sensor_msgs::Image & img, const std::string & img_name) {
#ifdef DEBUG_PRINT
  //printf("%s: %i x %i (encoding:'%s', data_size:%i)",
           img_name.c_str(), img.width, img.height, img.encoding.c_str(),
           img.data.size());
#endif // DEBUG_PRINT
}

////////////////////////////////////////////////////////////////////////////////

/*! Print all values of an image
  Template<_Type>: the basic type of the mat, ex float for cv::Mat3f
 \param img
    the image to be printed
 \param img_name
    how the image is called (is put on the first line)
*/
template<class _Type>
inline void print_img(const cv::Mat & img, const std::string & img_name) {
  print_cv_img_info(img, img_name);
#ifdef DEBUG_PRINT
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const _Type* data = img.ptr<_Type>(row);
    std::ostringstream info;
    info << "row " << row << ": ";
    for (int col = 0; col < img.cols; ++col) {
      info << (img.channels() == 1 ? "" : "(");
      for (int nchan = 0; nchan < img.channels(); ++nchan)
        info <<  (double) *data++
              << (nchan == img.channels() - 1 ? "" : ", ");
      info << (img.channels() == 1 ? ", " : "), ");
    } // end loop col
    printf("%s\n", info.str().c_str());
  } // end loop row
#endif // DEBUG_PRINT
} // end print_img()

////////////////////////////////////////////////////////////////////////////////

/*! Print some stats about the differences between two images */
inline double compare_two_images(const cv::Mat & img1, const cv::Mat & img2,
                                 bool clean_nans = true) {
  cv::Mat img1_one_channel_cleaned = img1.reshape(1).clone();
  cv::Mat img2_one_channel_cleaned = img2.reshape(1).clone();
  if (clean_nans) {
    remove_nans<float>(img1_one_channel_cleaned);
    remove_nans<float>(img2_one_channel_cleaned);
  } // end if (clean_nans)

  cv::Mat error_one_channel, error_rel_one_channel;
  cv::absdiff(img1_one_channel_cleaned, img2_one_channel_cleaned, error_one_channel);
  cv::divide(error_one_channel, img1_one_channel_cleaned, error_rel_one_channel);

  // find average error
  int n_values = img1.cols * img1.rows * img1.channels();
  double error_avg = cv::norm(error_rel_one_channel, cv::NORM_L1) / n_values;

  // find the worst value
  cv::Mat error_rel_one_channel_abs = cv::abs(error_rel_one_channel);
  double min_val, max_val;
  cv::Point min_pt, max_pt;
  cv::minMaxLoc(error_rel_one_channel_abs, &min_val, &max_val, &min_pt, &max_pt);

  printf("Total values:%i, different values:%i, avg error:%g %%\t "
         "max error:%g %%\t (at (%i, %i), orig:%g, dest:%g)\n",
         n_values,  cv::countNonZero(error_one_channel),
         error_avg * 100, max_val * 100, max_pt.x, max_pt.y,
         img1_one_channel_cleaned.at<float>(max_pt),
         img2_one_channel_cleaned.at<float>(max_pt));

  return error_avg;
} // end compare_two_images();

} // end namespace vision_utils

#endif // PRINT_3F_IMG_H
