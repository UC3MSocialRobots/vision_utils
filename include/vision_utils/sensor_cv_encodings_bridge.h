/*!
 * compressed_rounded_image_transport is an image_transport plugin for float images.
 * \author Arnaud Ramey ( arnaud.a.ramey@gmail.com )
            -- Robotics Lab, University Carlos III of Madrid
 * \date Jan. 2012
  */

#ifndef SENSOR_CV_ENCODINGS_BRIDGE_H
#define SENSOR_CV_ENCODINGS_BRIDGE_H

#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>

namespace vision_utils {

typedef std::string SensorEncoding;
typedef int CvEncoding;

/*!
  a function to convert an OpenCV encoding (an int obtained by mat.type)
  to a sensor_msgs encoding (a string, obtained by image.encoding)
 \param src_enc
 \return SensorEncoding
*/
inline SensorEncoding sensor_encoding_from_cv_encoding(const CvEncoding & src_enc) {
  switch(src_enc) {
  // 8UC()
  case CV_8UC1:
    return sensor_msgs::image_encodings::TYPE_8UC1;
  case CV_8UC2:
    return sensor_msgs::image_encodings::TYPE_8UC2;
  case CV_8UC3:
    return sensor_msgs::image_encodings::TYPE_8UC3;
  case CV_8UC4:
    return sensor_msgs::image_encodings::TYPE_8UC4;
    // 8SC()
  case CV_8SC1:
    return sensor_msgs::image_encodings::TYPE_8SC1;
  case CV_8SC2:
    return sensor_msgs::image_encodings::TYPE_8SC2;
  case CV_8SC3:
    return sensor_msgs::image_encodings::TYPE_8SC3;
  case CV_8SC4:
    return sensor_msgs::image_encodings::TYPE_8SC4;

    // 16UC()
  case CV_16UC1:
    return sensor_msgs::image_encodings::TYPE_16UC1;
  case CV_16UC2:
    return sensor_msgs::image_encodings::TYPE_16UC2;
  case CV_16UC3:
    return sensor_msgs::image_encodings::TYPE_16UC3;
  case CV_16UC4:
    return sensor_msgs::image_encodings::TYPE_16UC4;
    // 16SC()
  case CV_16SC1:
    return sensor_msgs::image_encodings::TYPE_16SC1;
  case CV_16SC2:
    return sensor_msgs::image_encodings::TYPE_16SC2;
  case CV_16SC3:
    return sensor_msgs::image_encodings::TYPE_16SC3;
  case CV_16SC4:
    return sensor_msgs::image_encodings::TYPE_16SC4;

    // 32FC()
  case CV_32FC1:
    return sensor_msgs::image_encodings::TYPE_32FC1;
  case CV_32FC2:
    return sensor_msgs::image_encodings::TYPE_32FC2;
  case CV_32FC3:
    return sensor_msgs::image_encodings::TYPE_32FC3;
  case CV_32FC4:
    return sensor_msgs::image_encodings::TYPE_32FC4;
    // 32SC()
  case CV_32SC1:
    return sensor_msgs::image_encodings::TYPE_32SC1;
  case CV_32SC2:
    return sensor_msgs::image_encodings::TYPE_32SC2;
  case CV_32SC3:
    return sensor_msgs::image_encodings::TYPE_32SC3;
  case CV_32SC4:
    return sensor_msgs::image_encodings::TYPE_32SC4;

    // 64FC()
  case CV_64FC1:
    return sensor_msgs::image_encodings::TYPE_64FC1;
  case CV_64FC2:
    return sensor_msgs::image_encodings::TYPE_64FC2;
  case CV_64FC3:
    return sensor_msgs::image_encodings::TYPE_64FC3;
  case CV_64FC4:
    return sensor_msgs::image_encodings::TYPE_64FC4;

  default:
    return "unknown encoding";
  } // end switch(src_enc)
} // end sensor_encoding_from_cv_encoding()

} // end namespace vision_utils

#endif // SENSOR_CV_ENCODINGS_BRIDGE_H
