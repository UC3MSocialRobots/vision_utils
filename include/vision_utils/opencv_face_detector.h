#ifndef OPENCV_FACE_DETECTOR_PPLP_H
#define OPENCV_FACE_DETECTOR_PPLP_H

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <vision_utils/img_path.h>
#include "vision_utils/geometry_utils.h"
// vision
#include "vision_utils/resize_utils.h"

namespace vision_utils {
// face detection param

static const std::string DEFAULT_FACE_CASCADE =
    IMG_DIR
    //"haarCascade_openCV/haarcascade_frontalface_alt.xml";
    "haarCascade_openCV/haarcascade_frontalface_alt2.xml";

/*! the scale reduction factor of the RGB image for face detection
 * (btwn 0 and 1) */
static const int DEFAULT_RESIZE_MAX_WIDTH = 250;
static const int DEFAULT_RESIZE_MAX_HEIGHT = 160;
//! parameter for face detection - the lower the more permissive
static const double DEFAULT_SCALE_FACTOR = 1.1;
//! parameter for face detection - the lower the more permissive
static const int DEFAULT_MIN_NEIGHBORS = 2; // 3
//! in pixels - the lower the more permissive
static const int DEFAULT_MIN_WIDTH = 10; // 15

////////////////////////////////////////////////////////////////////////////////

//! create a face classifier
cv::CascadeClassifier create_face_classifier
(const std::string & cascade1_name = DEFAULT_FACE_CASCADE) {
  //printf("cascade1_name:'%s'", cascade1_name.c_str());
  cv::CascadeClassifier cascadeClassifier;
  cascadeClassifier.load( cascade1_name );
  return cascadeClassifier;
} // end create_face_classifier()

////////////////////////////////////////////////////////////////////////////////

void detect_with_opencv(const cv::Mat3b & rgb_img,
                        cv::CascadeClassifier & cascadeClassifier,
                        cv::Mat3b & small_img,
                        std::vector< cv::Rect > & found_faces,
                        int resize_max_width = DEFAULT_RESIZE_MAX_WIDTH,
                        int resize_max_height = DEFAULT_RESIZE_MAX_HEIGHT,
                        double scale_factor = DEFAULT_SCALE_FACTOR,
                        int min_neighbors = DEFAULT_MIN_NEIGHBORS,
                        int min_width = DEFAULT_MIN_WIDTH) {
  // //printf("detect_with_opencv()");

  float scale = resize_if_bigger(rgb_img, small_img,
                                 resize_max_width, resize_max_height);
  //printf("scale:%f", scale);

  // convert to BW
  //    cv::cvtColor(big_img, frameBW, CV_BGR2GRAY);
  //    cv::equalizeHist( frameBW, frameBW );
  //printf("frameBW:%ix%i", frameBW.cols, frameBW.rows);


  //    detectMultiScale( const Mat& image,
  //                     CV_OUT vector<Rect>& faces,
  //                     double scaleFactor=1.1,
  //                     int minNeighbors=3, int flags=0,
  //                     Size minSize=Size(),
  //                     Size maxSize=Size());
  cascadeClassifier.detectMultiScale((scale > 0 ? small_img : rgb_img),
                                     found_faces,
                                     scale_factor,
                                     min_neighbors,
                                     0
                                     |CV_HAAR_DO_CANNY_PRUNING
                                     |CV_HAAR_SCALE_IMAGE
                                     |CV_HAAR_DO_ROUGH_SEARCH
                                     //|CV_HAAR_FIND_BIGGEST_OBJECT
                                     , cv::Size(min_width * scale,
                                                min_width * scale));

  // rescale found faces
  if (scale > 0) {
    // shrink rec cannot be used here, as we need to scale the whole rec
    for (unsigned int rec_idx = 0; rec_idx < found_faces.size(); ++rec_idx) {
      cv::Rect* r = &(found_faces[rec_idx]);
      r->x /= scale;
      r->y /= scale;
      r->width /= scale;
      r->height /= scale;
    } // end loop rec_idx
  } // end if need_scale
} // end detect_with_opencv();
} // end namespace vision_utils

#endif // OPENCV_FACE_DETECTOR_PPLP_H
