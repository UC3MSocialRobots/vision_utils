#ifndef CONNECTED_COMP_INTERFACE_H
#define CONNECTED_COMP_INTERFACE_H

#include "vision_utils/disjoint_sets2.h"

namespace vision_utils {

/*!
 *\brief   computes the biggest conencted component
     *
 *\param   img the monochrome image
 *\param   rep the vector which will contain the results
 */
//void biggestComponent_vector(const cv::Mat1b & img, std::vector<cv::Point> & rep);
inline void biggestComponent_vector2(cv::Mat1b & img,
                                     std::vector<cv::Point> & rep) {
  DisjointSets2 set(img);
  set.biggestComponent_vector(img.cols, rep);
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   computes the biggest conencted component
     *
 *\param   img the monochrome image
 *\param   imgRep a monochrome image, of the same size or greater than img
 */
//void biggestComponent_image(const cv::Mat1b & img, cv::Mat1b & imgRep);
inline void biggestComponent_image2(cv::Mat1b & img, cv::Mat1b & imgRep) {
  DisjointSets2 set(img);
  set.biggestComponent_image(img.cols, imgRep);
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   computess the center of the biggest connected component of the image
     *
 *\param   img the monochrome image
 *\return  the cv::Point corresponding to the center
 */
//cv::Point centroidOfMonochromeImage(const cv::Mat1b & img);
inline cv::Point centroidOfMonochromeImage2(cv::Mat1b & img) {
  DisjointSets2 set(img);
  return set.centroidOfMonochromeImage(img.cols);
}

} // end namespace vision_utils

#endif // CONNECTED_COMP_INTERFACE_H
