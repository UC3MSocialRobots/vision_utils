#ifndef CONNECTED_COMP_INTERFACE_H
#define CONNECTED_COMP_INTERFACE_H

#include "connected_comp/disjoint_sets2.h"

namespace image_utils {

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

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   returns all the list of points making all the connected components
 *of the image
     *
 *\param   img the monochrome image
 *\param   components_pts the vector of vector of points which will
 *contain the results
 *\param   boundingBoxes the bounding boxes of the points
 */
inline void connectedComponents2(cv::Mat1b & img,
                                 std::vector< std::vector<cv::Point> > & components_pts,
                                 std::vector<cv::Rect> & boundingBoxes) {
  DisjointSets2 set(img);
  return set.get_connected_components(img.cols, components_pts, boundingBoxes);
}

} // end namespace image_utils

#endif // CONNECTED_COMP_INTERFACE_H
