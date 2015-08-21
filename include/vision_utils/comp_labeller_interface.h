#ifndef COMP_LABELLER_INTERFACE_H
#define COMP_LABELLER_INTERFACE_H

// std
#include "opencv2/core/core.hpp"
#include "vision_utils/utils/sort_utils.h"

/*! \class  CompLabellerInterface
 *
 */
class CompLabellerInterface {
public:
  typedef cv::Point Point;
  typedef std::vector<cv::Point> Comp;

  /*! constructor */
  CompLabellerInterface() {}

  /*!
     * Labels the connected components.
     * This can be the place for the first pass.
     * \param img the image to process
     */
  virtual void process_image(cv::Mat1b & img) = 0;

  /*!
     * \brief   returns all the list of points making all the connected components
     * of the image
     * This can be the place for the second pass.
     *
     * \param   img the monochrome image
     * \param   components_pts the vector of vector of points which will
     * contain the results
     * \param   boundingBoxes the bounding boxes of the points
     */
  virtual void get_connected_components
  (const int cols,
   std::vector< std::vector<cv::Point> > & components_pts,
   std::vector<cv::Rect> & boundingBoxes) = 0;

  ////////////////////////////////////////////////////////////////////////////

  //! sort the components by number of points
  bool sort_comps_by_decreasing_size(std::vector< std::vector<cv::Point> > & components_pts,
                                     std::vector<cv::Rect> & boundingBoxes) {
    unsigned int ncomps = components_pts.size();
    sizes.resize(ncomps);
    for (unsigned int i = 0; i < ncomps; ++i)
      sizes[i] = components_pts[i].size();
    // do not sort sizes, not needed - decreasing order
    sort_order.from_vec(sizes, false, false);
    if (!sort_order.apply_on_vec(components_pts))
      return false;
    return (sort_order.apply_on_vec(boundingBoxes));
  }

private:
  std::vector<size_t> sizes;
  Order sort_order;
};

#endif // COMP_LABELLER_INTERFACE_H
