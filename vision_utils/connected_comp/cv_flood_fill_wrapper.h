#ifndef CV_FLOOD_FILL_WRAPPER_H
#define CV_FLOOD_FILL_WRAPPER_H

// the interface
#include "comp_labeller_interface.h"

/*! \class  CvFloodFillWrapper
 *
 */
class CvFloodFillWrapper : public CompLabellerInterface {
public:
    /*! constructor */
    CvFloodFillWrapper();

    //! \see CompLabellerInterface::process_image()
    void process_image(cv::Mat1b & img);

    //! \see CompLabellerInterface::get_connected_components()
    void get_connected_components
    (const int cols,
     std::vector< Comp > & components_pts,
     std::vector<cv::Rect> & boundingBoxes);

private:
    cv::Mat1b buffer;
    std::vector< Comp > _components_pts;
    std::vector< cv::Rect > _boundingBoxes;

    static const int POINT_NOT_SEEN = 255;
    static const int POINT_SEEN = 128;
};

#endif // CV_FLOOD_FILL_WRAPPER_H
