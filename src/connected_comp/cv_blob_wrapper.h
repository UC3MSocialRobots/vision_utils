#ifndef CV_BLOB_WRAPPER_H
#define CV_BLOB_WRAPPER_H

// third parties
#include "cvBlob/cvblob.h"
// the interface
#include "comp_labeller_interface.h"

/*! \class  CvBlobWrapper
 *
 */
class CvBlobWrapper : public CompLabellerInterface {
public:
    /*! constructor */
    CvBlobWrapper();

    //! \see CompLabellerInterface::process_image()
    void process_image(cv::Mat1b & img);

    //! \see CompLabellerInterface::get_connected_components()
    void get_connected_components
    (const int cols,
     std::vector< Comp > & components_pts,
     std::vector<cv::Rect> & boundingBoxes);

private:
    //! where the labels are stored
    IplImage* label_img;
    //! the CvBlobs real object
    cvb::CvBlobs blobs;

};

#endif // CV_BLOB_WRAPPER_H
