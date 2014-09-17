#ifndef CBLOB_WRAPPER_H
#define CBLOB_WRAPPER_H

// the interface
#include "comp_labeller_interface.h"
// third parties
#include "blob/BlobResult.h"
#include "blob/Blob.h"
#include "blob/BlobExtraction.h"
#include "blob/BlobLibraryConfiguration.h"


/*! \class  CBlobWrapper
 *
 */
class CBlobWrapper {
public:
    /*! constructor */
    CBlobWrapper();

    //! \see CompLabellerInterface::process_image()
    void process_image(cv::Mat1b & img);

    //! \see CompLabellerInterface::get_connected_components()
    void get_connected_components
    (const int cols,
     std::vector< std::vector<cv::Point> > & components_pts,
     std::vector<cv::Rect> & boundingBoxes);

private:
    cv::Mat1b buffer;
    CBlobResult blobs;
};

#endif // CBLOB_WRAPPER_H
