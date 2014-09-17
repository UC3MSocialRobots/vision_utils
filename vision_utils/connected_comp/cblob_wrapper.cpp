#include "cblob_wrapper.h"

#include "vision_utils/image_utils/content_processing.h"
#include "src/debug/debug.h"
#include <opencv2/highgui/highgui.hpp>

CBlobWrapper::CBlobWrapper() {
}

////////////////////////////////////////////////////////////////////////////////

void CBlobWrapper::process_image(cv::Mat1b & img) {
    maggieDebug3("process_image()");
    buffer.create(img.size());


    IplImage image_sample_ipl = img;
    blobs = CBlobResult( &image_sample_ipl, NULL, 100, false);

    // object with the blob with most perimeter in the image
    // ( the criteria to select can be any class derived from COperadorBlob )
    CBlob blobWithBiggestPerimeter;
    CBlob blobWithLessArea;
    // from the filtered blobs, get the blob with biggest perimeter
    blobs.GetNthBlob( CBlobGetPerimeter(), 0, blobWithBiggestPerimeter );

}

////////////////////////////////////////////////////////////////////////////////

void CBlobWrapper::get_connected_components(const int cols,
                                            std::vector< std::vector<cv::Point> > & components_pts,
                                            std::vector<cv::Rect> & boundingBoxes) {
    maggieDebug3("get_connected_components()");

    // clear memory
    components_pts.clear();
    components_pts.reserve(blobs.GetNumBlobs());
    boundingBoxes.clear();

    // init seenPoints as an aray of bools
    bool seenPoints[buffer.cols * buffer.rows];
    IplImage buffer_ipl = buffer;

    buffer = 0;
    for (int blob_idx = 0; blob_idx < blobs.GetNumBlobs(); ++blob_idx) {
        //maggiePrint("blob_idx:%i", blob_idx);

        CBlob* curr_blob = blobs.GetBlob(blob_idx);
        //maggiePrint("curr_blob:exterior:%i", curr_blob->Exterior());
        if (curr_blob->Exterior())
            continue;

        // color the whole component
        boundingBoxes.push_back(cv::Rect());
        curr_blob->FillBlob(&buffer_ipl, cvScalarAll(128));
        //cv::imshow("buffer", buffer); cv::waitKey(0);

        // collect that connected component
        components_pts.push_back(std::vector<cv::Point>());
        components_pts.back().reserve(curr_blob->area);
        // get the seed
        CvSeqReader reader;
        cvStartReadSeq( curr_blob->edges, &reader);
        CvPoint seed;
        CV_READ_SEQ_ELEM(seed ,reader);
        //maggiePrint("seed:%i, %i", seed.x, seed.y);

        // now region growth
        image_utils::region_growth(buffer, seenPoints,
                                  seed,
                                  1, 1,
                                  components_pts.back());
    } // end loop blob_idx

}
