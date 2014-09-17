#include "cv_blob_wrapper.h"
#include "src/debug/debug.h"

CvBlobWrapper::CvBlobWrapper() {
    label_img = NULL;
}

////////////////////////////////////////////////////////////////////////////////

void CvBlobWrapper::process_image(cv::Mat1b & img) {
    maggieDebug3("process_image()");
    // init if needed
    if (label_img == NULL ||
            label_img->width != img.cols || label_img->height != img.rows) {
        cvReleaseImage(&label_img);
        label_img = cvCreateImage(img.size(), 32, 1);
    }

    IplImage image_sample_ipl = img;
    maggieDebug3("depth:%i (== %i ?), %i channels",
                 image_sample_ipl.depth, IPL_DEPTH_LABEL,
                 image_sample_ipl.nChannels);
    //unsigned int nb_pixels =
    cvb::cvLabel(&image_sample_ipl, label_img, blobs);

}

////////////////////////////////////////////////////////////////////////////////

void CvBlobWrapper::get_connected_components(const int cols,
                                             std::vector< Comp > & components_pts,
                                             std::vector<cv::Rect> & boundingBoxes) {
    maggieDebug3("get_connected_components()");

    // allocate memory
    int nb_blobs = (int) blobs.size();
    //maggiePrint("nb_blobs:%i", nb_blobs);
    components_pts.clear();
    components_pts.resize(nb_blobs);
    boundingBoxes.clear();
    boundingBoxes.resize(nb_blobs);

    // reserve size
    for (int comp_idx = 0; comp_idx < nb_blobs; ++comp_idx) {
        components_pts.at(comp_idx).reserve(blobs.at(comp_idx + 1)->area);
    } // end loop comp_idx

    // iterate
    for (int row = 0; row < label_img->height; ++row) {
        for (int col = 0; col < label_img->width; ++col) {
            int comp_idx = cvb::cvGetLabel(label_img, col, row);
            if (comp_idx > 0) { // -1 is nothing, 0 is background
                //maggiePrint("comp_idx:%i", comp_idx);
                components_pts.at(comp_idx - 1).push_back(cv::Point(col, row));
            }
        } // end loop col
    } // end loop row

    //    cvb::CvLabel label = cvGreaterBlob(blobs);
    //    maggiePrint("biggest blob:%i, area:%i",
    //                label, blobs.at(label)->area);

    //CV_IMAGE_ELEM();
}
