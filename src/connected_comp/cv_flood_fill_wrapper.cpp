#include "cv_flood_fill_wrapper.h"
#include "image_utils/content_processing.h"
#include "debug/debug_utils.h"
// third parties
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

CvFloodFillWrapper::CvFloodFillWrapper() {
}

////////////////////////////////////////////////////////////////////////////////

void CvFloodFillWrapper::process_image(cv::Mat1b & img) {
    maggieDebug3("process_image()");

    // clear memory
    _components_pts.clear();
    _boundingBoxes.clear();

    // threshold the img
    buffer.create(img.cols, img.rows);
    cv::threshold(img, buffer, 1, POINT_NOT_SEEN, cv::THRESH_BINARY);
    //cv::imshow("buffer", buffer); cv::waitKey(0);

    // init seenPoints as an aray of bools
    bool seenPoints[buffer.cols * buffer.rows];

    for (int row = 0; row < img.rows; ++row) {
        // get the address of row
        const uchar* data = buffer.ptr<uchar>(row);

        for (int col = 0; col < img.cols; ++col) {
            //maggiePrint("x:%i, y:%i, data:%i", row, col, *data);

            // make a flood fill only if the img pixel is not black and
            // the mask pixel is black (not seen yet)
            if (*data == POINT_NOT_SEEN) {

                // color the whole component
                _boundingBoxes.push_back(cv::Rect());
                cv::floodFill(buffer, cv::Point(col, row), cv::Scalar(POINT_SEEN),
                              &_boundingBoxes.back());

                _components_pts.push_back(Comp());
                // collect that connected component
                image_utils::region_growth(buffer, seenPoints,
                                          cv::Point(col, row),
                                          1, 1,
                                          _components_pts.back());

                //cv::imshow("buffer", buffer); cv::waitKey(0);
            }
            ++data;
        } // end loop x
    } // end loop y
}

////////////////////////////////////////////////////////////////////////////////

void CvFloodFillWrapper::get_connected_components
(const int cols,
 std::vector< CvFloodFillWrapper::Comp > & components_pts,
 std::vector<cv::Rect> & boundingBoxes) {
    maggieDebug3("get_connected_components()");
    components_pts = _components_pts;
    boundingBoxes = _boundingBoxes;
}
