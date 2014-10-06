
#include "debug/debug_utils.h"
#include "time/timer.h"
#include <vision_utils/img_path.h>
// third parties
#include "vision_utils/third_parties/blob/BlobResult.h"
#include "vision_utils/third_parties/blob/Blob.h"
#include "vision_utils/third_parties/blob/BlobExtraction.h"
#include "vision_utils/third_parties/blob/BlobLibraryConfiguration.h"

#include "vision_utils/third_parties/cvBlob/cvblob.h"

#include "opencv2/highgui/highgui.hpp"
// C++
#include <vector>
#include <iostream>

////////////////////////////////////////////////////////////////////////////////

void test_conn(std::string filename) {
    std::cout << std::endl << std::endl;
    maggiePrint("test_conn(%s)", filename.c_str());
    cv::Mat1b image_sample = cv::imread( filename, CV_LOAD_IMAGE_GRAYSCALE);
    Timer timer;
    int nb_times = 1;

    /*
     * trying with centroidOfMonochromeImage()
     */
    cv::Point rep_pt;
    //    timer.reset();
    //    for (int i = 0; i < nb_times; ++i)
    //        rep_pt = image_utils::centroidOfMonochromeImage(image_sample);
    //    timer.printTime_factor("centroidOfMonochromeImage", nb_times);
    //    maggiePrint("rep_pt:x:%i, y:%i", rep_pt.x, rep_pt.y);

    timer.reset();
    for (int i = 0; i < nb_times; ++i)
        rep_pt = image_utils::centroidOfMonochromeImage2(image_sample);
    timer.printTime_factor("centroidOfMonochromeImage2", nb_times);
    maggiePrint("rep_pt:x:%i, y:%i", rep_pt.x, rep_pt.y);

    /*
     * trying with biggestComponent_image()
     */
    cv::Mat1b out = image_sample.clone();
    timer.reset();
    for (int iter = 0; iter < nb_times; ++iter)
        image_utils::biggestComponent_image2(image_sample, out);
    timer.printTime_factor("biggestComponent_image2", nb_times);

    maggiePrint("image saved in './out_biggestComponent_image.png'.");
    cv::imwrite("out_biggestComponent_image.png", out);

    /*
     * trying with biggestComponent_vector2()
     */
    std::vector<cv::Point> pts;
    timer.reset();
    for (int iter = 0; iter < nb_times; ++iter)
        image_utils::biggestComponent_vector2(image_sample, pts);
    timer.printTime_factor("biggestComponent_vector2", nb_times);
    maggiePrint("size:%i", (int) pts.size());

    /*
     * trying with the connectedComponents() function
     */
    std::vector<std::vector<cv::Point> > comps;
    std::vector<cv::Rect> bounding_boxes;
    timer.reset();
    for (int iter = 0; iter < nb_times; ++iter)
        image_utils::connectedComponents2(image_sample, comps, bounding_boxes);
    timer.printTime_factor("connectedComponents2", nb_times);
    maggiePrint("Number of components:%i", (int) bounding_boxes.size());

    /*
     * trying with the get_connected_components() in two steps
     */
    DisjointSets2 set;
    timer.reset();
    for (int iter = 0; iter < nb_times; ++iter) {
        set.process_image(image_sample);
        set.get_connected_components(image_sample.cols, comps, bounding_boxes);
    }
    timer.printTime_factor("get_connected_components()", nb_times);
    maggiePrint("Number of components:%i", (int) bounding_boxes.size());

    /*
     * cvBlob
     */
    timer.reset();
    IplImage* label_img = cvCreateImage(image_sample.size(), 32, 1);
    cvb::CvBlobs blobs;
    IplImage image_sample_ipl = image_sample, label_img_ipl = *label_img;
    maggiePrint("depth:%i (== %i ?), %i channels",
                image_sample_ipl.depth, IPL_DEPTH_LABEL,
                image_sample_ipl.nChannels);
    for(int i = 0 ; i < nb_times ; ++i)
        //unsigned int nb_pixels =
        cvLabel(&image_sample_ipl, &label_img_ipl, blobs);
    timer.printTime_factor("cvLabel()", nb_times);
    maggiePrint("Number of components:%i", (int) blobs.size());
    cvb::CvLabel label = cvGreaterBlob(blobs);
    maggiePrint("biggest blob:%i, area:%i",
                label, blobs.at(label)->area);

    // blobs
    CBlobResult blobs2;
    // Extract the blobs using a threshold of 100 in the image
    timer.reset();
    for(int i = 0 ; i < nb_times ; ++i)
        blobs2 = CBlobResult( &image_sample_ipl, NULL, 100, false);
    timer.printTime_factor("CBlobResult()", nb_times);
    // object with the blob with most perimeter in the image
    // ( the criteria to select can be any class derived from COperadorBlob )
    CBlob blobWithBiggestPerimeter;
    CBlob blobWithLessArea;
    // from the filtered blobs, get the blob with biggest perimeter
    blobs2.GetNthBlob( CBlobGetPerimeter(), 0, blobWithBiggestPerimeter );



    /* display each point */
    // for (std::vector<std::vector<cv::Point> >::iterator it = rep.begin(); it < rep.end(); it++) {
    // for (std::vector<cv::Point>::iterator it2 = it->begin(); it2 < it->end(); it2++) {
    // maggiePrint("(" << it2->x << "," << it2->y << ") ";
    // }
    // maggiePrint(endl;
    // }

    /* display each bounding box */
    // for (std::vector<cv::Rect>::iterator it = bounding_boxes.begin(); it < bounding_boxes.end(); it++)
    // maggiePrint("(" << it->x << ", " << it->y << ")" << "->("
    // << it->x + it->width << ", " << it->y + it->height << ") ";
}

////////////////////////////////////////////////////////////////////////////////

void test_redimContent() {
    maggiePrint("test_redimContent()");
    cv::Mat1b test = cv::imread(IMG_DIR  "balloonBW.png" , CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat1b test_roi;
    test(cv::Rect(50, 50, 150, 150)).copyTo(test_roi);

    // cv::Mat test0 = cv::imread(IMG_DIR "rectangles.png" , CV_LOAD_IMAGE_GRAYSCALE);
    // cv::Mat test_roi ( test0, cv::Rect(2, 2, 8, 8));

    /* std::vector version */
    std::vector<cv::Point> comp, comp_resized;

    Timer timer;
    image_utils::biggestComponent_vector2(test_roi, comp);
    timer.printTime("biggestComponent_vector2()");

    timer.reset();
    image_utils::redimContent_vector_without_repetition
            (comp, comp_resized, 150, 150, 200, 200);
    // for (int i = 0; i < comp.size(); ++i)
    // maggiePrint("(" << comp.at(i).x << ", " << comp.at(i).y << ") ";
    // for (int i = 0; i < comp_resized.size(); ++i)
    // maggiePrint("(" << comp_resized.at(i).x << ", " << comp_resized.at(i).y << ") ";
    timer.printTime("redimContent_vector_without_repetition");

    cv::Mat3b out (300, 300);
    out.setTo(0);
    image_utils::drawListOfPoints(out, comp_resized, cv::Vec3b(255, 0 ,0));
    image_utils::drawListOfPoints(out, comp, cv::Vec3b(0, 255 , 0));
    cv::imshow("out", out);
    cv::waitKey(0);
    cv::imwrite("out.png", out);
}

////////////////////////////////////////////////////////////////////////////////

void test_redimContent2() {
    maggiePrint("test_redimContent2()");
    cv::Mat1b test(300, 300);
    test = (uchar) 0;
    int circle_diam = 75;
    cv::circle(test, cv::Point(150, 150), circle_diam, cv::Scalar(255), -1);

    // get the circle
    std::vector<cv::Point> comp;
    image_utils::biggestComponent_vector2(test, comp);

    Timer timer;
    std::vector<cv::Point> comp_resized;
    int occur_max = 8;
    for (int occur = 1; occur <= occur_max; ++occur) {
        int new_width = circle_diam * occur / occur_max;
        int new_height = new_width / 2;
        int x = test.cols * occur / occur_max - new_width / 2;
        image_utils::redimContent_vector_without_repetition
                (comp, comp_resized, x, 50, x + new_width, 50 + new_height);
        timer.printTime("redimContent_vector_without_repetition");
        image_utils::drawListOfPoints(test, comp_resized,
                                     (uchar) (255 * occur / occur_max));
    }
    cv::imshow("test", test);
    cv::waitKey(0);
    cv::imwrite("test.png", test);
}

//////////////////////////////////////////////////////////////////////////////

int main() {
    maggiePrint("main()");

    maggiePrint("  1:test_conn()");
    maggiePrint("  2:test_redimContent()");
    maggiePrint("  3:test_redimContent2()");

    int choice;
    std::cin >> choice;

    if (choice == 1) {
        test_conn(IMG_DIR "rectangles.png");
        test_conn(IMG_DIR "star.png");
        test_conn(IMG_DIR "balloonBW.png");
        //test_conn(IMG_DIR "frame_thres_sample2.png");
        test_conn(IMG_DIR "pz/pz01_thres.png");
    }
    else if (choice == 2)
        test_redimContent();
    else if (choice == 3)
        test_redimContent2();
}
