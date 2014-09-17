
#include "vision_utils/xml_images_reader/xml_images_reader.h"

// implementations
#include "cv_blob_wrapper.h"
#include "cblob_wrapper.h"
#include "disjoint_sets2.h"
#include "cv_flood_fill_wrapper.h"

// other AD includes
#include "time/timer.h"
#include <vision_utils/img_path.h>
// third parties
#include "opencv2/highgui/highgui.hpp"
// C++
#include <vector>
#include <iostream>

class ComponentBenchmark : public XmlImagesReader {
public:
  // stuff that should be overloaded
  ////////////////////////////////////////////////////////////////////////////
  //! load from a given xml XmlDocument::Node
  void from_xml_node_custom(const XmlDocument & doc, XmlDocument::Node* node) {
  }

  //! replace the content of a XmlDocument::Node with the new data
  void to_xml_node_custom(XmlDocument & doc, XmlDocument::Node* node) const {
  }
  ////////////////////////////////////////////////////////////////////////////

  template<class _Interface>
  void test_interface(_Interface & interface, double & time) {
    timer.reset();
    interface.process_image(frame_thres);
    interface.get_connected_components(frame_thres.cols, comps, bounding_boxes);
    maggiePrint("Number of components:%i", (int) bounding_boxes.size());
    update_timer_ntimes(time, _nb_files_processed, timer.time());
  }


  //! make the processing
  void start() {
    maggieDebug2("start()");

    // process all the files
    _nb_files_processed = 0;
    time_disjoint_sets = 0;
    time_cblob = 0;
    time_cv_blob = 0;

    while (_nb_files_processed <= get_nb_files()) {
      ++_nb_files_processed;

      // threshold image
      cv::cvtColor(*get_current_cv_img(), frameBW, CV_BGR2GRAY); // grayscale conversion
      cv::adaptiveThreshold(frameBW, frame_thres, 255,
                            //cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                            cv::ADAPTIVE_THRESH_MEAN_C,
                            cv::THRESH_BINARY_INV,
                            91,
                            -2.f);

      // test it
      test_interface(interface_disjoint_sets, time_disjoint_sets);
      test_interface(interface_flood_fill, time_flood_fill);
      test_interface(interface_cv_blob, time_cv_blob);
      test_interface(interface_cblob, time_cblob);

      maggiePrint("%i files, time_disjoint_sets:%g, \ttime_flood_fill:%g, "
                  "\ttime_cv_blob:%g, \ttime_c_blob:%g",
                  _nb_files_processed,
                  time_disjoint_sets,
                  time_flood_fill,
                  time_cv_blob,
                  time_cblob);

      //cv::imshow("input", *get_current_cv_img()); cv::imshow("get_playzone", *_playzone_finder.get_playzone()); cv::waitKey(5000);
      //maggiePrint("waiting for keyboard return..."); std::cin.ignore();

      go_to_next_file();
    }
  }

protected:
  int _nb_files_processed;
  Timer timer;
  cv::Mat1b frameBW;
  cv::Mat1b frame_thres;

  DisjointSets2 interface_disjoint_sets;
  CvFloodFillWrapper interface_flood_fill;
  CvBlobWrapper interface_cv_blob;
  CBlobWrapper interface_cblob;

  double time_disjoint_sets;
  double time_flood_fill;
  double time_cv_blob;
  double time_cblob;

  std::vector<std::vector<cv::Point> > comps;
  std::vector<cv::Rect> bounding_boxes;
};

////////////////////////////////////////////////////////////////////////////////

template<class _Interface>
void test_interface(cv::Mat1b img) {
  std::vector<std::vector<cv::Point> > comps;
  std::vector<cv::Rect> bounding_boxes;

  Timer timer;
  int nb_times = 10;
  _Interface set;
  for (int iter = 0; iter < nb_times; ++iter) {
    set.process_image(img);
    set.get_connected_components(img.cols, comps, bounding_boxes);
  }
  timer.printTime_factor("get_connected_components()", nb_times);
  maggiePrint("Number of components:%i", (int) bounding_boxes.size());


  /* display each point */
  //for (std::vector<std::vector<cv::Point> >::iterator comp = comps.begin();
  //     comp < comps.end(); comp++) {
  //    maggiePrint("%s", StringUtils::accessible_to_string(*comp).c_str());
  //}

  /* display each bounding box */
  // for (std::vector<cv::Rect>::iterator it = bounding_boxes.begin(); it < bounding_boxes.end(); it++)
  // maggiePrint("(" << it->x << ", " << it->y << ")" << "->("
  // << it->x + it->width << ", " << it->y + it->height << ") ";
}

void test_interface(std::string filename) {
  std::cout << std::endl << std::endl;
  maggiePrint("test_interface(%s)", filename.c_str());
  cv::Mat1b img = cv::imread( filename, CV_LOAD_IMAGE_GRAYSCALE);

  maggiePrint("DisjointSets2");
  test_interface<DisjointSets2>(img);
  maggiePrint("CvBlobWrapper");
  test_interface<CvBlobWrapper>(img);
  maggiePrint("CBlobWrapper");
  test_interface<CBlobWrapper>(img);
  maggiePrint("CvFloodFillWrapper");
  test_interface<CvFloodFillWrapper>(img);
}

////////////////////////////////////////////////////////////////////////////////

void test_aux_functions(std::string filename) {
  std::cout << std::endl << std::endl;
  maggiePrint("test_aux_functions(%s)", filename.c_str());
  cv::Mat1b img = cv::imread( filename, CV_LOAD_IMAGE_GRAYSCALE);
  Timer timer;
  int nb_times = 1;

  /*
     * trying with centroidOfMonochromeImage()
     */
  cv::Point rep_pt;
  timer.reset();
  for (int i = 0; i < nb_times; ++i) {
    DisjointSets2 set(img);
    rep_pt = set.centroidOfMonochromeImage(img.cols);
  }
  timer.printTime_factor("centroidOfMonochromeImage2", nb_times);
  maggiePrint("rep_pt:x:%i, y:%i", rep_pt.x, rep_pt.y);

  /*
     * trying with biggestComponent_image()
     */
  cv::Mat1b out = img.clone();
  timer.reset();
  for (int iter = 0; iter < nb_times; ++iter) {
    DisjointSets2 set(img);
    set.biggestComponent_image(img.cols, out);
  }
  timer.printTime_factor("biggestComponent_image2", nb_times);

  maggiePrint("image saved in './out_biggestComponent_image.png'.");
  cv::imwrite("out_biggestComponent_image.png", out);

  /*
     * trying with biggestComponent_vector2()
     */
  std::vector<cv::Point> biggest_pts;
  timer.reset();
  for (int iter = 0; iter < nb_times; ++iter) {
    DisjointSets2 set(img);
    set.biggestComponent_vector(img.cols, biggest_pts);
  }
  timer.printTime_factor("biggestComponent_vector2", nb_times);
  maggiePrint("size:%i", (int) biggest_pts.size());

  /* display each point of the biggest component */
  //    for(std::vector<cv::Point>::const_iterator pt = biggest_pts.begin();
  //        pt != biggest_pts.end() ; ++pt) {
  //        maggiePrint("%i, %i", pt->x, pt->y);
  //    } // end loop pt
}

//////////////////////////////////////////////////////////////////////////////

int main() {
  maggiePrint("main()");

  maggiePrint("  1:test_interface()");
  maggiePrint("  2:test_aux_functions()");
  maggiePrint("  3:ComponentBenchmark");

  int choice;
  std::cin >> choice;
  //choice = 1;

  if (choice == 1) {
    test_interface(IMG_DIR "rectangles-big.png");

    //test_interface(IMG_DIR "rectangles.png");
    //        test_interface(IMG_DIR "star.png");
    //        test_interface(IMG_DIR "balloonBW.png");
    //        test_interface(IMG_DIR "pz/pz01_thres.png");
    //        test_interface(IMG_DIR "pz/pz40_thres.png");
  }
  else if (choice == 2) {
    test_aux_functions(IMG_DIR "rectangles.png");
    test_aux_functions(IMG_DIR "star.png");
    test_aux_functions(IMG_DIR "balloonBW.png");
    test_aux_functions(IMG_DIR "pz/pz01_thres.png");
    test_aux_functions(IMG_DIR "pz/pz40_thres.png");
  }
  else if (choice == 3){
    ComponentBenchmark bench;
    bench.from_xml_file(IMG_DIR "pz/", "pz.xml");
    bench.start();
  }
}
