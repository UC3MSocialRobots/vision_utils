// opencv
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/all_files_in_dir.h"
#include "vision_utils/cast_from_string.h"
#include "vision_utils/timer.h"
#include "vision_utils/opencv_face_detector.h"
#include <iomanip>

void detect_one_pic(const std::string filename,
                    int resize_max_width = vision_utils::DEFAULT_RESIZE_MAX_WIDTH,
                    int resize_max_height = vision_utils::DEFAULT_RESIZE_MAX_HEIGHT,
                    double scale_factor = vision_utils::DEFAULT_SCALE_FACTOR,
                    int min_neighbors = vision_utils::DEFAULT_MIN_NEIGHBORS,
                    int min_width = vision_utils::DEFAULT_MIN_WIDTH) {
  printf("detect_one_pic('%s', scale_factor:%g, min_neighbors:%i, min_width:%i)\n",
         filename.c_str(), scale_factor, min_neighbors, min_width);

  // load the imagez
  cv::Mat3b img_gp = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  cv::Mat3b small_img_gp;
  std::vector<cv::Rect> res;
  // create a classifier
  cv::CascadeClassifier classifier = vision_utils::create_face_classifier();
  // make the real detection
  int ntimes = 2;
  vision_utils::Timer timer;
  for (unsigned int time_idx = 0; time_idx < 10; ++time_idx)
    vision_utils::detect_with_opencv
        (img_gp, classifier, small_img_gp, res,
         resize_max_width, resize_max_height, scale_factor, min_neighbors, min_width);
  timer.printTime_factor("detect_with_opencv", ntimes);
  // draw them
  printf("Found %li faces!\n", res.size());
  for (unsigned int idx = 0; idx < res.size(); ++idx) {
    cv::Rect* curr_elt = &(res[idx]);
    cv::rectangle(img_gp, *curr_elt, CV_RGB(255, 0, 0), 2);
  } // end loop idx

  cv::imshow("img_gp", img_gp);
  cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

void detect_save_all_files_in_folder(const std::string & folder) {
  std::vector<std::string> files;
  vision_utils::all_files_in_dir(folder, files, "", true);
  cv::Mat3b small_img;
  std::vector<cv::Rect> rects;
  cv::CascadeClassifier classifier = vision_utils::create_face_classifier();
  int face_counter = 1;

  for (unsigned int file_idx = 0; file_idx < files.size(); ++file_idx) {
    if (file_idx % 50 == 0)
      printf("detect_save_all_files_in_folder(): read %i files out of %li\n",
             file_idx, files.size());

    cv::Mat3b frame = cv::imread(files[file_idx]);
    if (frame.empty()) {
      printf("Could not read file '%s'\n", files[file_idx].c_str());
      continue;
    }
    vision_utils::detect_with_opencv(frame, classifier, small_img, rects);
    for (unsigned int rec_idx = 0; rec_idx < rects.size(); ++rec_idx) {
      std::ostringstream outfilename;
      outfilename << "/tmp/face"
                  << std::setw(3) << std::setfill('0') << face_counter << ".png";
      cv::imwrite( outfilename.str(), frame( rects[rec_idx] ));
      printf("Saved face in '%s'\n", outfilename.str().c_str());
      ++face_counter;
    } // end loop rec_idx
  } // end loop file_idx
} // end detect_save_all_files_in_folder();

////////////////////////////////////////////////////////////////////////////////

void test_camera() {
  // init data
  cv::VideoCapture capture(0);
  //  capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  //  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 800);
  cv::Mat3b frame;
  cv::Mat3b small_img;
  std::vector<cv::Rect> res;
  // create a classifier
  cv::CascadeClassifier classifier = vision_utils::create_face_classifier();

  while (capture.isOpened()) {
    capture >> frame;
    vision_utils::Timer timer;
    // make the real detection
    vision_utils::detect_with_opencv(frame, classifier, small_img, res);
    timer.printTime("detect_with_opencv()");
    // draw them
    for (unsigned int idx = 0; idx < res.size(); ++idx) {
      cv::Rect* curr_elt = &(res[idx]);
      cv::rectangle(frame, *curr_elt, CV_RGB(255, 0, 0), 2);
    } // end loop idx

    cv::imshow("frame", frame);
    cv::waitKey(5);
  } // end while (capture.isOpened)
} // end test_camera();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  int idx = 1;
  if (argc < 2) {
    printf("%i: detect_one_pic(IMG_DIR group.jpg)\n", idx++);
    printf("%i: detect_one_pic(IMG_DIR arnaud001.png)\n", idx++);
    printf("%i: detect_one_pic(argv[2])\n", idx++);
    printf("%i: test_camera()\n", idx++);
    printf("%i: detect_save_detect_save_all_files_in_folder(/home/user/Downloads/0datasets/google_genders/man)\n", idx++);
    return -1;
  }

  int choice = 0;
  choice = atoi(argv[1]);

  idx = 1;
  if (choice == idx++)
    detect_one_pic(vision_utils::IMG_DIR() + "group.jpg");
  else if (choice == idx++)
    detect_one_pic(vision_utils::IMG_DIR() + "arnaud001.png");
  else if (choice == idx++) {
    if (argc < 3) {
      printf("Please specify: file, scale_factor, min_neighbors, min_width!\n");
      return -1;
    }
    else if (argc < 8)
      detect_one_pic(argv[2]);
    else
      detect_one_pic(argv[2],
          vision_utils::cast_from_string<int>(argv[3]),
          vision_utils::cast_from_string<int>(argv[4]),
          vision_utils::cast_from_string<double>(argv[5]),
          vision_utils::cast_from_string<int>(argv[6]),
          vision_utils::cast_from_string<int>(argv[7]));
  }
  else if (choice == idx++)
    detect_one_pic(vision_utils::IMG_DIR() + "depth/albertopng");
  else if (choice == idx++)
    test_camera();
  else if (choice == idx++)
    detect_save_all_files_in_folder("/home/user/Downloads/0datasets/google_genders/man");
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
