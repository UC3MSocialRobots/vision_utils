/*!
 * \file test_CameraUtils.cpp
 *
 * tests for the camera
 *
 * \date 10/11/2010
 * \author Arnaud Ramey
 */

////// my imports
#include "camera_utils/camera_utils.h"
#include <ros/package.h>

#include <vision_utils/img_path.h>

inline int get_camera_nb_prompt() {
  std::cout << "Camera index ? (0=default)" << std::endl;
  int camera_nb = 0;
  std::cin >> camera_nb;
  return camera_nb;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  printf("grabbing the camera...\n");
  std::cout << "1 : camera test mode (OpenCV 2.1 style)" << std::endl;
  std::cout << "2 : photo camera mode" << std::endl;
  std::cout << "3 : video recorder mode" << std::endl;
  std::cout << "4 : video player mode" << std::endl;
  std::cout << "5 : three-layers camera test mode" << std::endl;
  std::cout << "6 : rectify image" << std::endl;
  std::cout << "7 : rectify image ROS" << std::endl;

  std::string camera_path = ros::package::getPath("kinect") + "/data/";

  int choice = 7;
  std::cin >> choice;

  /* camera tests */

  if (choice == 1) {
    CameraUtils::cameraTester_OpenCV2(get_camera_nb_prompt());
  }

  if (choice == 2) {
    cv::VideoCapture capture;
    CameraUtils::getCamera(get_camera_nb_prompt(), capture);
    CameraUtils::photo_camera(capture, "pic", 1);
  }

  if (choice == 3) {
    const char* filename =
        (argc < 2 || strlen(argv[1]) == 0 ? "video_capture.avi"
                                          : argv[1]);
    std::cout << "input:" << filename << std::endl;
    cv::VideoCapture capture;
    CameraUtils::getCamera(get_camera_nb_prompt(), capture);
    CameraUtils::cameraRecorder(capture, filename);
  }

  if (choice == 4) {
    const char* filename =
        (argc < 2 || strlen(argv[1]) == 0 ? "video_capture.avi"
                                          : argv[1]);
    std::cout << "input:" << filename << std::endl;
    CameraUtils::videoPlayer(filename);
  }

  if (choice == 5) {
    cv::VideoCapture capture;
    CameraUtils::getCamera(get_camera_nb_prompt(), capture);
    CameraUtils::showThreeLayers(capture);
  }

  if (choice == 6) {
    cv::VideoCapture capture;
    CameraUtils::getCamera(get_camera_nb_prompt(), capture);
    CameraUtils::rectify_image_skeleton
        (capture, camera_path + "Logitech_QuickCam_Arnaud.yml");
  }

  if (choice == 7) {
    cv::VideoCapture capture;
    CameraUtils::getCamera(get_camera_nb_prompt(), capture);
    CameraUtils::rectify_image_ros
        (capture, camera_path + "Logitech_QuickCam_Arnaud.yml");
  }

  return 0;
}

