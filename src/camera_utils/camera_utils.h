#ifndef CAMERAUTILS_H
#define CAMERAUTILS_H

/*!
 * \file camera_utils.h
 *
 * Some useful functions for using a camera
 *
 * \date 10/11/2010
 * \author Arnaud Ramey
 */

///// opencv includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

///// STL imports
#include <stdio.h>
#include <iostream>         // for cin, cout
#include <string>           // for strings
#include <iomanip>          // for setw : leading zeros
#include <sstream>          // for std::sstreams
#include <vector>           // for vectors

#include <ros/ros.h>
#include <ros_utils/marker_utils.h>
#include <ros_utils/pt_utils.h>

/** some useful functions for handling images */
namespace CameraUtils {

inline void get_video_capture_param(cv::VideoCapture & capture,
                                    const std::string & name,
                                    const int & param_idx) {
  std::cout << std::setw(30) << name << " : "
            << capture.get(param_idx) << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void getCamera(int camera_nb, cv::VideoCapture & capture) {
  printf("getCamera(%i)\n", camera_nb);
  capture.open(camera_nb);
  assert(capture.isOpened());

  double f = capture.get(CV_CAP_PROP_FOURCC);
  char* fourcc = (char*) (&f);
  printf(" -> 4-character code of codec : '%s' ( = '%f' )\n", fourcc, f);

  get_video_capture_param(capture, "CV_CAP_PROP_FPS", CV_CAP_PROP_FPS);
  get_video_capture_param(capture, "CV_CAP_PROP_FRAME_WIDTH", CV_CAP_PROP_FRAME_WIDTH);
  get_video_capture_param(capture, "CV_CAP_PROP_FRAME_HEIGHT", CV_CAP_PROP_FRAME_HEIGHT);
  get_video_capture_param(capture, "CV_CAP_PROP_BRIGHTNESS", CV_CAP_PROP_BRIGHTNESS);
  get_video_capture_param(capture, "CV_CAP_PROP_CONTRAST", CV_CAP_PROP_CONTRAST);
  get_video_capture_param(capture, "CV_CAP_PROP_SATURATION", CV_CAP_PROP_SATURATION);
  get_video_capture_param(capture, "CV_CAP_PROP_HUE", CV_CAP_PROP_HUE);
  get_video_capture_param(capture, "CV_CAP_PROP_AUTO_EXPOSURE", CV_CAP_PROP_AUTO_EXPOSURE);
  get_video_capture_param(capture, "CV_CAP_PROP_TEMPERATURE", CV_CAP_PROP_TEMPERATURE);

  //  printf(" -> contrast   of image       : %f\n", capture.get(CV_CAP_PROP_CONTRAST));
  //  printf(" -> saturation of image       : %f\n", capture.get(CV_CAP_PROP_SATURATION));
  //  printf(" -> hue        of image       : %f\n", capture.get(CV_CAP_PROP_HUE));
}

////////////////////////////////////////////////////////////////////////////////

void cameraTester_OpenCV2(int camera_nb) {
  printf("\n***\n*** Camera tester OpenCV2 - [ESC]=quit\n***\n");
  cv::VideoCapture cap(camera_nb); // open the default camera
  if (!cap.isOpened()) // check if we succeeded
    return;

  cv::Mat edges;
  cv::namedWindow("edges", 1);
  for (;;) {
    cap >> edges; // get a new frame from camera
    cv::imshow("edges", edges);

    if (cv::waitKey(20) >= 0)
      break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return;
}

////////////////////////////////////////////////////////////////////////////////

/** a minimal aparato (photo camera) using the camera */
void photo_camera(cv::VideoCapture & capture,
                   const std::string & filename_radix = "pic", int nbPhoto = 1) {
  printf("\n***\n*** Simple camera - "
         "[SPACE]=make photos, [ESC]=quit. Filename radix='%s'\n***\n",
         filename_radix.c_str() );
  bool showPhoto = 0;
  int keyPressed;
  const std::string windowName = "simpleAparato";
  cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);

  cv::Mat3b frame;
  //cv::Mat lastPhoto;
  capture >> frame;

  while (1) {
    if (!showPhoto) { // acquire a frame from the camera
      capture >> frame;
    }

    cv::imshow(windowName, frame);
    keyPressed = cv::waitKey(33);

    if ((keyPressed & 255) == 32) { // SPACE pressed
      if (showPhoto == 0) { // take a photo
        /// creating filename
        std::ostringstream filename_sstream;
        filename_sstream << filename_radix << std::setw(3)
                         << std::setfill('0') << nbPhoto << ".png";
        const std::string & filename = filename_sstream.str();
        printf("Saving pic '%s'", filename.c_str());

        /// saving picture
        cv::imwrite(filename, frame);
        nbPhoto++;
      }
      showPhoto = !showPhoto;
    }
    if ((keyPressed & 255) == 27)
      break; // ESC pressed
  }

  capture.release();
  cvDestroyAllWindows();
}

////////////////////////////////////////////////////////////////////////////////

void cameraSkeleton(cv::VideoCapture & capture) {
  cv::Mat3b frame;
  const std::string & windowName = "cameraSkeleton";
  cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);

  while (1) {
    capture >> frame;
    //if (!frame)
    //break;
    cv::imshow(windowName, frame);
    char c = cv::waitKey(33);
    if (c == 27)
      break;
  }

  capture.release();
  cvDestroyAllWindows();
}

////////////////////////////////////////////////////////////////////////////////

/** a simple video recorder */
void cameraRecorder(cv::VideoCapture & capture, const std::string & outputFilename =
    "../images/video_capture.avi") {
  printf("\n***\n*** Simple video recorder "
         "- [SPACE] to start/stop capture, "
         "[ESC]=quit. outputFilename='%s'\n***\n", outputFilename.c_str() );

  /** initialize the video writer */
  const std::string & windowName = "cameraRecorder";
  cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
  cv::Mat3b frame;
  capture >> frame;
  bool recording = 0;
  cv::VideoWriter writer(
        outputFilename, //
        //-1,
        //CV_FOURCC('D','I','V','X'),
        //CV_FOURCC('P','I','M','1'),    //= MPEG-1 codec
        //          CV_FOURCC('M','J','P','G'),    //= motion-jpeg codec (does not work well)
        CV_FOURCC('M', 'P', '4', '2'), //= MPEG-4.2 codec
        //CV_FOURCC('D', 'I', 'V', '3'), //= MPEG-4.3 codec
        //CV_FOURCC('D', 'I', 'V', 'X'), //= MPEG-4 codec
        //CV_FOURCC('U', '2', '6', '3'), //= H263 codec
        //CV_FOURCC('I', '2', '6', '3'), //= H263I codec
        //CV_FOURCC('F', 'L', 'V', '1'), //= FLV1 codec
        10, frame.size());

  while (1) {
    capture >> frame;
    //        if (!frame)
    //            break;

    if (recording) {
      // printf("frame:%s\n", image_utils::infosImage(frame).c_str());
      writer << frame; // save image
      cv::circle(frame, cv::Point(frame.cols - 30, 30), 2,
                 CV_RGB(255, 0, 0), 10);
    }

    cv::imshow(windowName, frame);

    char c = cv::waitKey(33);
    if ((c & 255) == 32) { // SPACE pressed
      recording = !recording;
    }
    if (c == 27)
      break; // ESC pressed
  }

  capture.release();
  cvDestroyAllWindows();
}

////////////////////////////////////////////////////////////////////////////////

void videoPlayer( const std::string & filename) {
  printf("videoPlayer('%s')\n", filename.c_str());

  const std::string & windowName = "videoPlayer";
  cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
  cv::VideoCapture capture (filename);
  cv::Mat3b frame;

  while (1) {
    if (capture.grab() == false) {
      printf("Video finished\n");
    }
    else {
      capture.retrieve(frame, 0);
      cv::imshow(windowName, frame);
    }

    char c = cv::waitKey(33);
    //printf("%i\n", (int) c);
    if (c == 52 || c == 81) // left
      capture.set(CV_CAP_PROP_POS_FRAMES, -100 + capture.get(CV_CAP_PROP_POS_FRAMES));
    if (c == 54 || c == 83) // right
      capture.set(CV_CAP_PROP_POS_FRAMES, +100 + capture.get(CV_CAP_PROP_POS_FRAMES));
    if (c == 27)
      break;
  }

  capture.release();
  cvDestroyAllWindows();
}

////////////////////////////////////////////////////////////////////////////////

void showThreeLayers(cv::VideoCapture & capture) {
  cv::Mat3b frame;
  capture >> frame;
  cv::Mat3b hsv = cv::Mat3b(frame.rows, frame.cols);
  cv::Mat1b H(frame.rows, frame.cols);
  cv::Mat1b S(frame.rows, frame.cols);
  cv::Mat1b V(frame.rows, frame.cols);

  cv::namedWindow("H", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("S", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("V", CV_WINDOW_AUTOSIZE);
  //cv::namedWindow( "HueRGB", CV_WINDOW_AUTOSIZE );

  while (1) {
    capture >> frame;
    //        if (!frame)
    //            break;
    cv::cvtColor(frame, hsv, CV_BGR2HSV);

    std::vector<cv::Mat1b> channels;
    channels.push_back(H);
    channels.push_back(S);
    channels.push_back(V);
    cv::split(hsv, channels);

    //color_utils::rgb_saturate_saturation_value(frame, hueRGB);

    cv::equalizeHist(H, H);
    cv::equalizeHist(S, S);
    cv::equalizeHist(V, V);

    cv::imshow("H", H);
    cv::imshow("S", S);
    cv::imshow("V", V);
    //cv::imshow( "HueRGB", H );
    char c = cv::waitKey(33);
    if (c == 27)
      break;
  }

  capture.release();
  cvDestroyAllWindows();
}

////////////////////////////////////////////////////////////////////////////////

/*! Load the calibration parameters from a calibrated camera.
    Calibration file can be obtained with OpenCV sample "calibrate"
    with an instruction such as:
    'bin/calibration -w 7 -h 5 -s 0.031 -d 3000 -o camera.yml -op -oe'
 \param calibration_filename
    the file containing the intrinsic and extrinsic parameters of the camera
 \param mapx, mapy [out]
  the maps for the inverse mapping algorithm that is used by remap() .
  That is, for each pixel  in the destination (corrected and rectified) image,
  the function computes the corresponding coordinates in the source image
  (that is, in the original image from camera).
 \param intrinsics [out]
  the intrinsic parmeters of the camera.
 \param distortion [out]
  the distortion coefficients of the camera.
 \param intrinsics_inv [out]
  the inverse matrix of intrinsics. Useful for reprojecting 2D points.
*/
inline void read_calibration_file(const std::string & calibration_filename,
                                  cv::Mat3f & mapx, cv::Mat3f & mapy,
                                  cv::Mat & intrinsics,
                                  cv::Mat & distortion,
                                  cv::Mat & intrinsics_inv) {
  // Read camera parameters from XML/YAML file:
  cv::FileStorage fs(calibration_filename, cv::FileStorage::READ);
  int image_width, image_height;
  fs["camera_matrix"] >> intrinsics;
  fs["distortion_coefficients"] >> distortion;
  fs["image_width"] >> image_width;
  fs["image_height"] >> image_height;
  fs.release();

  // Computes the undistortion and rectification transformation map.
  // cf http://docs.opencv.org/trunk/modules/imgproc/doc/geometric_transformations.html#initundistortrectifymap
  cv::Mat newCameraMatrix;
  cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), newCameraMatrix,
                              cv::Size(image_width, image_height), CV_32FC1,
                              mapx, mapy);

  intrinsics_inv = intrinsics.inv();
} // end read_calibration_file();

////////////////////////////////////////////////////////////////////////////////

/*!
 Get the vector of the line passing through optical center and (pixel_x, pixel_y)
 \param intrinsics_inv
    the inverse of the intrinsic parameters matrix of our camera
 \param pixel_x, pixel_y
    the point where we want to get the 3D line
 \return cv::Vec3f
    the vector of this line. Careful, it is not normalized.
*/
cv::Vec3f pixel2world_line_vector(const cv::Mat & intrinsics_inv,
                                  const int & pixel_x, const int & pixel_y) {
  // world = intrinsics_inv * pixel
#if 0 // normalize the vector
  cv::Mat world = intrinsics_inv * (cv::Mat_<double>(3,1) << pixel_x, pixel_y, 1);
  double x = world.at<double>(0, 0), y = world.at<double>(1, 0), z = world.at<double>(2, 0);
  double norm = sqrt(x * x + y * y + z * z);
  return cv::Vec3f(x / norm, y / norm, z / norm);
#else
  return cv::Vec3f((cv::Mat)(intrinsics_inv * (cv::Mat_<double>(3,1) << pixel_x, pixel_y, 1)));
#endif
}

////////////////////////////////////////////////////////////////////////////////

/*!
  A version of pixel2world_line_vector() for a cv::Point.
  \see pixel2world_line_vector()
*/
cv::Vec3f pixel2world_line_vector(const cv::Mat & intrinsics_inv, const cv::Point & pixel) {
  return pixel2world_line_vector(intrinsics_inv, pixel.x, pixel.y);
}

////////////////////////////////////////////////////////////////////////////////


/*!
  Reproject a pixel to the 3D point that corresponds
  and is located at a given distance of the camera.
 \param intrinsics_inv
    the inverse of the intrinsic parameters matrix of our camera
 \param x, y
    the pixel in the image
 \param dist
    a distance in meters
 \return cv::Point3f
    the 3D point that corresponds to (x, y)
    and is located at dist from the camera optical center.
*/
cv::Point3f pixel2world_known_distance(const cv::Mat & intrinsics_inv,
                                       const int & x, const int & y, const double & dist) {
  // get line vector
  cv::Vec3f line_vector = pixel2world_line_vector(intrinsics_inv, x, y);
  double mult_fac = dist / line_vector[2]; // multiplicative factor
  return cv::Point3f(line_vector[0] * mult_fac, line_vector[1] * mult_fac, dist);
}

////////////////////////////////////////////////////////////////////////////////

/*!
  A version of pixel2world_known_distance() for a cv::Point.
  \see pixel2world_known_distance()
*/
cv::Point3f pixel2world_known_distance(const cv::Mat & intrinsics_inv,
                                       const cv::Point & pixel, const double & dist) {
  return pixel2world_known_distance(intrinsics_inv, pixel.x, pixel.y, dist);
}

////////////////////////////////////////////////////////////////////////////////

/*!
  Project a 3D point into the image.
 \param intrinsics
    the intrinsic parameters matrix of our camera
 \param x, y, z
    the 3D point
 \return cv::Point
    the projection of (x, y, z) into our camera frame.
*/
cv::Point world2pixel(const cv::Mat & intrinsics,
                      const double & x, const double & y, const double & z) {
  // pixel = intrinsics * world
  cv::Mat pixel_hom = intrinsics * (cv::Mat_<double>(3,1) << x, y, z);
  // std::cout << "pixel_hom:" << pixel_hom << std::endl;
  return cv::Point(pixel_hom.at<double>(0, 0) / pixel_hom.at<double>(2, 0),
                   pixel_hom.at<double>(1, 0) / pixel_hom.at<double>(2, 0));
}

////////////////////////////////////////////////////////////////////////////////

/*!
  A version of world2pixel() for a cv::Point3f.
  \see world2pixel()
*/
cv::Point world2pixel(const cv::Mat & intrinsics, const cv::Point3f & world) {
  return world2pixel(intrinsics, world.x, world.y, world.z);
}

////////////////////////////////////////////////////////////////////////////////

/*!
  A simple skeleton for rectifying an image.
 \param capture
    the access to the camera stream
 \param calibration_filename
    the file containing the intrinsic and extrinsic parameters of the camera
*/
void rectify_image_skeleton(cv::VideoCapture & capture,
                            const std::string & calibration_filename) {
  cv::Mat3f mapx, mapy;
  cv::Mat intrinsics, distortion, intrinsics_inv;
  read_calibration_file(calibration_filename,
                        mapx, mapy, intrinsics, distortion, intrinsics_inv);

  //  acquire image from cam, remap it and display both
  cv::Mat3b frame_raw, frame_remapped;
  cv::namedWindow("frame_raw");
  cv::namedWindow("frame_remapped");
  cv::Point test_pixel;
  bool init_test_pixel = true;
  while (true) {
    capture >> frame_raw; // acquire image
    if (frame_raw.empty()) {
      printf("Could not get image from capture device!\n");
      break;
    }
    // Undistort image
    cv::remap(frame_raw, frame_remapped, mapx, mapy, cv::INTER_LINEAR);

    // project 3D axes and draw them
    double arrow_length = .1;
    cv::line(frame_remapped,
             world2pixel(intrinsics, 0, 0, 1),
             world2pixel(intrinsics, arrow_length, 0, 1),
             CV_RGB(255, 0, 0), 2);
    cv::line(frame_remapped,
             world2pixel(intrinsics, 0, 0, 1),
             world2pixel(intrinsics, 0, arrow_length, 1),
             CV_RGB(0, 255, 0), 2);
    cv::line(frame_remapped,
             world2pixel(intrinsics, 0, 0, 1),
             world2pixel(intrinsics, arrow_length, arrow_length, 1 + arrow_length),
             CV_RGB(0, 0, 255), 2);

    // remap a random point
    if (init_test_pixel) { // set it to random values when needed
      init_test_pixel = false;
      test_pixel = cv::Point(rand() % frame_raw.cols, rand() % frame_raw.rows);
    }
    cv::Point3f p3D = pixel2world_known_distance(intrinsics_inv, test_pixel, 1);
    std::ostringstream txt; txt << "p:" << test_pixel << " -> p3D:" << p3D;
    cv::putText(frame_remapped, txt.str(), test_pixel, CV_FONT_HERSHEY_PLAIN,
                1, CV_RGB(0, 255, 0));

    // show resulting images
    cv::imshow("frame_raw", frame_raw);
    cv::imshow("frame_remapped", frame_remapped);
    char c = cv::waitKey(15);
    if ((int) c == 27)
      break;
    else if (c == ' ') // reset test_pixel to random values at next frame
      init_test_pixel = true;
  }
  capture.release();
} // end rectify_image();

////////////////////////////////////////////////////////////////////////////////

/*!
  A simple example for transforming the RGB stream of a camera
  into a ROS visualization_msgs::Marker
 \param capture
    the access to the camera stream
 \param calibration_filename
    the file containing the intrinsic and extrinsic parameters of the camera
*/
void rectify_image_ros(cv::VideoCapture & capture,
                       const std::string & calibration_filename) {
  int argc = 0;
  ros::init(argc, NULL, "rectify_image_ros");
  ros::NodeHandle nh_public;
  visualization_msgs::Marker marker;
  marker_utils::make_header(marker, visualization_msgs::Marker::POINTS,
                            "rectify_image_ros", .02, 1, 0, 0, 1);
  ros::Publisher marker_pub =
      nh_public.advertise<visualization_msgs::Marker>("rectify_image_ros_marker", 1);
  ROS_INFO("Publishing markers on '%s'", marker_pub.getTopic().c_str());

  cv::Mat3f mapx, mapy;
  cv::Mat intrinsics, distortion, intrinsics_inv;
  read_calibration_file(calibration_filename,
                        mapx, mapy, intrinsics, distortion, intrinsics_inv);

  //  acquire image from cam, remap it and display both
  cv::Mat3b frame_raw, frame_remapped;
  cv::namedWindow("frame_raw");
  cv::namedWindow("frame_remapped");

  double static_z = 1.f;

  while (true) {
    // printf("acquiring image...\n");
    capture >> frame_raw; // acquire image
    if (frame_raw.empty()) {
      printf("Could not get image from capture device!\n");
      break;
    }
    // Undistort image
    cv::remap(frame_raw, frame_remapped, mapx, mapy, cv::INTER_LINEAR);

    // rectify all pts
    int cols = frame_remapped.cols, rows = frame_remapped.rows;
    int data_step = 10, n_pts = rows * cols / data_step;
    marker.colors.clear();
    marker.colors.reserve(n_pts);
    marker.points.clear();
    marker.points.reserve(n_pts);
    for (int row = 0; row < rows; row += data_step) {
      // get the address of row
      uchar* data = frame_remapped.ptr<uchar>(row);
      for (int col = 0; col < cols; col += data_step) {
        if (data[3 * col] == 0 &&
            data[3 * col + 1] == 0 &&
            data[3 * col + 2] == 0)
          continue;
        // get color
        std_msgs::ColorRGBA color;
        color.r = data[3 * col + 2] / 255.f;
        color.g = data[3 * col + 1] / 255.f;
        color.b = data[3 * col    ] / 255.f;
        color.a = 1;
        marker.colors.push_back(color);
        // get 3D point
        cv::Point3f pt = pixel2world_known_distance(intrinsics_inv, col, row, static_z);
        geometry_msgs::Point pt_ros;
        pt_utils::copy3(pt, pt_ros);
        marker.points.push_back(pt_ros);
      } // end loop col
    } // end loop row
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
    ros::spinOnce();

    // show resulting images
    cv::imshow("frame_raw", frame_raw);
    cv::imshow("frame_remapped", frame_remapped);
    char c = cv::waitKey(15);
    // printf("c:%i\n", (int) c);
    if ((int) c == 27)
      break;
    else if (c == 82)
      static_z += .1;
    else if (c == 84)
      static_z -= .1;
  }
  capture.release();
} // end rectify_image_ros();
} // end namespace  CameraUtils

#endif /* CAMERAUTILS_H */


