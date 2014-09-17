/*!
 * \file CameraUtils.cpp
 *
 * The implementation of camera_utils.h
 *
 * \date 10/11/2010
 * \author Arnaud Ramey
 */

#include "camera_utils/camera_utils.h"

inline void get_video_capture_param(cv::VideoCapture & capture,
                                    const std::string & name,
                                    const int & param_idx) {
  std::cout << std::setw(30) << name << " : "
            << capture.get(param_idx) << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void CameraUtils::getCamera(int camera_nb, cv::VideoCapture & capture) {
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

void CameraUtils::cameraTester_OpenCV2(int camera_nb) {
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

void CameraUtils::photo_camera(cv::VideoCapture & capture,
                                const std::string & filename_radix /*= "pic"*/, int nbPhoto /*= 1*/) {
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

void CameraUtils::cameraSkeleton(cv::VideoCapture & capture) {
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

// #include <image_utils/io.h>

void CameraUtils::cameraRecorder(cv::VideoCapture & capture,  const std::string & outputFilename /*= "../images/video_capture.avi"*/) {
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

void CameraUtils::videoPlayer( const std::string & filename) {
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

void CameraUtils::showThreeLayers(cv::VideoCapture & capture) {
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
