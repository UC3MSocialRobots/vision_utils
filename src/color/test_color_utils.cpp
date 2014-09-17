#include "color_utils.h"
#include "debug/error.h"
#include <opencv2/highgui/highgui.hpp>

////////////////////////////////////////////////////////////////////////////////

int cols = 800, rows = 50;
cv::Mat3b test_hue_to_string_bg(rows, cols), test_hue_to_string_bg_name;

void test_hue_to_string_mouse_cb(int event, int x, int y, int flags, void* param) {
   test_hue_to_string_bg.copyTo(test_hue_to_string_bg_name);
   float h = 180.f * x / cols;
   cv::putText(test_hue_to_string_bg_name, color_utils::hue_to_string(h),
               cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));
}

void test_hue_to_string() {
  // init test_hue_to_string_bg
  for (int col = 0; col < cols; ++col) {
    float h = 180.f * col / cols;
    cv::line(test_hue_to_string_bg, cv::Point(col, 0), cv::Point(col, rows),
             color_utils::hue2rgb<cv::Scalar>(h));
  } // end loop col

  // set callbacks
  std::string window_name = "hue_to_string";
  cv::namedWindow(window_name);
  cv::setMouseCallback(window_name, test_hue_to_string_mouse_cb);

  // spin
  test_hue_to_string_mouse_cb(CV_EVENT_MOUSEMOVE, cols / 2, rows / 2, 0, NULL);
  while (true) {
    cv::imshow(window_name, test_hue_to_string_bg_name);
    char c = cv::waitKey(25);
    if ((int) c == 27)
      break;
  } // end while true
}

////////////////////////////////////////////////////////////////////////////////

void test_hue_scales() {
  cv::Mat3b img(600, 800);
  for (int row = 0; row < img.rows; ++row) {
    cv::Vec3b* data = img.ptr<cv::Vec3b>(row);
    for (int col = 0; col < img.cols; ++col) {
      float r, g, b;
      float h = 360.f * col / img.cols;
      float s = 1.f * row / img.rows;
      color_utils::HSVtoRGB(h, s, 1.f, r, g, b);
      data[col][0] = 255 * b;
      data[col][1] = 255 * g;
      data[col][2] = 255 * r;
    } // end loop col
  } // end loop row

  cv::Mat3b img2(50, 800);
  for (int col = 0; col < img.cols; ++col) {
    float h = 180.f * col / img.cols;
    cv::line(img2, cv::Point(col, 0), cv::Point(col, img2.rows),
             color_utils::hue2rgb<cv::Scalar>(h));
  } // end loop col

  cv::imshow("img", img);
  cv::imshow("img2", img2);
  cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

int main() {
  maggieDebug2("main()");
  int idx = 1;
  printf("%i: test_hue_scales()\n", idx++);
  printf("%i: test_hue_to_string()\n", idx++);
  printf("\n");

  printf("choice?\n");
  int choice = 1;
  std::cin >> choice;

  idx = 1;
  if (choice == idx++)
    test_hue_scales();
  else if (choice == idx++)
    test_hue_to_string();

  return 0;
}

