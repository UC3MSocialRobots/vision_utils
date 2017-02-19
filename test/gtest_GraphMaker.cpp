/*!
 * \file test_GraphMaker.cpp
 *
 * Some tests for the graph maker
 *
 * \date Dec 18, 2010
 * \author Arnaud Ramey
 */
bool display = false;
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "vision_utils/GraphMaker.h"
#include "highgui.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gtest_GraphMaker");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);

  // rows, cols
  cv::Mat image(500, 800, CV_8UC3);

  vision_utils::GraphMaker<double, double> gm;
  gm.set_image(image);
  gm.set_window(5, 15, 1, -8, 8, 1);

  double global_counter = 0;
  for (int i = 0; i < 1000; ++i) {
    ++global_counter;
    // build a custom curve
    vision_utils::GraphMaker<double, double>::Curve curve_cos, curve_sin;
    for (double t = 0; t < 15; t += .1f) {
      double y_cos = 5 * cos(t + global_counter / 10.f);
      double y_sin = 3 * sin(t + global_counter / 10.f);
      curve_cos.push_back(std::pair<double, double> (t, y_cos));
      curve_sin.push_back(std::pair<double, double> (t, y_sin));
    }

    // draw
    gm.remove_all_curves();
    gm.add_curve(curve_cos, CV_RGB(255, 0, 0));
    gm.add_curve(curve_sin, CV_RGB(0, 255, 0));
    gm.draw();
    gm.draw_horizontal(5, CV_RGB(255, 255, 0), 2);
    gm.draw_vertical(11.5, CV_RGB(0, 255, 255), 2);
    if (display) {
      cv::imshow("image", image);
      if ((char) cv::waitKey(5) == 27)
        break;
    } // end if display
  }

  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
