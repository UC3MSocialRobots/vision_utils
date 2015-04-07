/*!
  \file        gtest_mini_stage.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/24

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

Some tests for \b MiniStage class
and \b mini_stage_plugins namespace.

 */
//#define DISPLAY
// Bring in gtest
#include <gtest/gtest.h>
#include <time/timer.h>
#include <vision_utils/img_path.h>
#include "visu_utils/mini_stage_plugins.h"
#include "image_utils/io.h"

std::string _window_name = "test_mini_stage";

TEST(TestSuite, reproject_points) {
  MiniStage ms;
  cv::Point2f origin(0, 0);
  cv::Point2f origin_back_to_world = ms.pixel2world(ms.world2pixel(origin));
  double origin_dist = geometry_utils::distance_points(origin, origin_back_to_world);
  ASSERT_NEAR(fabs(origin_dist), 0, 0.1);

  for (unsigned int i = 0; i < 50; ++i) {
    // cv::Point2f test_pt(0, 0);
    cv::Point2f test_pt(drand48() * 5, drand48() * 5);
    cv::Point2f back_to_world = ms.pixel2world(ms.world2pixel(test_pt));
    double dist = geometry_utils::distance_points(test_pt, back_to_world);
    ASSERT_NEAR(fabs(dist), 0, 0.1)
        << "test_pt:" << geometry_utils::printP2(test_pt).c_str()
        << ", to pixel:" << geometry_utils::printP2(ms.world2pixel(test_pt)).c_str()
        << ", back_to_world:" << geometry_utils::printP2(back_to_world).c_str()
        << ", dist:" << dist;
  } // end loop i}
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, mouse_move_callback) {
#ifdef DISPLAY
  MiniStage ms;
  cv::namedWindow(_window_name);
  ms.set_mouse_move_callback(_window_name);
  while (true) {
    ms.clear();
    ms.draw_grid(.5f, 150);
    ms.draw_axes();
    cv::imshow(_window_name, ms.get_viz());
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  } // end while (true)
#endif // DISPLAY
} // end test_mouse_move_callback()

////////////////////////////////////////////////////////////////////////////////

static inline void test_custom_mouse_callback_mouse_cb(int event, int x, int y, int flags, void* param) {
  MiniStage* ms_ptr = ((MiniStage*) param);
  ms_ptr->mouse_move_callback(event, x, y);
  // convert to 3D
  cv::Point2f pt_3d = ms_ptr->pixel2world(x, y);
  // draw axes and stuff
  ms_ptr->clear();
  ms_ptr->draw_grid(1.f, 150);
  ms_ptr->draw_axes(  );
  // draw point
  cv::circle(ms_ptr->get_viz(), cv::Point(x, y), 2,CV_RGB(255, 255, 0), -1);
  // draw txt
  std::ostringstream txt;
  txt << "(" << x << ", " << y << ") -> " << pt_3d;
  cv::putText(ms_ptr->get_viz(), txt.str(), cv::Point(x + 5, y),
              CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));
  // rotate
  // ms_ptr->set_heading(ms_ptr->get_real_heading() + .01, false);
}

TEST(TestSuite, custom_mouse_callback) {
#ifdef DISPLAY
  MiniStage ms;
  ms.clear();
  ms.draw_grid(1.f, 150);
  ms.draw_axes();
  cv::namedWindow(_window_name);
  cv::setMouseCallback(_window_name, test_custom_mouse_callback_mouse_cb, &ms);
  while (true) {
    cv::imshow(_window_name, ms.get_viz());
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  } // end while (true)
#endif // DISPLAY
} // end test_custom_mouse_callback()

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, moving_origin_and_heading) {
  MiniStage ms;
  ms.set_mouse_move_callback(_window_name);
  cv::namedWindow(_window_name);
  Timer t;
  for (int i = 0; i < 100; ++i) {
    ms.set_origin(cv::Point2f(cos(t.getTimeSeconds()), sin(t.getTimeSeconds())));
    ms.clear();
    ms.draw_grid(.5f, 150);
    ms.draw_axes();
    ms.set_heading(ms.get_real_heading() + .03);
#ifdef DISPLAY
    cv::imshow(_window_name, ms.get_viz());
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
#endif // DISPLAY    cv::circle(ms.get_viz(), ms.world2pixel(ms.get_origin()), 4, CV_RGB(0, 0, 255), 2);
  }
} // end test_moving_origin_and_heading();

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, brownian_motion) {
  MiniStage ms;
  ms.set_mouse_move_callback(_window_name);
  cv::Point2f object_center;
  std::vector<cv::Point2f> object_center_hist;
  std::vector<cv::Point> object_center_hist_pixels;
  float heading = 0;
  double speed = .5; // m.s-1
  Timer t;
  bool track_heading = false;
  for (int i = 0; i < 100; ++i) {
    // change heading from time to time
    if (rand() % 10 == 0)
      heading = drand48() * 2 * M_PI;
    object_center.x += speed * t.getTimeSeconds() * cos(heading);
    object_center.y += speed * t.getTimeSeconds() * sin(heading);
    object_center_hist.push_back(object_center);
    t.reset();

    ms.clear();
    ms.set_heading((track_heading ? heading - M_PI_2 : 0));
    ms.set_origin(object_center);
    ms.draw_grid(.5f, 150);
    ms.draw_axes();
    ms.world2pixel(object_center_hist, object_center_hist_pixels);
    image_utils::drawPolygon(ms.get_viz(), object_center_hist_pixels, false,
                             CV_RGB(255, 0, 0), 2);
    cv::circle(ms.get_viz(), ms.world2pixel(object_center), 4, CV_RGB(0, 0, 0), -1);
    cv::putText(ms.get_viz(), "space to switch heading", cv::Point(10, 20),
                CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));
#ifdef DISPLAY
    cv::imshow(_window_name, ms.get_viz());
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
    else if (c == ' ')
      track_heading = !track_heading;
#endif // DISPLAY
  }
} // end test_brownian_motion();

////////////////////////////////////////////////////////////////////////////////

nav_msgs::GridCells test_costmap_map;
std::vector<cv::Point3f> test_costmap_map_to_corners;

static inline void test_costmap_mouse_cb(int event, int x, int y, int flags, void* param) {
  MiniStage* ms_ptr = ((MiniStage*) param);
  bool need_to_redraw = ms_ptr->mouse_move_callback(event, x, y);
  // convert to 3D
  cv::Point2f pt_world = ms_ptr->pixel2world(x, y);
  bool was_map_changed = false;
  if (event == CV_EVENT_LBUTTONDOWN) {
    costmap_utils::add_point_to_costmap(pt_world, test_costmap_map);
    was_map_changed = true;
  }
  else if (event == CV_EVENT_MBUTTONDOWN) {
    test_costmap_map.cells.clear();
    was_map_changed = true;
  }
  if (was_map_changed || need_to_redraw) { // draw axes and stuff
    ms_ptr->clear();
    mini_stage_plugins::draw_costmap(*ms_ptr, test_costmap_map, test_costmap_map_to_corners);
    ms_ptr->draw_grid(test_costmap_map.cell_width, 150);
    ms_ptr->draw_axes();
    cv::imshow(_window_name, ms_ptr->get_viz());
  }
} // end test_costmap_mouse_cb()

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, costmap) {
  test_costmap_map.cell_width = 1;
  test_costmap_map.cell_height = 1;
  MiniStage ms;
  cv::namedWindow(_window_name);
  cv::setMouseCallback(_window_name, test_costmap_mouse_cb, &ms);
  test_costmap_mouse_cb(CV_EVENT_MBUTTONDOWN, 0, 0, 0, &ms); // init drawing
#ifdef DISPLAY
  while (true) {
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  } // end while (true)
#endif // DISPLAY
} // end test_costmap()

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, reproject_image) {
  std::string kinect_serial_number = KINECT_SERIAL_ARNAUD();
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  kinect_openni_utils::read_camera_model_files
      (kinect_serial_number, depth_camera_model, rgb_camera_model);

  // read depth and rgb files
  cv::Mat rgb, depth;
  image_utils::read_rgb_depth_user_image_from_image_file
      (IMG_DIR "depth/juggling1", &rgb, &depth, NULL);

  MiniStage ms;
  std::vector<cv::Point3f> depth_reprojected;
  std::vector<cv::Scalar> colors;
  cv::namedWindow(_window_name);
  ms.set_mouse_move_callback(_window_name);
  for (int i = 0; i < 100; ++i) {
    ms.clear();
    ms.draw_grid(.5f, 150);
    ms.draw_axes();
    mini_stage_plugins::reproject_image(ms, rgb, depth, depth_camera_model,
                                        depth_reprojected, colors, 3);
#ifdef DISPLAY
    cv::imshow(_window_name, ms.get_viz());
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
#endif // DISPLAY
  } // end while (true)
} // end test_reproject_image()

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
