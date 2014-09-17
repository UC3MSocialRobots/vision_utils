/*!
  \file        test_mini_stage.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/26

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

\todo Description of the file

 */

#include <src/debug/debug.h>
#include <src/time/timer.h>
#include <vision_utils/img_path.h>
#include "mini_stage.h"
#include "mini_stage_plugins.h"

std::string _window_name = "test_mini_stage";

void test_reproject_points() {
  MiniStage ms;
  for (unsigned int i = 0; i < 5; ++i) {
    // cv::Point2f test_pt(0, 0);
    cv::Point2f test_pt(drand48() * 5, drand48() * 5);
    cv::Point2f back_to_world = ms.pixel2world(ms.world2pixel(test_pt));
    double dist = geometry_utils::distance_points(test_pt, back_to_world);
    maggieDebug2("p:%s, to pixel:%s, back to world:%s : OK:%i",
                 geometry_utils::printP2(test_pt).c_str(),
                 geometry_utils::printP2(ms.world2pixel(test_pt)).c_str(),
                 geometry_utils::printP2(back_to_world).c_str(),
                 (fabs(dist) < 1E-1));
  } // end loop i
} // end test_reproject_points()
////////////////////////////////////////////////////////////////////////////////

void test_mouse_move_callback() {
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

void test_custom_mouse_callback() {
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
} // end test_custom_mouse_callback()

////////////////////////////////////////////////////////////////////////////////

void test_moving_origin_and_heading() {
  MiniStage ms;
  ms.set_mouse_move_callback(_window_name);
  cv::namedWindow(_window_name);
  Timer t;
  while(true) {
    ms.set_origin(cv::Point2f(cos(t.getTimeSeconds()), sin(t.getTimeSeconds())));
    ms.clear();
    ms.draw_grid(.5f, 150);
    ms.draw_axes();
    ms.set_heading(ms.get_real_heading() + .03);
    cv::circle(ms.get_viz(), ms.world2pixel(ms.get_origin()), 4, CV_RGB(0, 0, 255), 2);
    cv::imshow(_window_name, ms.get_viz());
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  }
} // end test_moving_origin_and_heading();

////////////////////////////////////////////////////////////////////////////////

void test_brownian_motion() {
  MiniStage ms;
  ms.set_mouse_move_callback(_window_name);
  cv::Point2f object_center;
  std::vector<cv::Point2f> object_center_hist;
  std::vector<cv::Point> object_center_hist_pixels;
  float heading = 0;
  double speed = .5; // m.s-1
  Timer t;
  bool track_heading = false;
  while(true) {
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
    cv::imshow(_window_name, ms.get_viz());
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
    else if (c == ' ')
      track_heading = !track_heading;
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

void test_costmap() {
  test_costmap_map.cell_width = 1;
  test_costmap_map.cell_height = 1;
  MiniStage ms;
  cv::namedWindow(_window_name);
  cv::setMouseCallback(_window_name, test_costmap_mouse_cb, &ms);
  test_costmap_mouse_cb(CV_EVENT_MBUTTONDOWN, 0, 0, 0, &ms); // init drawing
  while (true) {
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  } // end while (true)
} // end test_costmap()

////////////////////////////////////////////////////////////////////////////////

void test_reproject_image(const std::string & kinect_serial_number) {
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  kinect_openni_utils::read_camera_model_files
      (kinect_serial_number, depth_camera_model, rgb_camera_model);

  // read depth and rgb files
  cv::Mat3b rgb_img = cv::imread(std::string(IMG_DIR) + "sample_rgb.jpg",
                                 CV_LOAD_IMAGE_COLOR);
  cv::Mat1f depth_img = cv::imread(std::string(IMG_DIR) + "sample_depth.pgm",
                                   CV_LOAD_IMAGE_UNCHANGED);

  MiniStage ms;
  std::vector<cv::Point3f> depth_reprojected;
  std::vector<cv::Scalar> colors;
  cv::namedWindow(_window_name);
  ms.set_mouse_move_callback(_window_name);
  while (true) {
    ms.clear();
    ms.draw_grid(.5f, 150);
    ms.draw_axes();
    mini_stage_plugins::reproject_image(ms, rgb_img, depth_img, depth_camera_model,
                                        depth_reprojected, colors, 3);
    cv::imshow(_window_name, ms.get_viz());
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  } // end while (true)
} // end test_reproject_image()

////////////////////////////////////////////////////////////////////////////////

int main() {
  maggiePrint("%s", _window_name.c_str());
  srand(time(NULL));
  int idx = 1;
  printf("%i: test_reproject_points();\n", idx++);
  printf("%i: test_mouse_move_callback()\n", idx++);
  printf("%i: test_custom_mouse_callback()\n", idx++);
  printf("%i: test_moving_origin_and_heading()\n", idx++);
  printf("%i: test_brownian_motion()\n", idx++);
  printf("%i: test_costmap()\n", idx++);
  printf("%i: test_reproject_image(KINECT_SERIAL_ARNAUD())\n", idx++);
  printf("\n");

  printf("choice?\n");
  int choice = 1;
  std::cin >> choice;

  idx = 1;
  if (choice == idx++)
    test_reproject_points();
  else if (choice == idx++)
    test_mouse_move_callback();
  else if (choice == idx++)
    test_custom_mouse_callback();
  else if (choice == idx++)
    test_moving_origin_and_heading();
  else if (choice == idx++)
    test_brownian_motion();
  else if (choice == idx++)
    test_costmap();
  else if (choice == idx++)
    test_reproject_image(KINECT_SERIAL_ARNAUD());
  return 0;
}
