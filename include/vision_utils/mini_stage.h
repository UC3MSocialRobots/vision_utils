/*!
  \file        mini_stage.h
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

A tiny world visualizer, a bit like Stage, based on OpenCV.
It is focused on modularity and ease of use:
the representation is a OpenCV cv::Mat3b on which the user can easily
draw.

Conversion between real world coordinates and image coordinates
are done thanks to pixel2world() and world2pixel().

 */

#ifndef MINI_STAGE_H
#define MINI_STAGE_H

#include <opencv2/highgui/highgui.hpp>
#include "vision_utils/utils/geometry_utils.h"
#include "vision_utils/drawing_utils.h"

class MiniStage {
public:
  MiniStage(int w = 800, int h = 600) : _origin(0, 0) {
    set_scale(1 / 100.f);
    set_dims(w, h);
    clear();
    _mouse_is_lbutton_dragged = false;
    _mouse_is_mbutton_dragged = false;
    _mouse_is_rbutton_dragged = false;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! set the dimensions of the output image - can be called anytime
  void set_dims(int w = 800, int h = 600) {
    _w = w;
    _whalf = _w / 2;
    _h = h;
    _hhalf = _h / 2;
    _viz.create(_h, _w);
    set_heading(0, false); // no smoothing
  }

  //! \return width, in pixels
  inline int cols() const { return _w; }
  //! \return height, in pixels
  inline int rows() const { return _h; }

  //////////////////////////////////////////////////////////////////////////////

  void clear() {
    _viz.setTo(255);
    // update heading
    _real_heading = .3 * _goal_heading + .7 * _real_heading;
  }

  //////////////////////////////////////////////////////////////////////////////

  void draw_grid(const float step = 1, const int grey_color = 0) {
    // find min and max of x, y visibles in the current window
    cv::Point2f corner_pt = pixel2world(0, 0);
    float min_x = corner_pt.x, min_y = corner_pt.y, max_x = min_x, max_y = min_y;
    corner_pt = pixel2world(_w, 0);
    min_x = std::min(min_x, corner_pt.x); min_y = std::min(min_y, corner_pt.y);
    max_x = std::max(max_x, corner_pt.x); max_y = std::max(max_y, corner_pt.y);
    corner_pt = pixel2world(0, _h);
    min_x = std::min(min_x, corner_pt.x); min_y = std::min(min_y, corner_pt.y);
    max_x = std::max(max_x, corner_pt.x); max_y = std::max(max_y, corner_pt.y);
    corner_pt = pixel2world(_w, _h);
    min_x = std::min(min_x, corner_pt.x); min_y = std::min(min_y, corner_pt.y);
    max_x = std::max(max_x, corner_pt.x); max_y = std::max(max_y, corner_pt.y);
    // ROS_WARN("min_x:%g, max_x:%g, min_y:%g, max_x:%g", min_x, max_x, min_y, max_y);

    // draw lines
    cv::Scalar color = CV_RGB(grey_color, grey_color, grey_color);
    for (float x = step * (int) (min_x / step);
         x <= step * (int) (max_x / step);
         x+=step)
      cv::line(_viz, world2pixel(x, min_y), world2pixel(x, max_y), color);
    for (float y = step * (int) (min_y / step);
         y <= step * (int) (max_y / step);
         y+=step)
      cv::line(_viz, world2pixel(min_x, y), world2pixel(max_x, y), color);
  } // end draw_grid();

  //////////////////////////////////////////////////////////////////////////////

  void draw_axes(const int thickness = 2) {
    image_utils::draw_arrow(_viz, world2pixel(0, 0), world2pixel(1, 0),
                            CV_RGB(255, 0, 0), thickness);
    image_utils::draw_arrow(_viz, world2pixel(0, 0), world2pixel(0, 1),
                            CV_RGB(0, 255, 0), thickness);
    cv::circle(_viz, world2pixel(0, 0), 4, CV_RGB(0, 0, 0), 2);
  } // end draw_grid();

  //////////////////////////////////////////////////////////////////////////////

  inline const cv::Mat3b & get_viz() const { return _viz; }
  inline       cv::Mat3b & get_viz()       { return _viz; }

  //////////////////////////////////////////////////////////////////////////////

  inline cv::Point2f pixel2world(const int & pixel_x, const int & pixel_y) const {
    // convert to polar
    float rho = hypot(pixel_y - _hhalf, pixel_x - _whalf),
        theta = -atan2(pixel_y - _hhalf, pixel_x - _whalf);
    // reconvert to cartesian with scale and heading
    return cv::Point2f(_origin.x + rho * _PIXEL2METER * cos(theta + _real_heading),
                       _origin.y + rho * _PIXEL2METER * sin(theta + _real_heading));
  }

  template<class Pt2i>
  inline cv::Point2f pixel2world(const Pt2i & pixel) const {
    return pixel2world(pixel.x, pixel.y);
  }

  //! batch convert a list of points
  template<class Pt2i>
  inline void pixel2world(const std::vector<Pt2i> & pixel,
                          std::vector<cv::Point2f> & world) const {
    world.clear();
    world.reserve(pixel.size());
    for (unsigned int pixel_idx = 0; pixel_idx < pixel.size(); ++pixel_idx)
      world.push_back(pixel2world(pixel[pixel_idx]));
  }

  //! convert a pixel distance, to a world distance in meters
  inline float pixel2world(const float & pixel_distance) const {
    return pixel_distance * _PIXEL2METER;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline cv::Point2i world2pixel(const float & world_x, const float & world_y) const {
    // convert to polar
    float rho = hypot(world_y - _origin.y, world_x - _origin.x),
        theta = atan2(world_y - _origin.y, world_x - _origin.x);
    // reconvert to cartesian with scale and heading
    return cv::Point(_whalf + rho * _METER2PIXEL * cos(theta - _real_heading),
                     _hhalf - rho * _METER2PIXEL * sin(theta - _real_heading));
  }

  template<class Pt2f>
  inline cv::Point2i world2pixel(const Pt2f & world) const {
    return world2pixel(world.x, world.y);
  }

  //! batch convert a list of points
  template<class Pt2f>
  inline void world2pixel(const std::vector<Pt2f> & world,
                          std::vector<cv::Point> & pixel) const {
    pixel.clear();
    pixel.reserve(world.size());
    for (unsigned int world_idx = 0; world_idx < world.size(); ++world_idx)
      pixel.push_back(world2pixel(world[world_idx]));
  }

  //! convert a real distance, in meters, to a distance in pixels
  inline float world2pixel(const float & world_distance) const {
    return world_distance * _METER2PIXEL;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! increase it (decrease the denominator) to zoom out;
   *  decrease it (increase the denominator) to zoom in */
  inline void   set_scale(const float & new_PIXEL2METER) {
    _PIXEL2METER = new_PIXEL2METER;
    _METER2PIXEL = 1. / _PIXEL2METER;
  }
  inline float get_scale() const { return _PIXEL2METER; }

  //////////////////////////////////////////////////////////////////////////////

  inline void   set_heading(const float & new_heading, bool smooth = true) {
    _goal_heading = new_heading /*+ M_PI_2*/;
    if (!smooth)
      _real_heading = _goal_heading;
  }
  inline float get_real_heading() const { return _real_heading /*+ M_PI_2*/; }

  //////////////////////////////////////////////////////////////////////////////

  inline void        set_origin(const cv::Point2f & new_origin) { _origin = new_origin; }
  inline cv::Point2f get_origin() const { return _origin; }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_visible_window(const double xmin, const double xmax,
                                 const double ymin, const double ymax) {
    //printf("set_visible_window(x:%g->%g, y:%g->%g)\n", xmin, xmax, ymin, ymax);
    set_origin(cv::Point2f(.5*(xmin+xmax), .5*(ymin+ymax)));
    // _scale:  1 pixel = scale meters (meter/pixel)
    set_scale(std::max(1. * (xmax - xmin) / _w, 1. * (ymax - ymin) / _h));
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_mouse_move_callback(const std::string& window_name) {
    cv::namedWindow(window_name);
    cv::setMouseCallback(window_name, mouse_move_callback_static, this);
  }

  //////////////////////////////////////////////////////////////////////////////

  static inline void mouse_move_callback_static
  (int event, int x, int y, int flags, void* param) {
    ((MiniStage*) param)->mouse_move_callback(event, x, y);
  } // end mouse_move_callback_static();

  //////////////////////////////////////////////////////////////////////////////

  //! return true if need to redraw
  inline bool mouse_move_callback(int event, int x, int y) {
    // maggiePrint("mouse_move_callback(event:%i, x:%i, y:%i)", event, x, y);
    // determine if we are dragging or not
    if (event == CV_EVENT_LBUTTONDOWN) {
      _mouse_is_lbutton_dragged = true;
      _mouse_drag_begin_x = x;
      _mouse_drag_begin_y = y;
    }
    else if (event == CV_EVENT_MBUTTONDOWN) {
      _mouse_is_mbutton_dragged = true;
      _mouse_drag_begin_x = x;
      _mouse_drag_begin_y = y;
    }
    else if (event == CV_EVENT_RBUTTONDOWN) {
      _mouse_is_rbutton_dragged = true;
      _mouse_drag_begin_x = x;
      _mouse_drag_begin_y = y;
    }
    else if (event == CV_EVENT_LBUTTONUP)
      _mouse_is_lbutton_dragged = false;
    else if (event == CV_EVENT_MBUTTONUP)
      _mouse_is_mbutton_dragged = false;
    else if (event == CV_EVENT_RBUTTONUP)
      _mouse_is_rbutton_dragged = false;

    // use left button for moving
    if (event == CV_EVENT_LBUTTONDOWN)
      _mouse_original_origin = get_origin();
    if (_mouse_is_lbutton_dragged) {
      set_origin(_mouse_original_origin
                 + _PIXEL2METER * cv::Point2f(_mouse_drag_begin_x - x,
                                        -(_mouse_drag_begin_y - y))
                 );
    }

    // use right button for zoom/unzoom
    if (event == CV_EVENT_RBUTTONDOWN)
      _mouse_original_scale = _PIXEL2METER;
    if (_mouse_is_rbutton_dragged) {
      double new_scale = (1 + (y - _mouse_drag_begin_y) / 100.f) * _mouse_original_scale;
      // maggiePrint("new_scale:%g", new_scale);
      if (new_scale >= 0)
        set_scale(new_scale);
    }

    // only need to redraw if left or right button is dragged
    return (_mouse_is_lbutton_dragged
            || _mouse_is_rbutton_dragged
            //    || event == CV_EVENT_LBUTTONUP
            //    || event == CV_EVENT_RBUTTONUP
            );
  } // end mouse_move_callback();

protected:
  //! size of the window
  int _w, _h, _whalf, _hhalf;

  cv::Mat3b _viz;

  //! coordinates of the image center
  cv::Point2f _origin;
  //! 1 pixel = scale meters (meter/pixel)
  float _PIXEL2METER, _METER2PIXEL;
  //! heading of the world
  float _real_heading, _goal_heading;
  //!
  int _mouse_drag_begin_x, _mouse_drag_begin_y;
  bool _mouse_is_lbutton_dragged, _mouse_is_mbutton_dragged, _mouse_is_rbutton_dragged;
  float _mouse_original_scale;
  cv::Point2f _mouse_original_origin;
}; // end class MiniStage

#endif // MINI_STAGE_H
