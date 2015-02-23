/*!
  \file        head_finder.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/10/17

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

A useful class for finding the head inside of a user mask.

 */

#ifndef HEAD_FINDER_H
#define HEAD_FINDER_H

#include <image_utils/content_processing.h>
#include <stats/stats_utils.h>
#include <data_filters/clamp.h>
#include <time/timer.h>
#include <kinect_utils/user_image_to_rgb.h>
#include <image_utils/voronoi.h>
#include <image_utils/ellipse_utils.h>


class HeadFinder {
public:
  typedef int Coord;
  typedef cv::Point Pt2;

  //////////////////////////////////////////////////////////////////////////////

  HeadFinder() {
    _thinning_method = IMPL_ZHANG_SUEN_FAST;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool find(const cv::Mat1b & user_mask,
            cv::Point & head_pos) {
    if (user_mask.empty()) {
      printf("HeadFinder:user_mask is empty!\n");
      return false;
    }
    // Timer timer_;
    // erode the img
    cv::Rect bbox = image_utils::boundingBox(user_mask);
    int erode_kernel_size = clamp(bbox.width / 10, 5, 30);
    // erode_kernel_size must be odd
    erode_kernel_size = 1 + 2 * (int) (erode_kernel_size / 2);
    // printf("erode_kernel_size:%i\n", erode_kernel_size);
    cv::Mat erode_kernel = cv::Mat(erode_kernel_size, erode_kernel_size, CV_8U, 255);
    cv::erode(user_mask, user_mask_eroded, erode_kernel);
    if (cv::countNonZero(user_mask_eroded) == 0) {
      if (cv::countNonZero(user_mask) == 0) {
        printf("HeadFinder:user_mask is empty!\n");
        return false;
      }
      printf("HeadFinder:eroded user_mask is black but user_mask is not, "
             "retrying without eroding!\n");
      user_mask.copyTo(user_mask_eroded);
    }
    //    printf("user_mask:%s, user_mask_eroded:%s!\n",
    //           image_utils::img2string(user_mask).c_str(),
    //           image_utils::img2string(user_mask_eroded).c_str());

    // thin img
    if (!_thinner.thin(user_mask_eroded, _thinning_method, true)) {
      printf("HeadFinder:thinning failed!\n");
      return false;
    }
    // timer_.printTime("thinning\n");
    // cv::imshow("_skeleton", _thinner.get_skeleton()); cv::waitKey(0);

    // find end points of skeleton
    if (!image_utils::detect_end_points(_thinner.get_skeleton(), _end_pts)
        || _end_pts.size() == 0) {
      printf("HeadFinder:detect_end_points() failed, skeleton:%s!\n",
             image_utils::img2string(_thinner.get_skeleton()).c_str());
      return false;
    }
    // add offset
    cv::Point offset = _thinner.get_bbox().tl();
    for (unsigned int end_pt_idx = 0; end_pt_idx < _end_pts.size(); ++end_pt_idx)
      _end_pts[end_pt_idx] += offset;
    // timer_.printTime("end points\n");
    if (_end_pts.size() == 1) {
      printf("HeadFinder:only one end_point (%i, %i), returning it!\n",
             _end_pts.front().x, _end_pts.front().y);
      head_pos = _end_pts.front();
      return true;
    }

    // fit ellipse
    image_utils::nonNulPoints(user_mask_eroded, _x, _y);
    if (_x.size() == 0) {
      printf("HeadFinder:user_mask_eroded empty!\n");
      return false;
    }
    if (!gaussian_pdf_ellipse(_x, _y, _ellipse_center, _ellipse_end1, _ellipse_end2, 2)) {
      printf("HeadFinder:could not find the axis of the ellipse!\n");
      return false;
    }
    // timer_.printTime("gaussian_pdf2ellipse\n");
    // find the end axe of the ellipse that must correspond to the neck
    // i.e. find point with lowest y in all end axes
    _fitted_ellipse = ellipse_utils::three_pts2ellipse
                      (_ellipse_center, _ellipse_end1, _ellipse_end2);
    _fitted_ellipse.size.width += erode_kernel_size; // compensate the erode
    _fitted_ellipse.size.height += erode_kernel_size;
    _ellipse_all_axes.resize(4);
    ellipse_utils::ellipse_axes(_fitted_ellipse, _ellipse_all_axes[0], _ellipse_all_axes[1],
        _ellipse_all_axes[2], _ellipse_all_axes[3]);
    _neck_pt.y = INT_MAX;
    for (unsigned int pt_idx = 0; pt_idx < 4; ++pt_idx) {
      if (_neck_pt.y > _ellipse_all_axes[pt_idx].y)
        _neck_pt = _ellipse_all_axes[pt_idx];
    } // end loop pt_idx
    // timer_.printTime("ellipse\n");

    // find closest point between end points and neck pos
    _head_pos = cv::Point(0, 0);
    double dist = std::numeric_limits<double>::max();
    for (unsigned int end_pt_idx = 0; end_pt_idx < _end_pts.size(); ++end_pt_idx) {
      double curr_dist = geometry_utils::distance_points_squared(_end_pts[end_pt_idx], _neck_pt);
      if (curr_dist < dist) {
        _head_pos = _end_pts[end_pt_idx];
        dist = curr_dist;
      }
    } // end loop end_pt_idx
    // timer_.printTime("head pos\n");

    head_pos = _head_pos;
    return true;
  } // end find()

  //////////////////////////////////////////////////////////////////////////////

  void illus(const cv::Mat1b & user_mask) {
    if (user_mask.empty() || user_mask_eroded.empty())
      return;
    // draw user_mask_eroded in color
    //user_image_to_rgb(user_mask_eroded, viz);
    viz.create(user_mask.size());
    viz.setTo(0);
    viz.setTo(cv::Vec3b(0, 0, 255), user_mask_eroded);
    // draw user mask
    viz.setTo(cv::Vec3b(0, 0, 100), user_mask & (user_mask_eroded == 0));
    // draw ellipse
    cv::Scalar ellipse_color = CV_RGB(255, 255, 0);
    cv::ellipse(viz, _fitted_ellipse, ellipse_color, 2);
    //
    cv::line(viz, _ellipse_center, _ellipse_end1, ellipse_color, 2);
    cv::line(viz, _ellipse_center, _ellipse_end2, ellipse_color, 2);
    for (unsigned int axe_idx = 0; axe_idx < _ellipse_all_axes.size(); ++axe_idx) {
      cv::circle(viz, _ellipse_all_axes[axe_idx], 3, ellipse_color, 2);
    } // end loop axe_idx
    // draw neck
    cv::circle(viz, _neck_pt, 3, CV_RGB(255, 255, 255), 2);
    // draw skeleton
    if (!_thinner.get_skeleton().empty())
      viz(_thinner.get_bbox()).setTo(cv::Vec3b(0, 255, 0), _thinner.get_skeleton());
    for (unsigned int end_pt_idx = 0; end_pt_idx < _end_pts.size(); ++end_pt_idx)
      cv::circle(viz, _end_pts[end_pt_idx], 3, CV_RGB(0, 0, 255), 2);

    cv::circle(viz, _head_pos, 5, CV_RGB(0, 255, 0), 2);
    // cv::imwrite("skeleton.png", _skeleton.get_skeleton());
    cv::imshow("viz", viz); cv::imshow("user_mask", user_mask); cv::waitKey(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline std::string get_thinning_method() const { return _thinning_method; }
  inline const cv::Mat1b & get_skeleton() const { return _thinner.get_skeleton(); }
  inline const cv::Rect    get_bbox() const { return _thinner.get_bbox(); }

protected:
  cv::Mat1b user_mask_eroded;

  std::vector<int> _x, _y;
  std::vector<cv::Point> _ellipse_all_axes;
  Pt2 _ellipse_center, _ellipse_end1, _ellipse_end2;

  std::vector<std::vector<Pt2> > contours_struct;
  std::vector<Pt2> _contour;
  cv::RotatedRect _fitted_ellipse;

  Pt2 _neck_pt;

  VoronoiThinner _thinner;
  std::string _thinning_method;
  std::vector<Pt2> _end_pts;
  cv::Point _head_pos;

  // viz stuff
  cv::Mat3b viz;
}; // end class HeadFinder

#endif // HEAD_FINDER_H
