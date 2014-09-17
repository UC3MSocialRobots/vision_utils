/*!
  \file        mini_stage_plugins.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/3/10

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

Some plugins to draw more stuff with a MiniStage.

 */

#ifndef MINI_STAGE_PLUGINS_H
#define MINI_STAGE_PLUGINS_H

#include "mini_stage.h"
#include "ros_utils/costmap_utils.h"
// kinect
#include "kinect_utils/kinect_openni_utils.h"
// ellipse
#include "image_utils/ellipse_utils.h"

namespace mini_stage_plugins {

template<class T1, class T2>
void plot_xy_lines(MiniStage & ms,
                   const std::vector<T1> & xvec,
                   const std::vector<T2> & yvec,
                   const cv::Scalar& color, int thickness=1, int lineType=8) {
  unsigned int npts = std::min(xvec.size(), yvec.size());
  if (npts < 1)
    return;
  cv::Point curr, prev = ms.world2pixel(xvec[0], yvec[0]);
  for (unsigned int pt_idx = 1; pt_idx < npts; ++pt_idx) {
    curr = ms.world2pixel(xvec[pt_idx], yvec[pt_idx]);
    cv::line(ms.get_viz(), prev, curr, color, thickness, lineType);
    prev = curr;
  } // end loop pt_idx
}

template<class Pt2>
void plot_xy_lines(MiniStage & ms,
                   const std::vector<Pt2> & vec,
                   const cv::Scalar& color, int thickness=1, int lineType=8) {
  unsigned int npts = vec.size();
  if (npts < 1)
    return;
  cv::Point curr, prev = ms.world2pixel(vec[0].x, vec[0].y);
  for (unsigned int pt_idx = 1; pt_idx < npts; ++pt_idx) {
    curr = ms.world2pixel(vec[pt_idx]);
    cv::line(ms.get_viz(), prev, curr, color, thickness, lineType);
    prev = curr;
  } // end loop pt_idx
}

////////////////////////////////////////////////////////////////////////////////

template<class T1, class T2>
void plot_xy_pts(MiniStage & ms,
                 const std::vector<T1> & xvec,
                 const std::vector<T2> & yvec,
                 const cv::Scalar& color, int thickness=1, int lineType=8,
                 const double xshift = 0, const double yshift = 0) {
  unsigned int npts = std::min(xvec.size(), yvec.size());
  for (unsigned int pt_idx = 1; pt_idx < npts; ++pt_idx)
    cv::circle(ms.get_viz(), ms.world2pixel(xvec[pt_idx]+xshift, yvec[pt_idx]+yshift),
               thickness, color, -1, lineType);
}

template<class Pt2>
void plot_xy_pts(MiniStage & ms,
                 const std::vector<Pt2> & vec,
                 const cv::Scalar& color, int thickness=1, int lineType=8,
                 const double xshift = 0, const double yshift = 0) {
  unsigned int npts = vec.size();
  for (unsigned int pt_idx = 1; pt_idx < npts; ++pt_idx)
    cv::circle(ms.get_viz(), ms.world2pixel(vec[pt_idx].x+xshift, vec[pt_idx].y+yshift),
               thickness, color, -1, lineType);
}

////////////////////////////////////////////////////////////////////////////////

//! draw costmap
void draw_costmap(MiniStage & ms,
                  const nav_msgs::GridCells & map,
                  std::vector<cv::Point3f> & map_to_corners,
                  const cv::Scalar inner_color = CV_RGB(200, 200, 255),
                  const cv::Scalar edge_color = CV_RGB(0, 0, 255),
                  const int edge_width = 3) {
  // get corners of the cell
  costmap_utils::costmap_to_polygon_list(map, map_to_corners);
  // convert world 2D corners into MiniStage 2D corners
  std::vector< std::vector<cv::Point2i> > polygons;
  polygons.reserve(map.cells.size());
  std::vector<cv::Point2i> one_cell_polygon;
  one_cell_polygon.resize(4);
  for (unsigned int cell_idx = 0; cell_idx < map.cells.size(); ++cell_idx) {
    for (unsigned int corner_idx = 0; corner_idx < 4; ++corner_idx)
      one_cell_polygon[corner_idx] = ms.world2pixel(map_to_corners[4 * cell_idx + corner_idx]);
    // image_utils::drawPolygon(ms.get_viz(), polygon, true, CV_RGB(0, 0, 255), 3);
    polygons.push_back(one_cell_polygon);
  } // end for (cell_idx)
  // draw polygons
  cv::fillPoly(ms.get_viz(), polygons, inner_color);
  cv::polylines(ms.get_viz(), polygons, true, edge_color, edge_width);
} // end draw_costmap();

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
void reproject_image(MiniStage & ms,
                     const cv::Mat & bgr_img,
                     const cv::Mat & depth_img,
                     const image_geometry::PinholeCameraModel & depth_cam_model,
                     std::vector<Pt3> & depth_reprojected,
                     std::vector<cv::Scalar> & colors,
                     const int data_step = 1,
                     const cv::Mat1b & mask_depth = cv::Mat())
{
  kinect_openni_utils::pixel2world_rgb_color255(bgr_img, depth_img, depth_cam_model,
                                                depth_reprojected, colors,
                                                data_step, mask_depth);
  unsigned int npts = colors.size();
  if (depth_reprojected.size() != npts) {
    maggiePrint("reproject_image(): depth_reprojected.size()=%i != colors.size()=%i!",
                depth_reprojected.size(), colors.size());
    return;
  }
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    cv::circle(ms.get_viz(), ms.world2pixel(depth_reprojected[pt_idx]), 2,
               colors[pt_idx], -1);
  } // end loop pt_idx
} // end reproject_image();

////////////////////////////////////////////////////////////////////////////////

//! draw ellipse
void draw_ellipse(MiniStage & ms,
                  const ellipse_utils::Ellipse & e,
                  const cv::Scalar& color, int thickness=1, int lineType=8) {
  cv::Point2f long1, long2, short1, short2;
  ellipse_utils::ellipse_axes(e, long1, long2, short1, short2);
  cv::Point2f px_center = ms.world2pixel(e.center),
      px_long1 = ms.world2pixel(long1), px_short1 = ms.world2pixel(short1);
  cv::ellipse(ms.get_viz(),
              ellipse_utils::three_pts2ellipse(px_center, px_long1, px_short1),
              color, thickness, lineType);
}

} // end namespace mini_stage_plugins

#endif // MINI_STAGE_PLUGINS_H
