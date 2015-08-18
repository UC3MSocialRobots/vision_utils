/*!
  \file        cloud_viewer.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/07/20

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

Some useful functions for visualizing point clouds

 */

#ifndef CLOUD_VIEWER_H
#define CLOUD_VIEWER_H

#include "vision_utils/cloud_viewer.h"
#include <opencv2/core/core.hpp>

namespace cloud_viewer {

//typedef pcl::visualization::CloudViewer Viewer;
typedef pcl::visualization::PCLVisualizer Viewer;

inline void set_default_params(Viewer & viewer) {
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();
}

////////////////////////////////////////////////////////////////////////////////

/*! Look at a given point from a given point. From
 * http://www.pcl-users.org/Need-more-clarification-on-PCL-Visualizer-s-setCameraPosition-tp3699057p3712055.html
 */
inline void look_at(Viewer & v,
                    double cam_x, double cam_y, double cam_z,
                    double lookat_x, double lookat_y, double lookat_z,
                    double up_x = 0, double up_y = 1, double up_z = 0) {
  /*v.camera_.pos[0] = cam_x;
  v.camera_.pos[1] = cam_y;
  v.camera_.pos[2] = cam_z;
  v.camera_.focal[0] = lookat_x;
  v.camera_.focal[1] = lookat_y;
  v.camera_.focal[2] = lookat_z;
  v.camera_.view[0] = up_x;
  v.camera_.view[1] = up_y;
  v.camera_.view[2] = up_z;
  */
  v.setCameraPosition(cam_x, cam_y, cam_z,  lookat_x, lookat_y, lookat_z,  up_x, up_y, up_z);
  v.updateCamera();
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
inline bool is_point_nonvalid(const Pt3 & pt) {
  return (pt.x != pt.x || isnan(pt.x) ||
                  pt.y != pt.y || isnan(pt.y) ||
                          pt.z != pt.z || isnan(pt.z) ||
                                  fabsf(pt.x) > 10.0 ||
                                  fabsf(pt.y) > 10.0 ||
                                  fabsf(pt.z) > 10.0);
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2pcl(const std::vector<Pt3>& pointcloud) {
  typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  //... populate cloud
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  unsigned int npts = pointcloud.size();

  for (unsigned int i=0; i < npts; i++) {
    // check for erroneous coordinates (NaN, Inf, etc.)
    if (is_point_nonvalid(pointcloud[i]))
      continue;
    cloud->push_back(pcl::PointXYZ(pointcloud[i].x, pointcloud[i].y, pointcloud[i].z));
  } // end loop i
  cloud->width = (uint32_t) cloud->points.size(); // number of points
  cloud->height = 1; // a list of points, one row of data
  return cloud;
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud2pcl(const std::vector<Pt3>& pointcloud,
                                                     const std::vector<cv::Vec3b>& pointcloud_RGB) {
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  //... populate cloud
  cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  unsigned int npts = pointcloud.size(), npts_rgb = pointcloud_RGB.size();

  for (unsigned int i=0; i < npts; i++) {
    // check for erroneous coordinates (NaN, Inf, etc.)
    if (is_point_nonvalid(pointcloud[i]))
      continue;
    pcl::PointXYZRGB pclp;
    // 3D coordinates
    pclp.x = pointcloud[i].x;
    pclp.y = pointcloud[i].y;
    pclp.z = pointcloud[i].z;

    // get the RGB color value for the point
    cv::Vec3b rgbv(255,255,255);
    if (i < npts_rgb)
      rgbv = pointcloud_RGB[i];
    // RGB color, needs to be represented as an integer
    uint32_t rgb = ((uint32_t)rgbv[2] << 16 | (uint32_t)rgbv[1] << 8 |
                                                                   (uint32_t)rgbv[0]);
    pclp.rgb = *reinterpret_cast<float*>(&rgb);

    cloud->push_back(pclp);
  } // end loop i
  cloud->width = (uint32_t) cloud->points.size(); // number of points
  cloud->height = 1; // a list of points, one row of data
  return cloud;
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
void view_cloud(const std::vector<Pt3>& pointcloud,
                Viewer & viewer) {
  viewer.addPointCloud<pcl::PointXYZ>(cloud2pcl<Pt3>(pointcloud));
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
void view_rgb_cloud(const std::vector<Pt3>& pointcloud,
                    const std::vector<cv::Vec3b>& pointcloud_RGB,
                    Viewer & viewer) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_could =
      rgb_cloud2pcl<Pt3>(pointcloud, pointcloud_RGB);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pcl_could);
  viewer.addPointCloud<pcl::PointXYZRGB>(pcl_could);
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
void view_cloud(const std::vector<Pt3>& pointcloud,
                bool blocking = true) {
  Viewer viewer("SimpleCloudViewer");
  set_default_params(viewer);
  view_cloud<Pt3>(pointcloud, viewer);
  if (!blocking)
    return;
  viewer.spin();
} // end view_cloud()

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
void view_rgb_cloud(const std::vector<Pt3>& pointcloud,
                    const std::vector<cv::Vec3b>& pointcloud_RGB,
                    bool blocking = true) {
  Viewer viewer("SimpleCloudViewer");
  set_default_params(viewer);
  view_rgb_cloud<Pt3>(pointcloud, pointcloud_RGB, viewer);
  if (!blocking)
    return;
  viewer.spin();
} // end view_cloud()

////////////////////////////////////////////////////////////////////////////////

} // end namespace cloud_viewer

#endif // CLOUD_VIEWER_H
