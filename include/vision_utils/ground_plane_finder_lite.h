/*!
  \file        ground_plane_finder.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/11

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

#ifndef GROUND_PLANE_FINDER_H
#define GROUND_PLANE_FINDER_H

// PCL
#include <range_image/range_image.h>
#include <ModelCoefficients.h>
#include <io/pcd_io.h>
#include <point_types.h>
#include <sample_consensus/method_types.h>
#include <sample_consensus/model_types.h>
#include <segmentation/sac_segmentation.h>
// http://blog.martinperis.com/2012/01/3d-reconstruction-with-opencv-and-point.html
// http://docs.pointclouds.org/trunk/classpcl_1_1_range_image.html
// http://pointclouds.org/documentation/tutorials/range_image_creation.php

namespace vision_utils {


class GroundPlaneFinder {
public:
  typedef cv::Point3f Pt3f;
  static const double DEFAULT_DISTANCE_THRESHOLD_M = 0.05; // 5 cm
  static const double DEFAULT_LOWER_RATIO_TO_USE = .4; // 40%
  static const int DEFAULT_DATA_SKIP = 2;

  // proj matrix [a11 0  a13 0]
  //             [0  a22 a23 0]
  //             [0   0   1  0]
  static const double a11 = 520.482604980469, a13 = 321.952954956043,
  a22 = 522.613891601562, a23 = 244.539890703203;

  //////////////////////////////////////////////////////////////////////////////

  GroundPlaneFinder() : _plane_found(false) {
  }

  //////////////////////////////////////////////////////////////////////////////

  inline Pt3f pixel2world(int col, int row, float depth) const {
    return Pt3f ((1. * col - a13) * depth / a11,
                 (1. * row - a23) * depth / a22,
                 depth);
  }

  // cf http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php
  bool compute_plane(const cv::Mat1f & depth,
                     double distance_threshold_m = DEFAULT_DISTANCE_THRESHOLD_M,
                     double lower_ratio_to_use = DEFAULT_LOWER_RATIO_TO_USE,
                     int data_skip = DEFAULT_DATA_SKIP) {
    // use only second half of the image
    if (depth.rows == 0) {
      printf("GroundPlaneFinder:depth image too small!\n");
      _plane_found = false;
      return false;
    }

    unsigned int min_row = (1 - lower_ratio_to_use) * depth.rows,
        max_row = depth.rows, min_col = 0, max_col = depth.cols;
    // http://answers.ros.org/question/10350/how-to-construct-the-point-cloud-data-from-the-depth-data/
    // TODO convert subimage depth(roi) to point cloud (in _pts)
    for (int row = min_row; row < max_row; row+=data_skip) {
      const float* depth_ptr = depth.ptr<float>(row);
      for (int col = min_col; col < max_col; col+=data_skip) {
        if (depth_ptr[col] == 0)
          continue;
        _pts.push_back(pixel2world(col, row, depth_ptr[col]));
      } // end loop col
    } // end loop row
    unsigned int npts = _pts.size();
    std::cout << "Point cloud data: " << npts << " points" << std::endl;
    if (npts == 0) {
      printf("GroundPlaneFinder:depth image does not contain valid depth point!\n");
      _plane_found = false;
      return false;
    }

    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width  = npts;
    cloud.height = 1;
    cloud.points.resize(npts);
    for (size_t i = 0; i < npts; ++i) {
      cloud.points[i].x = _pts[i].x;
      cloud.points[i].y = _pts[i].y;
      cloud.points[i].z = _pts[i].z;
    }

    coefficients.values.clear();
    inliers.indices.clear();
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold_m);
    seg.setInputCloud (cloud.makeShared ());
    seg.segment (inliers, coefficients);

    if (inliers.indices.size () == 0) {
      printf("GroundPlaneFinder:Could not estimate a planar model for the given dataset.\n");
      _plane_found = false;
      return false;
    }

    a = coefficients.values[0];
    b = coefficients.values[1];
    c = coefficients.values[2];
    d = coefficients.values[3];

    std::cout << "Model coefficients: " << a << " " << b << " " << c << " " << d << std::endl;
#if 0
    std::cout << "Model inliers: " << inliers.indices.size () << std::endl;
    for (size_t i = 0; i < inliers.indices.size (); ++i)
      std::cout << inliers.indices[i] << "    " << cloud.points[inliers.indices[i]].x << " "
                << cloud.points[inliers.indices[i]].y << " "
                << cloud.points[inliers.indices[i]].z << std::endl;
#endif
    _plane_found = true;
    return true;
  } // end compute_plane()

  //////////////////////////////////////////////////////////////////////////////

  /*! generate a mask image with grounds plane marked in white and the rest in black
   *  \var mark_if_ground if false, mark points that do NOT belong to the plane
   *                      (but that are not NaN)
    */
  bool to_img(const cv::Mat1f & depth,
              cv::Mat1b & img,
              double min_dist = -1, double max_dist = -1,
              double distance_threshold_m = 0.05,
              bool mark_if_ground = true) const {
    img.create(depth.size());
    img.setTo(0);
    if (!_plane_found) {
      printf("GroundPlaneFinder: you need to call compute_plane() before to_img()!\n");
      return false;
    }
    // locates matrix header within a parent matrix
    // cf http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
    cv::Size size;
    cv::Point offset;
    depth.locateROI(size, offset);

    bool use_min_dist = (min_dist > 0);
    bool use_max_dist = (max_dist > 0);
    int cols = depth.cols, rows = depth.rows;
    for (int row = 0; row < rows; ++row) {
      const float* depth_ptr = depth.ptr<float>(row);
      uchar* img_ptr = img.ptr(row);
      for (int col = 0; col < cols; ++col) {
        //if (is_nan_depth(depth_ptr[col]))
        if (depth_ptr[col] == 0)
          continue;
        if (use_min_dist && depth_ptr[col] < min_dist)
          continue;
        if (use_max_dist && depth_ptr[col] > max_dist)
          continue;
        Pt3f curr_cv = pixel2world(col, row, depth_ptr[col]);
        pcl::PointXYZ curr(curr_cv.x, curr_cv.y, curr_cv.z);
        // = pixel2world_depth<pcl::PointXYZ>
        //        (cv::Point2d(col + offset.x, row + offset.y), model, depth_ptr[col]);
        double dist = fabs(a * curr.x + b * curr.y + c * curr.z + d);
        if ((mark_if_ground && dist < distance_threshold_m) // belongs to ground
            || (!mark_if_ground && dist > distance_threshold_m)) // does not belong to ground
          img_ptr[col] = 255;
      } // end loop col
    } // end loop row
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool is_plane_found() const { return _plane_found; }

  //////////////////////////////////////////////////////////////////////////////

private:
  bool _plane_found;
  double a, b, c, d;

  // cached data
  std::vector<Pt3f> _pts;
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
}; // en class GroundPlaneFinder

} // end namespace vision_utils

#endif // GROUND_PLANE_FINDER_H
