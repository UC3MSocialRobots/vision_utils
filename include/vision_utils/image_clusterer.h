/*!
  \file        image_clusterer.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/25
  
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

\class ImageClusterer
An extension of \b geometry_utils::Clusterer
designed for clustering reprojected images.
 */

#ifndef IMAGE_CLUSTERER_H
#define IMAGE_CLUSTERER_H

#include "vision_utils/clusterer.h"
#include "kinect/kinect_openni_utils.h"


class ImageClusterer : public geometry_utils::Clusterer {
public:
  ImageClusterer() {}

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Reproject an image and cluster the corresponding points
   * \param bgr_img
   *   The color image
   * \param depth_img
   *   The depth image
   * \param depth_cam_model
   *   The camera model of the depth image
   * \param data_step
   *   The increment in columns and rows for reprojecting the pixels.
   *   Use 1 to reproject all pixels, 2 to reproject 50% of them, etc.
   * \param mask_depth
   *   An optional mask to indicate what points should be kept
   *  (0=> discared pixel, anything else => reproject)
   * \param cluster_tolerance
   *    the spatial cluster tolerance as a measure in the L2 Euclidean space, in meters.
   *    A reasonable value would be 0.1f (10 cm).
   * \return
   *   true if success
   */
  bool cluster(const cv::Mat & bgr_img,
               const cv::Mat & depth_img,
               const image_geometry::PinholeCameraModel & depth_cam_model,
               const unsigned int data_step = 5,
               const cv::Mat1b & mask_depth = cv::Mat(),
               const double & cluster_tolerance = 0.1f) {
    // reset results
    cluster_indices_pcl.clear();

    // first get the reprojected points
    bool success = kinect_openni_utils::pixel2world_rgb_color255
                   (bgr_img, depth_img, depth_cam_model,
                    depth_reprojected, colors,
                    data_step, mask_depth, true);
    if (!success)
      return false;

    // now call the cluster() function with the point cloud
    success = geometry_utils::Clusterer::cluster(depth_reprojected, cluster_tolerance);
    return success;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool get_biggest_cluster_pixels
  (const image_geometry::PinholeCameraModel & depth_cam_model,
   std::vector<cv::Point2i> & cluster_pixels,
   std::vector<cv::Scalar> & cluster_colors)
  {
    cluster_pixels.clear();
    cluster_colors.clear();
    if (cluster_indices_pcl.size() == 0) {
      ROS_WARN("No cluster found by PCL EuclideanClusterExtraction");
      return false;
    }
    if (data_pcl->size() != depth_reprojected.size()) {
      ROS_WARN("%i PCL points != %i reprojected points, "
               "you must have called cluster() with a point cloud and not an image",
               data_pcl->size(), depth_reprojected.size());
      return false;
    }

    // reproject all points to 2D
    const std::vector<int32_t>* biggest_indices_pcl =
        &(cluster_indices_pcl.front().indices);
    cluster_pixels.reserve(biggest_indices_pcl->size());
    cluster_colors.reserve(biggest_indices_pcl->size());
    for (unsigned int pt_idx = 0; pt_idx < biggest_indices_pcl->size(); ++pt_idx) {
      unsigned int data_index = (*biggest_indices_pcl)[pt_idx];
      cluster_pixels.push_back(kinect_openni_utils::world2pixel
                    <cv::Point2i>(depth_reprojected[data_index], depth_cam_model));
      cluster_colors.push_back(colors[data_index]);
    } // end loop pt_idx
    return true;
  } // end get_biggest_cluster_pixels()

protected:
  std::vector<cv::Point3d> depth_reprojected;
  std::vector<cv::Scalar> colors;
}; // end class ImageClusterer

#endif // IMAGE_CLUSTERER_H
