/*!
  \file        clusterer.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/19
  
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

Some useful functions for manipulating point clouds.

 */

#ifndef POINT_CLOUDS_H
#define POINT_CLOUDS_H

// PCL
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
// utils
#undef RAD2DEG // defined in both "pcl_macros.h" and "geometry_utils.h"
#undef DEG2RAD // defined in both "pcl_macros.h" and "geometry_utils.h"
#include "vision_utils/utils/Rect3.h"
#include "vision_utils/utils/pt_utils.h"

namespace geometry_utils {

/*!
 * \class Clusterer
 * A class for clustering point clouds easily
 */
class Clusterer {
public:
  Clusterer()
    : data_pcl (new pcl::PointCloud<pcl::PointXYZ>),
      tree (new pcl::search::KdTree<pcl::PointXYZ>) {
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Find the main cluster in a set of points.
   * from http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
   * \param data
   *    the input cloud
   * \param cluster_tolerance
   *    the spatial cluster tolerance as a measure in the L2 Euclidean space, in meters.
   *    A reasonable value would be 0.1f (10 cm).
   * \return
   *   true if success7
   */
  template<class Pt3>
  bool cluster(const std::vector<Pt3> & data,
               const double & cluster_tolerance = 0.1f) {
    // reset results
    cluster_indices_pcl.clear();

    if (data.size() == 0) {
      maggieDebug2("Clustering an empty cloud.");
      return true; // no error needed
    }

    // convert to pcl point cloud
    data_pcl->resize(data.size());
    for (unsigned int data_idx = 0; data_idx < data.size(); ++data_idx)
      pt_utils::copy3(data[data_idx], (*data_pcl)[data_idx]);

    // Creating the KdTree object for the search method of the extraction
    tree->setInputCloud (data_pcl);
    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (data.size() / 10);
    ec.setSearchMethod (tree);
    ec.setInputCloud (data_pcl);
    ec.extract (cluster_indices_pcl);
    //  ROS_INFO("cluster_indices_pcl:'%s'",
    //           StringUtils::accessible_to_string(cluster_indices_pcl).c_str());
    return true; // success
  } // end cluster()

  //////////////////////////////////////////////////////////////////////////////

  //! \return the number of clusters
  inline unsigned int get_cluster_nb() const {
    return cluster_indices_pcl.size();
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Get the indices of the points belonging to the biggest cluster
   * Can only be called after \a cluster()
   * \param cluster_indices
   *  [OUT] the indices of the \a data vector that was supplied to \a cluster()
   * \return true if success
   */
  bool get_biggest_cluster_indices(std::vector<int> & cluster_indices) const {
    cluster_indices.clear();
    if (cluster_indices_pcl.size() == 0) {
      ROS_WARN("No cluster found by PCL EuclideanClusterExtraction");
      return false;
    }
    const std::vector<int32_t>* biggest_indices_pcl =
        &(cluster_indices_pcl.front().indices);
    //  ROS_INFO("biggest_indices_pcl:'%s'",
    //           StringUtils::accessible_to_string(*biggest_indices_pcl).c_str());

    // copy cluster_indices_pcl -> cluster_indices
    std::copy(biggest_indices_pcl->begin(), biggest_indices_pcl->end(),
              std::back_inserter(cluster_indices));
    return true; // success
  } // end get_biggest_cluster_indices()

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Get the 3D bounding box of the biggest cluster computed by \a cluster().
   * Can only be called after \a cluster()
   * \param bbox
   *    [OUT] The 3D bounding box of the main cluster.
   * \return true if success
   */
  bool get_biggest_cluster_bbox(geometry_utils::Rect3d & bbox) const {
    bbox = geometry_utils::Rect3d();
    if (cluster_indices_pcl.size() == 0) {
      ROS_WARN("No cluster found by PCL EuclideanClusterExtraction");
      return false;
    }
    // now compute bbox
    const std::vector<int32_t>* biggest_indices_pcl =
        &(cluster_indices_pcl.front().indices);
    // bbox = boundingBox_vec3<double, Pt3, std::vector<Pt3> >(data);
    for (unsigned int pt_idx = 0; pt_idx < biggest_indices_pcl->size(); ++pt_idx)
      bbox.extendToInclude((*data_pcl)[ (*biggest_indices_pcl)[pt_idx] ]);
    return true; // success
  } // end get_biggest_cluster_bbox()

  //////////////////////////////////////////////////////////////////////////////

protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_pcl;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
  std::vector<pcl::PointIndices> cluster_indices_pcl;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
}; // end class Clusterer

////////////////////////////////////////////////////////////////////////////////

} // end namespace geometry_utils

#endif // POINT_CLOUDS_H
