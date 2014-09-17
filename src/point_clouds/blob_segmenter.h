/*!
  \file        blob_segmenter.h
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

#ifndef BLOB_SEGMENTER_H
#define BLOB_SEGMENTER_H

#include <vision_utils/image_utils/depth_canny.h>
#include <vision_utils/image_utils/content_processing.h>
#include <vision_utils/connected_comp/disjoint_sets2.h>
#include "vision_utils/point_clouds/ground_plane_finder.h"

class BlobSegmenter {
public:
  BlobSegmenter() {}

  enum CleaningMethod {
    NONE = 0,
    GROUND_PLANE_FINDER = 1,
    FLOODFILL_EDGE_CLOSER = 2
  };

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Get the blob including a given seed.
   * The edges are detected thanks to a Canny fulter.
   * \param depth
   *    the depth map
   * \param seed
   *    a point that belongs to the user
   * \param out
   *    the generated user map, null where no user, > 0 in the user blob
   * \param method
   *    a method for closing blobs at the level of the ground.
   *    Choose between NONE, GROUND_PLANE_FINDER, FLOODFILL_EDGE_CLOSER
   * \param depth_cam_model
   *    the model of the camera that generated the depth map
   * \param canny_thres1, canny_thres2
   *    \see DepthCanny::set_canny_thresholds()
   * \param ground_distance_threshold_m, ground_lower_ratio_to_use, data_skip
   *    \see GroundPlaneFinder::compute_plane()
   * \param prev_line_diff_thres, src_width_ratio_thres
   *    \see image_utils::FloodFillEdgeCloser::close()
   * \return true if success
   */
  bool find_blob(const cv::Mat1f & depth,
                 const cv::Point & seed,
                 cv::Mat1b & out,
                 CleaningMethod method = NONE,
                 const image_geometry::PinholeCameraModel* depth_cam_model = NULL,
                 bool ground_recompute_plane = true,
                 double canny_thres1 = DepthCanny::DEFAULT_CANNY_THRES1,
                 double canny_thres2 = DepthCanny::DEFAULT_CANNY_THRES2,
                 double ground_distance_threshold_m = GroundPlaneFinder::DEFAULT_DISTANCE_THRESHOLD_M,
                 double ground_lower_ratio_to_use = GroundPlaneFinder::DEFAULT_LOWER_RATIO_TO_USE,
                 int data_skip = GroundPlaneFinder::DEFAULT_DATA_SKIP,
                 double prev_line_diff_thres = image_utils::FloodFillEdgeCloser::DEFAULT_PREV_LINE_DIFF_THRES,
                 double src_width_ratio_thres = image_utils::FloodFillEdgeCloser::DEFAULT_SRC_WIDTH_RATIO_THRES)
  {

    if (!image_utils::bbox_full(depth).contains(seed)) {
      printf("BlobSegmenter: seed(%i, %i) out of depth image '%s'!\n",
             seed.x, seed.y, image_utils::infosImage(depth).c_str());
      return false;
    }

    // apply a Canny filter on depth
    _depth_canny.set_canny_thresholds(canny_thres1, canny_thres2);
    _depth_canny.thresh(depth);
    _depth_canny.get_thresholded_image().copyTo(_final_mask);
    // cv::imshow("_final_mask", _final_mask); cv::waitKey(0);
    //cv::imwrite("final_mask.png", _final_mask);
    // seed on edge -> fail
    if (_final_mask(seed) == 0) {
      printf("BlobSegmenter: seed(%i, %i) on depth edges!\n", seed.x, seed.y);
      return false;
    }

    // try to close contour with a dirty floodfill, if wanted
    if (method == FLOODFILL_EDGE_CLOSER
        && image_utils::bbox_full(depth).contains(seed)
        && !_closer.close(_final_mask, seed, true, false, (uchar) 0,
                          prev_line_diff_thres, src_width_ratio_thres))
      return false;

    // try to detect ground if wanted
    if (method == GROUND_PLANE_FINDER) {
      // compute plane if needed
      if ((ground_recompute_plane || !_ground_finder.is_plane_found())
          && !_ground_finder.compute_plane
          (depth, depth_cam_model, ground_distance_threshold_m, ground_lower_ratio_to_use, data_skip)) {
        printf("find_all_blobs(): GroundPlaneFinder::compute_plane() failed!\n");
        return false;
      }
      if (!_ground_finder.to_img
          (depth, _not_ground_mask, -1, -1, depth_cam_model, ground_distance_threshold_m, false))
        return false;
      // combine with _ground_mask
      _final_mask &= _not_ground_mask;
    } // end if (method == GROUND_PLANE_FINDER)

    if (_final_mask(seed) == 0) {
      printf("BlobSegmenter: seed(%i, %i) on final mask!\n", seed.x, seed.y);
      return false;
    }

    // make a flood fill on the edged
    uchar flood_fill_color = 127; // must be != 0, 255
    cv::floodFill(_final_mask, seed, cv::Scalar::all(flood_fill_color));
    out = (_final_mask == flood_fill_color);
    //cv::imshow("_final_mask", _final_mask); // cv::waitKey(0);
    return true;
  } // end find_blob()

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * find all blobs in the given depth range and sort them by size (biggest first)
   * \param depth
   *    the depth map
   * \param components_pts
   *    the connected components corresponding to the users
   * \param boundingBoxes
   *    the bounding boxes of \a components_pts
   * \param method
   *    a method for closing blobs at the level of the ground.
   *    Choose between NONE, GROUND_PLANE_FINDER, FLOODFILL_EDGE_CLOSER
   * \param depth_cam_model
   *    the model of the camera that generated the depth map
   * \param min_dist_m, max_dist_m
   *    the range of depth values where we will look for users, in meters.
   *    -1 for not using this filter.
   * \param max_blobs_nb
   *    the maximum allowed number of users.
   *    the real number of found users is in [0, max_blobs_nb],
   *    it can be obtained with boundingBoxes.size().
   * -1 for not using this filter.
   * \param min_blob_size_pix
   *    the minimum size in pixels for the found blobs.
   *    The smaller blobs will be dismissed.
   *    -1 for not using this filter.
   * \param ground_recompute_plane
   *    If true (default), the ground plane is estimated thanks
   *    to the lower half of the depth map.
   *    If false, the previous ground plane estimation by GroundPlaneFinder will be skipt.
   * \param canny_thres1, canny_thres2
   *    \see DepthCanny::set_canny_thresholds()
   * \param ground_distance_threshold_m, ground_lower_ratio_to_use, data_skip
   *    \see GroundPlaneFinder::compute_plane()
   * \return true if success
   */
  bool find_all_blobs(const cv::Mat1f & depth,
                      std::vector< DisjointSets2::Comp > & components_pts,
                      std::vector<cv::Rect> & boundingBoxes,
                      CleaningMethod method = NONE,
                      const image_geometry::PinholeCameraModel* depth_cam_model = NULL,
                      double min_dist_m = -1, double max_dist_m = -1,
                      int max_blobs_nb = -1, int min_blob_size_pix = -1,
                      bool ground_recompute_plane = true,
                      double canny_thres1 = DepthCanny::DEFAULT_CANNY_THRES1,
                      double canny_thres2 = DepthCanny::DEFAULT_CANNY_THRES2,
                      double ground_distance_threshold_m = GroundPlaneFinder::DEFAULT_DISTANCE_THRESHOLD_M,
                      double ground_lower_ratio_to_use = GroundPlaneFinder::DEFAULT_LOWER_RATIO_TO_USE,
                      int data_skip = GroundPlaneFinder::DEFAULT_DATA_SKIP) {
    if (method == FLOODFILL_EDGE_CLOSER) {
      printf("find_all_blobs(): FLOODFILL_EDGE_CLOSER cant be used with this method\n");
      return false;
    }

    // apply a Canny filter on depth
    _depth_canny.set_canny_thresholds(canny_thres1, canny_thres2);
    _depth_canny.thresh(depth);
    // cv::imshow("_final_mask", _final_mask); cv::waitKey(0);
    //cv::imwrite("final_mask.png", _final_mask);

    if (method == GROUND_PLANE_FINDER) {
      // compute plane if needed
      if ((ground_recompute_plane || !_ground_finder.is_plane_found())
          && !_ground_finder.compute_plane
          (depth, depth_cam_model, ground_distance_threshold_m, ground_lower_ratio_to_use, data_skip)) {
        printf("find_all_blobs(): GroundPlaneFinder::compute_plane() failed!\n");
        return false;
      }
      if (!_ground_finder.to_img
          (depth, _not_ground_mask, min_dist_m, max_dist_m,
           depth_cam_model, ground_distance_threshold_m, false)){
        printf("find_all_blobs(): GroundPlaneFinder::to_img() failed!\n");
        return false;
      }
      // combine with _ground_mask
      _final_mask = _not_ground_mask & _depth_canny.get_thresholded_image();
    }
    else { // method NONE
      _depth_canny.get_thresholded_image().copyTo(_final_mask);
      // use min_dist, max_dist if needed
      bool use_min_dist = (min_dist_m > 0);
      bool use_max_dist = (max_dist_m > 0);
      if (use_min_dist || use_max_dist) {
        int cols = depth.cols, rows = depth.rows;
        for (int row = 0; row < rows; ++row) {
          const float* depth_ptr = depth.ptr<float>(row);
          uchar* mask_ptr = _final_mask.ptr(row);
          for (int col = 0; col < cols; ++col) {
            if (!std_utils::is_nan_depth(depth_ptr[col])
                && ((use_min_dist && depth_ptr[col] < min_dist_m)
                    || (use_max_dist && depth_ptr[col] > max_dist_m)))
              mask_ptr[col] = 0;
          } // end loop col
        } // end loop row
      }
    } // end if method NONE

    // if (!_final_mask.empty()) cv::imshow("_final_mask", _final_mask); cv::waitKey(0);

    // get all blobs
    _set.process_image(_final_mask);
    _set.get_connected_components(depth.cols, components_pts, boundingBoxes);
    if (!_set.sort_comps_by_decreasing_size(components_pts, boundingBoxes))
      return false;

    // remove the undesired components
    unsigned int nblobs = components_pts.size();
    for (unsigned int blob_idx = 0; blob_idx < nblobs; ++blob_idx) {
      unsigned int npts = components_pts[blob_idx].size();
      if ( (max_blobs_nb > 0 && (int) blob_idx >= max_blobs_nb)
           || (min_blob_size_pix > 0 && (int) npts < min_blob_size_pix)) {
        //  printf("find_all_blobs(): blob #%i, %i px does not comply: "
        //         "max_blobs_nb:%i, min_blob_size_pix:%i px. Erasing all the following.\n",
        //         blob_idx, npts, max_blobs_nb, min_blob_size_pix);
        components_pts.erase(components_pts.begin() + blob_idx, components_pts.end());
        boundingBoxes.erase(boundingBoxes.begin() + blob_idx, boundingBoxes.end());
        break;
      }
    } // end for blob_idx

    return true;
  } // end find_all_blobs()

  //////////////////////////////////////////////////////////////////////////////

  //! return the real number of users drawn
  unsigned int all_blobs_to_user_img(const cv::Size & depth_in_size,
                                     const std::vector< DisjointSets2::Comp > & components_pts,
                                     cv::Mat1b & out) const {
    int nblobs = components_pts.size();
    out.create(depth_in_size);
    out.setTo(0);
    for (int blob_idx = 0; blob_idx < nblobs; ++blob_idx) {
      uchar out_value = (blob_idx+1);
      const DisjointSets2::Comp* curr_blob = &(components_pts[blob_idx]);
      unsigned int npts = curr_blob->size();
      for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
        out((*curr_blob)[pt_idx]) = out_value;
    } // end loop blob_idx
    return nblobs;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool generate_ground_mask
  (const cv::Mat1f & depth,
   cv::Mat1b & out,
   const image_geometry::PinholeCameraModel* depth_cam_model = NULL,
   double min_dist_m = -1, double max_dist_m = -1,
   double ground_distance_threshold_m = GroundPlaneFinder::DEFAULT_DISTANCE_THRESHOLD_M)
  const {
    return _ground_finder.to_img(depth, out, min_dist_m, max_dist_m,
                                 depth_cam_model, ground_distance_threshold_m, true);
  }

  //////////////////////////////////////////////////////////////////////////////

  const cv::Mat1b & get_final_mask() const { return _final_mask; }

  //////////////////////////////////////////////////////////////////////////////

private:
  DepthCanny _depth_canny;
  cv::Mat1b _final_mask;
  image_utils::FloodFillEdgeCloser _closer;
  GroundPlaneFinder _ground_finder;
  cv::Mat1b _not_ground_mask;
  DisjointSets2 _set;

}; // end class BlobSegmenter
#endif // BLOB_SEGMENTER_H
