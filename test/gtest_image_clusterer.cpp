/*!
  \file        test_image_clusterer.cpp
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

Some tests for class \a ImageClusterer

 */

bool display = false;

// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>
// utils
#include "vision_utils/image_clusterer.h"
#include <vision_utils/img_path.h>
#include "vision_utils/kinect_serials.h"
#include "vision_utils/read_camera_model_files.h"
#include "vision_utils/read_rgb_and_depth_image_from_image_file.h"
#include "vision_utils/timer.h"

void test_image_roi(const std::string & rgb_depth_filename_prefix,
                    const std::string & kinect_serial_number) {
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  ASSERT_TRUE(vision_utils::read_camera_model_files
              (kinect_serial_number, depth_camera_model, rgb_camera_model));

  // read depth and rgb files
  cv::Mat rgb, depth_img;
  ASSERT_TRUE(vision_utils::read_rgb_and_depth_image_from_image_file
              (rgb_depth_filename_prefix, &rgb, &depth_img));

  // get ROI
  cv::Rect roi(200, 200, 200, 200);
  unsigned int npts_roi = roi.width * roi.height;

  // reproject
  vision_utils::ImageClusterer clusterer;
  unsigned int data_step = 5;
  vision_utils::Timer timer;
  ASSERT_TRUE(clusterer.cluster(rgb(roi), depth_img(roi), depth_camera_model, data_step));
  timer.printTime("clusterer.cluster()");

  // get the cluster
  std::vector<cv::Point2i> cluster_pixels;
  std::vector<cv::Scalar> cluster_colors;
  ASSERT_TRUE(clusterer.get_biggest_cluster_pixels
              (depth_camera_model, cluster_pixels, cluster_colors));
  ASSERT_TRUE(cluster_pixels.size() == cluster_colors.size());
  unsigned int npts_cluster = cluster_pixels.size();

  if (display) {
    cv::Mat3b out = rgb.clone();
    cv::rectangle(out, roi, CV_RGB(255, 0, 0), 2); // paint selection
    for (unsigned int pt_idx = 0; pt_idx < npts_cluster; ++pt_idx)
      cv::circle(out, cluster_pixels[pt_idx], 2, CV_RGB(0, 255, 0), -1);
    cv::imshow("rgb", rgb);
    cv::imshow("out", out);
    cv::waitKey(0);
  } // end if display

  // check cluster somewhat big
  double ratio_cluster2roi = 1.f * npts_cluster * data_step * data_step / npts_roi,
      min_ratio = .25; // 25%
  ASSERT_TRUE(ratio_cluster2roi > min_ratio)
      << "npts_roi:" << npts_roi << ", npts_cluster:" << npts_cluster
      << ", data_step:" << data_step << " -> ratio_cluster2roi:" << ratio_cluster2roi;

  // check cluster is in ROI
  for (unsigned int pt_idx = 0; pt_idx < npts_cluster; ++pt_idx) {
    // check pt in ROI
    ASSERT_TRUE(roi.contains(cluster_pixels[pt_idx]));
    // check color match
    cv::Vec3b rgb_color3b = rgb.at<cv::Vec3b>(cluster_pixels[pt_idx]);
    cv::Scalar rgb_colorScalar(rgb_color3b[0], rgb_color3b[1], rgb_color3b[2]);
    ASSERT_TRUE(rgb_colorScalar == cluster_colors[pt_idx])
        << "rgb_colorScalar:" << rgb_colorScalar
        << ", cluster_colors[pt_idx]:" << cluster_colors[pt_idx];
  }
} // end test_image_roi()

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, all_tests) {
  test_image_roi(vision_utils::IMG_DIR() + "depth/inside1", vision_utils::KINECT_SERIAL_LAB());
  test_image_roi(vision_utils::IMG_DIR() + "depth/juggling1", vision_utils::KINECT_SERIAL_LAB());
  test_image_roi(vision_utils::IMG_DIR() + "depth/juggling2", vision_utils::KINECT_SERIAL_LAB());
  test_image_roi(vision_utils::IMG_DIR() + "depth/juggling3", vision_utils::KINECT_SERIAL_LAB());
  test_image_roi(vision_utils::IMG_DIR() + "depth/alberto1", vision_utils::KINECT_SERIAL_LAB());
  test_image_roi(vision_utils::IMG_DIR() + "depth/alberto2", vision_utils::KINECT_SERIAL_LAB());
  test_image_roi(vision_utils::IMG_DIR() + "depth/alvaro1", vision_utils::KINECT_SERIAL_LAB());
  test_image_roi(vision_utils::IMG_DIR() + "depth/alvaro2", vision_utils::KINECT_SERIAL_LAB());
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
