/*!
  \file        gtest_cloud_tilter.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/27

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
// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <image_geometry/pinhole_camera_model.h>
// vision_utils
#include "vision_utils/cloud_viewer_gnuplot.h"
#include "vision_utils/cloud_tilter.h"
#include "vision_utils/dgaitdb_filename.h"
#include <vision_utils/img_path.h>
#include "vision_utils/kinect_serials.h"
#include "vision_utils/timer.h"
#include "vision_utils/read_rgb_depth_user_image_from_image_file.h"
#include "vision_utils/read_camera_model_files.h"
#include "vision_utils/pixel2world_depth.h"

bool display = false;

typedef cv::Point3f Pt3f;

TEST(TestSuite, empty) {
  vision_utils::CloudTilter tilter;
}

void test_straighten_picture(const std::string & filename_prefix,
                             const std::string & kinect_serial_number,
                             uchar user_idx) {
  cv::Mat1b user_mask;
  cv::Mat3b rgb;
  cv::Mat1f depth;
  ASSERT_TRUE(vision_utils::read_rgb_depth_user_image_from_image_file
      (filename_prefix, &rgb, &depth, &user_mask));
  user_mask = (user_mask == user_idx);
  ASSERT_TRUE(cv::countNonZero(user_mask) > 0);
  // load default camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  ASSERT_TRUE(vision_utils::read_camera_model_files
              (kinect_serial_number, depth_camera_model, rgb_camera_model));
  // transform in 3D
  std::vector<Pt3f> pts;
  ASSERT_TRUE(vision_utils::pixel2world_depth(depth, depth_camera_model, pts, 2,
                                         user_mask));
  // view cloud
  vision_utils::CloudViewerGnuPlot viewer;
  //viewer.view_cloud(pts);
  // tilt
  vision_utils::CloudTilter tilter;
  vision_utils::Timer timer;
  ASSERT_TRUE(tilter.straighten(pts));
  timer.printTime("straighten()");
if (display) {
  viewer.view_cloud(pts);
    } // end if display
}

void test_straighten_picture(int dgaitdb_index) {
  // load user mask and depth
  vision_utils::DGaitDBFilename f("/home/user/Downloads/0datasets/DGaitDB_imgs/");
  if (!f.directory_exists())
    return;
  std::vector<std::string> files = f.all_filenames_test();
  std::string filename_prefix = files[dgaitdb_index];
  test_straighten_picture(filename_prefix, vision_utils::DEFAULT_KINECT_SERIAL(), 1);
}

TEST(TestSuite, dgaitdb_picture_user0) { test_straighten_picture(0); }
TEST(TestSuite, dgaitdb_picture_user1) { test_straighten_picture(vision_utils::DGaitDBFilename::NFILES_TEST+1); }
TEST(TestSuite, dgaitdb_all_users) {
  for (unsigned int oni_idx = 0; oni_idx < vision_utils::DGaitDBFilename::ONI_FILES; ++oni_idx)
    test_straighten_picture(vision_utils::DGaitDBFilename::NFILES_TEST*oni_idx + 1);
}

TEST(TestSuite, dgaitdb_ltm_depth) {
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/alberto1", vision_utils::DEFAULT_KINECT_SERIAL(), 255);
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/david_arnaud1", vision_utils::DEFAULT_KINECT_SERIAL(), 1);
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/david_arnaud1", vision_utils::DEFAULT_KINECT_SERIAL(), 2);
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/david_arnaud2", vision_utils::DEFAULT_KINECT_SERIAL(), 1);
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/david_arnaud2", vision_utils::DEFAULT_KINECT_SERIAL(), 2);
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/david_arnaud3", vision_utils::DEFAULT_KINECT_SERIAL(), 1);
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/david_arnaud3", vision_utils::DEFAULT_KINECT_SERIAL(), 2);
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/juggling1", vision_utils::DEFAULT_KINECT_SERIAL(), 255);
  test_straighten_picture(vision_utils::IMG_DIR() + "depth/juggling2", vision_utils::DEFAULT_KINECT_SERIAL(), 255);
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
