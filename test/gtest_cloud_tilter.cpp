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
#include <opencv2/core/core.hpp>
#include "kinect/kinect_openni_utils.h"
#include <combinatorics/combinatorics_utils.h>
#include <time/timer.h>
// vision_utils
#include "point_clouds/cloud_viewer_gnuplot.h"
#include "point_clouds/cloud_tilter.h"
#include "databases_io/dgaitdb_filename.h"
#include "image_utils/io.h"
#include <vision_utils/img_path.h>

//#define DISPLAY

typedef cv::Point3f Pt3f;

TEST(TestSuite, empty) {
  CloudTilter tilter;
}

void test_straighten_picture(const std::string & filename_prefix,
                             const std::string & kinect_serial_number,
                             uchar user_idx) {
  cv::Mat1b user_mask;
  cv::Mat3b rgb;
  cv::Mat1f depth;
  ASSERT_TRUE(image_utils::read_rgb_depth_user_image_from_image_file
      (filename_prefix, &rgb, &depth, &user_mask));
  user_mask = (user_mask == user_idx);
  ASSERT_TRUE(cv::countNonZero(user_mask) > 0);
  // load default camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  ASSERT_TRUE(kinect_openni_utils::read_camera_model_files
              (kinect_serial_number, depth_camera_model, rgb_camera_model));
  // transform in 3D
  std::vector<Pt3f> pts;
  ASSERT_TRUE(kinect_openni_utils::pixel2world_depth(depth, depth_camera_model, pts, 2,
                                         user_mask));
  // view cloud
  CloudViewerGnuPlot viewer;
  //viewer.view_cloud(pts);
  // tilt
  CloudTilter tilter;
  Timer timer;
  ASSERT_TRUE(tilter.straighten(pts));
  timer.printTime("straighten()");
#ifdef DISPLAY
  viewer.view_cloud(pts);
#endif // DISPLAY
}

void test_straighten_picture(int dgaitdb_index) {
  // load user mask and depth
  DGaitDBFilename f("/home/user/Downloads/0datasets/DGaitDB_imgs/");
  if (!f.directory_exists())
    return;
  std::vector<std::string> files = f.all_filenames_test();
  std::string filename_prefix = files[dgaitdb_index];
  test_straighten_picture(filename_prefix, DEFAULT_KINECT_SERIAL(), 1);
}

TEST(TestSuite, dgaitdb_picture_user0) { test_straighten_picture(0); }
TEST(TestSuite, dgaitdb_picture_user1) { test_straighten_picture(DGaitDBFilename::NFILES_TEST+1); }
TEST(TestSuite, dgaitdb_all_users) {
  for (unsigned int oni_idx = 0; oni_idx < DGaitDBFilename::ONI_FILES; ++oni_idx)
    test_straighten_picture(DGaitDBFilename::NFILES_TEST*oni_idx + 1);
}

TEST(TestSuite, dgaitdb_ltm_depth) {
  test_straighten_picture(IMG_DIR "depth/alberto1", KINECT_SERIAL_LAB(), 255);
  test_straighten_picture(IMG_DIR "depth/david_arnaud1", KINECT_SERIAL_LAB(), 1);
  test_straighten_picture(IMG_DIR "depth/david_arnaud1", KINECT_SERIAL_LAB(), 2);
  test_straighten_picture(IMG_DIR "depth/david_arnaud2", KINECT_SERIAL_LAB(), 1);
  test_straighten_picture(IMG_DIR "depth/david_arnaud2", KINECT_SERIAL_LAB(), 2);
  test_straighten_picture(IMG_DIR "depth/david_arnaud3", KINECT_SERIAL_LAB(), 1);
  test_straighten_picture(IMG_DIR "depth/david_arnaud3", KINECT_SERIAL_LAB(), 2);
  test_straighten_picture(IMG_DIR "depth/juggling1", KINECT_SERIAL_LAB(), 255);
  test_straighten_picture(IMG_DIR "depth/juggling2", KINECT_SERIAL_LAB(), 255);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
