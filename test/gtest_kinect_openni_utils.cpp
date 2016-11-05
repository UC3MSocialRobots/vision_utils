/*!
  \file        gtest_kinect_openni_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/23

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

Some tests for \b kinect_openni_utils namespace.

 */
// Bring in gtest
#include <gtest/gtest.h>

#include "vision_utils/kinect_serials.h"
#include "vision_utils/pixel2world_rgb.h"
#include "vision_utils/read_camera_model_files.h"
// ROS
#include <ros/package.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>

typedef cv::Point3f Pt3;
//typedef std_msgs::ColorRGBA Color;
typedef cv::Scalar Color;

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, pixel2world_rgb_empty_calls) {
  std::vector<Pt3> depth_reprojected;
  std::vector<Color> colors;
  bool ret;
  // empty call with no cam model -> should fail
  ret = vision_utils::pixel2world_rgb_color255(cv::Mat(), cv::Mat(),
                                                      image_geometry::PinholeCameraModel(),
                                                      depth_reprojected, colors,
                                                      1, cv::Mat(), true);
  ASSERT_TRUE(ret == false); // failure (no camera model)
  ASSERT_TRUE(colors.size() == 0) << "colors size:" << colors.size();

  // empty call with cam model -> should succeed
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  vision_utils::read_camera_model_files
      (vision_utils::DEFAULT_KINECT_SERIAL(), depth_camera_model, rgb_camera_model);
  ret = vision_utils::pixel2world_rgb_color255(cv::Mat(), cv::Mat(),
                                                      depth_camera_model,
                                                      depth_reprojected, colors,
                                                      1, cv::Mat(), true);
  ASSERT_TRUE(ret == true); // no failure
  ASSERT_TRUE(colors.size() == 0) << "colors size:" << colors.size();

  // one of the matrices empty -> should fail
  ret = vision_utils::pixel2world_rgb_color255(cv::Mat(), cv::Mat(10, 10, CV_8UC3),
                                                      image_geometry::PinholeCameraModel(),
                                                      depth_reprojected, colors,
                                                      1, cv::Mat(), false);
  ASSERT_TRUE(ret == false); // failure
  ASSERT_TRUE(colors.size() == 0) << "colors size:" << colors.size();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, pixel2world_nan) {
  std::vector<Pt3> depth_reprojected;
  std::vector<Color> colors;
  bool ret;
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  vision_utils::read_camera_model_files
      (vision_utils::DEFAULT_KINECT_SERIAL(), depth_camera_model, rgb_camera_model);
  // only NaNs -> should remove all of them
  cv::Mat3b color(10, 10, cv::Vec3b(255, 0, 0));
  cv::Mat1f depth;
  for (unsigned int idx = 0; idx < 1; ++idx) {
    switch (idx) {
      case 0:
        depth = cv::Mat1f(10, 10, (float) 0);
      case 1:
        depth = cv::Mat1f(10, 10, vision_utils::NAN_DOUBLE);
      case 2:
      default:
        depth = cv::Mat1f(10, 10, (double) 0);
    } // end switch (idx)
    ret = vision_utils::pixel2world_rgb_color255
          (color, depth, depth_camera_model, depth_reprojected, colors, 1, cv::Mat(), true);
    ASSERT_TRUE(ret == true); // no failure
    ASSERT_TRUE(colors.size() == 0) << "colors size:" << colors.size();
  } // end loop idx
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, pixel2world_rgb) {
  std::string IMG_DIR = ros::package::getPath("kinect") + std::string("/data/tests/");
  // read depth and rgb files
  cv::Mat3b rgb_img = cv::imread(IMG_DIR + "sample_rgb.jpg", CV_LOAD_IMAGE_COLOR);
  ASSERT_FALSE(rgb_img.empty());
  cv::Mat1f depth_img = cv::imread(IMG_DIR + "sample_depth.pgm", CV_LOAD_IMAGE_UNCHANGED);
  ASSERT_FALSE(depth_img.empty());
  ASSERT_TRUE(depth_img.size() == rgb_img.size());

  const std::string kinect_serial_number = vision_utils::KINECT_SERIAL_ARNAUD();
  std::vector<Pt3> depth_reprojected;
  std::vector<Color> colors;
  bool ret;
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  ASSERT_TRUE(vision_utils::read_camera_model_files
              (kinect_serial_number, depth_camera_model, rgb_camera_model));

  // rgb and depth sizes no corresponding -> should fail
  ASSERT_FALSE(vision_utils::pixel2world_rgb_color255
               (rgb_img, depth_img(cv::Rect(10, 10, 10, 10)),
                image_geometry::PinholeCameraModel(),
                depth_reprojected, colors,
                1, cv::Mat(), false));
  ASSERT_TRUE(colors.size() == 0) << "colors size:" << colors.size();

  // cam model not initialized -> should fail
  ASSERT_FALSE(vision_utils::pixel2world_rgb_color255
        (rgb_img, depth_img,
         image_geometry::PinholeCameraModel(),
         depth_reprojected, colors,
         1, cv::Mat(), false));
  ASSERT_TRUE(colors.size() == 0) << "colors size:" << colors.size();

  // mask size not corresponding -> should not use mask
  ret = vision_utils::pixel2world_rgb_color255
        (rgb_img, depth_img,
         depth_camera_model,
         depth_reprojected, colors,
         1, cv::Mat(10, 10, CV_8U), false);
  ASSERT_TRUE(ret == true); // no failure
  ASSERT_TRUE(colors.size() == depth_reprojected.size());
  ASSERT_TRUE((int) colors.size() == rgb_img.cols * rgb_img.rows);

  // different data step
  for (unsigned int data_step = 1; data_step <= 5; ++data_step) {
    printf("data_step:%i\n", data_step);
    ret = vision_utils::pixel2world_rgb_color255
          (rgb_img, depth_img,
           depth_camera_model,
           depth_reprojected, colors,
           data_step, cv::Mat(), false);
    unsigned int color_exp_size = rgb_img.cols * rgb_img.rows / (data_step * data_step);
    ASSERT_TRUE(ret == true); // no failure
    ASSERT_TRUE(colors.size() == depth_reprojected.size());
    EXPECT_NEAR(colors.size(), color_exp_size, .1f * color_exp_size)
        << "color size:" << colors.size() << ", expected:" << color_exp_size;
  } // end loop data_step

  // different sizes of Region of Interest
  for (unsigned int roi_size = 0; roi_size <= 100; roi_size+=10) {
    printf("roi_size:%i\n", roi_size);
    cv::Rect ROI(10, 10, roi_size, roi_size);
    ret = vision_utils::pixel2world_rgb_color255
          (rgb_img(ROI), depth_img(ROI),
           depth_camera_model,
           depth_reprojected, colors,
           1, cv::Mat(), false);
    ASSERT_TRUE(ret == true); // no failure
    ASSERT_TRUE(colors.size() == depth_reprojected.size());
    unsigned int color_exp_size = roi_size * roi_size;
    ASSERT_TRUE(colors.size() == color_exp_size)
        << "color size:" << colors.size() << ", expected:" << color_exp_size;
  } // end loop roi_size
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
