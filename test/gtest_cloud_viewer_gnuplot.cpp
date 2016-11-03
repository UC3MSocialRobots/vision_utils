/*!
  \file        gtest_cloud_viewer_gnuplot.cpp
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

 */
// Bring in my package's API, which is what I'm testing
#include "vision_utils/cloud_viewer_gnuplot.h"
#include "vision_utils/rand_gaussian.h"

// Bring in gtest
#include <gtest/gtest.h>

typedef cv::Point3f Pt3f;

#define BLOCKING false

TEST(TestSuite, empty) {
  vision_utils::CloudViewerGnuPlot viewer;
  std::vector<Pt3f> pointcloud;
  std::vector<cv::Vec3b> pointcloud_RGB;
  viewer.view_cloud(pointcloud, "Caption", BLOCKING);
  viewer.view_rgb_cloud(pointcloud, pointcloud_RGB, "Caption", BLOCKING);
}

////////////////////////////////////////////////////////////////////////////////

void test_gaussian(bool use_rgb) {
  std::vector<Pt3f> pointcloud;
  std::vector<cv::Vec3b> pointcloud_RGB;
  for (unsigned int pt = 0; pt < 100; ++pt) {
    pointcloud.push_back(Pt3f(vision_utils::rand_gaussian() / 10.f,
                              vision_utils::rand_gaussian() / 10.f,
                              vision_utils::rand_gaussian() / 10.f));
    pointcloud_RGB.push_back(cv::Vec3b(50 + rand() % 200,
                                       50 + rand() % 200,
                                       50 + rand() % 200));
  } // end for pt
  vision_utils::CloudViewerGnuPlot viewer;
  if (use_rgb)
    viewer.view_rgb_cloud(pointcloud, pointcloud_RGB, "Caption", BLOCKING);
  else
    viewer.view_cloud(pointcloud, "Caption", BLOCKING);
}

TEST(TestSuite, gaussian_bw) {
  test_gaussian(false);
}

TEST(TestSuite, gaussian_rgb) {
  test_gaussian(true);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
