/*!
  \file        gtest_point_clouds.cpp
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

Some tests for \a
 */
// Bring in my package's API, which is what I'm testing
#include "vision_utils/clusterer.h"
#include "vision_utils/foo_point.h"
#include "vision_utils/timer.h"
// Bring in gtest
#include <gtest/gtest.h>

typedef vision_utils::FooPoint3f Pt3f;

TEST(TestSuite, empty) {
  { // a scope for calling destructor
  vision_utils::Clusterer clusterer;
  }
  ASSERT_NO_FATAL_FAILURE();
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_call) {
  vision_utils::Clusterer clusterer;
  vision_utils::Rect3d bbox;
  bool ret = clusterer.get_biggest_cluster_bbox(bbox);
  ASSERT_TRUE(ret == false); // failure

  std::vector<int> cluster_indices;
  ret = clusterer.get_biggest_cluster_indices(cluster_indices);
  ASSERT_TRUE(ret == false); // failure

  // cluster an emtpy cloud
  std::vector<Pt3f> data;
  ret = clusterer.cluster(data);
  ASSERT_TRUE(ret == true); // success
  ASSERT_TRUE(clusterer.get_cluster_nb() == 0);

  ret = clusterer.get_biggest_cluster_bbox(bbox);
  ASSERT_TRUE(ret == false); // failure
  ret = clusterer.get_biggest_cluster_indices(cluster_indices);
  ASSERT_TRUE(ret == false); // failure
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, single_pt) {
  vision_utils::Clusterer clusterer;
  std::vector<Pt3f> data;
  data.push_back( Pt3f(0, 0, 0));
  bool ret = clusterer.cluster(data);
  ASSERT_TRUE(ret == true); // success
  ASSERT_TRUE(clusterer.get_cluster_nb() == 1);

  vision_utils::Rect3d bbox;
  ret = clusterer.get_biggest_cluster_bbox(bbox);
  ASSERT_TRUE(ret == true); // success

  std::vector<int> cluster_indices;
  ret = clusterer.get_biggest_cluster_indices(cluster_indices);
  ASSERT_TRUE(ret == true); // success
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, two_clusters) {
  vision_utils::Clusterer clusterer;
  std::vector<Pt3f> data;
  Pt3f pt1(0, 0, 0), pt2(1, 1, 0);
  unsigned int pt1_amount = 10, pt2_amount = 5;
  for (unsigned int pt1_idx = 0; pt1_idx < pt1_amount; ++pt1_idx)
    data.push_back(pt1);
  for (unsigned int pt2_idx = 0; pt2_idx < pt2_amount; ++pt2_idx)
    data.push_back(pt2);
  bool ret = clusterer.cluster(data);
  ASSERT_TRUE(ret == true); // success
  ASSERT_TRUE(clusterer.get_cluster_nb() == 2);

  vision_utils::Rect3d bbox; // should only include pt1
  ret = clusterer.get_biggest_cluster_bbox(bbox);
  ASSERT_TRUE(ret == true); // success
  ASSERT_TRUE(bbox.centroid<Pt3f>() == pt1);

  std::vector<int> cluster_indices; // should contain 0 .. pt1_amount
  ret = clusterer.get_biggest_cluster_indices(cluster_indices);
  ASSERT_TRUE(ret == true); // success
  ASSERT_TRUE(cluster_indices.size() == pt1_amount);
  for (unsigned int pt_idx = 0; pt_idx < pt1_amount; ++pt_idx)
    ASSERT_TRUE(cluster_indices[pt_idx] == (int) pt_idx);
}

////////////////////////////////////////////////////////////////////////////////

// test coming from test_clusterer, needs to be converted to GTest \todo
void foo() {
  typedef vision_utils::FooPoint3d Pt3;
  std::vector<Pt3> pts;
  for (unsigned int pt_idx = 0; pt_idx < 500; ++pt_idx) {
    if (rand() % 100 < 80) // inliers
      pts.push_back(Pt3(drand48(), drand48(), drand48()));
    else
      pts.push_back(Pt3(10 * drand48(), 10 * drand48(), 10 * drand48()));
  } // end loop pt_idx

  vision_utils::Timer timer;
  vision_utils::Clusterer clusterer;
  bool success = clusterer.cluster(pts, 0.2);
  timer.printTime("clusterer.cluster()");
  printf("success:%i", success);
  if (!success)
    return;

  vision_utils::Rect3d bbox;
  // std::vector<int> cluster_indices;
  success =clusterer.get_biggest_cluster_bbox(bbox);

  printf("success:%i, bbox:'%s'", success, bbox.to_string().c_str());
  return;
}


////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
