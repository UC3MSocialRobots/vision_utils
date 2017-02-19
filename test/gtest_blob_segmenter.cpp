/*!
  \file        gtest_blob_segmenter.cpp
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
// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "vision_utils/timer.h"
#include "vision_utils/imwrite_debug.h"
#include "vision_utils/matrix_testing.h"
#include "vision_utils/blob_segmenter.h"
#include "vision_utils/read_rgb_depth_user_image_from_image_file.h"
#include "vision_utils/user_image_to_rgb.h"
#include <vision_utils/img_path.h>

bool display = false;

TEST(TestSuite, empty_ctor) {
  vision_utils::BlobSegmenter segmenter;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, blob_vs_nite_empty_img) {
  vision_utils::BlobSegmenter segmenter;
  cv::Mat1f depth;
  cv::Mat1b out;
  bool ok = segmenter.find_blob(depth, cv::Point(0, 0), out);
  ASSERT_TRUE(!ok);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, seed_outside) {
  vision_utils::BlobSegmenter segmenter;
  cv::Mat3b rgb;
  cv::Mat1f depth;
  ASSERT_TRUE(vision_utils::read_rgb_and_depth_image_from_image_file
              (vision_utils::IMG_DIR() + "depth/juggling1", &rgb, &depth));
  cv::Mat1b out;
  bool ok = segmenter.find_blob(depth, cv::Point(10, depth.cols + 10), out);
  ASSERT_TRUE(!ok);
}

////////////////////////////////////////////////////////////////////////////////

void blob_vs_nite(const std::string & filename_prefix,
                  vision_utils::BlobSegmenter::CleaningMethod method,
                  uchar user_idx) {
  vision_utils::BlobSegmenter segmenter;
  cv::Mat3b rgb;
  cv::Mat1f depth;
  cv::Mat1b nite_user_mask, user_mask;
  ASSERT_TRUE(vision_utils::read_rgb_depth_user_image_from_image_file
              (filename_prefix, &rgb, &depth, &nite_user_mask));
  nite_user_mask = (nite_user_mask == user_idx);
  // find a random seed
  cv::Mat1b nite_user_mask_eroded; // erode to avoid points at the border
  cv::erode(nite_user_mask, nite_user_mask_eroded, cv::Mat(5, 5, CV_8U, 255));
  ASSERT_TRUE(cv::countNonZero(nite_user_mask_eroded));
  cv::Point seed(0, 0);
  while (nite_user_mask_eroded(seed) == 0) {
    seed.x = rand() % nite_user_mask.cols;
    seed.y = rand() % nite_user_mask.rows;
  }
  // printf("seed:%i;%i\n", seed.x, seed.y);
  cv::circle(rgb, seed, 5, CV_RGB(0, 255, 0), 3);

  vision_utils::Timer timer;
  bool ok = segmenter.find_blob(depth, seed, user_mask, method);
  timer.printTime("find_blob");
  ASSERT_TRUE(ok);
  if (display) {
    vision_utils::imwrite_debug("rgb.png", rgb,
                                vision_utils::COLOR_24BITS);
    vision_utils::imwrite_debug("depth.png", vision_utils::depth2viz(depth),
                                vision_utils::COLORS256);
    vision_utils::imwrite_debug("segmenter_final_mask.png", segmenter.get_final_mask(),
                                vision_utils::COLORS256);
    vision_utils::imwrite_debug("user_mask.png", user_mask,
                                vision_utils::MONOCHROME);
    //vision_utils::imwrite_debug("nite_user_mask.png", nite_user_mask);

    cv::imshow("rgb", rgb);
    cv::imshow("depth", vision_utils::depth2viz(depth));
    cv::imshow("segmenter_final_mask", segmenter.get_final_mask());
    cv::imshow("user_mask", user_mask);
    cv::imshow("nite_user_mask", nite_user_mask);
    cv::waitKey(0);
  } // end if display
  cv::Mat1b frame_diff;
  double rate_change = vision_utils::rate_of_changes_between_two_images
      (nite_user_mask, user_mask, frame_diff, (uchar) 1);
  printf("%s, method:%i: rate_change:%g\n",
         filename_prefix.c_str(), method, rate_change);
  ASSERT_NEAR(rate_change, 0, 5E-2);
}

TEST(TestSuite, blob_vs_nite_juggling1) {
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/juggling1", vision_utils::BlobSegmenter::FLOODFILL_EDGE_CLOSER, 255);
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/juggling1", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, 255);
}
TEST(TestSuite, blob_vs_nite_alberto1) {
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/alberto1", vision_utils::BlobSegmenter::FLOODFILL_EDGE_CLOSER, 255);
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/alberto1", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, 255);
}
TEST(TestSuite, blob_vs_nite_david_arnaud1) {
  // david
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud1", vision_utils::BlobSegmenter::FLOODFILL_EDGE_CLOSER, 1);
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud1", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, 1);
  // arnaud
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud1", vision_utils::BlobSegmenter::FLOODFILL_EDGE_CLOSER, 2);
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud1", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, 2);
}
TEST(TestSuite, blob_vs_nite_david_arnaud2) {
  // david

  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud2", vision_utils::BlobSegmenter::FLOODFILL_EDGE_CLOSER, 1);
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud2", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, 1);
  // arnaud
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud2", vision_utils::BlobSegmenter::FLOODFILL_EDGE_CLOSER, 2);
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud2", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, 2);
}
TEST(TestSuite, blob_vs_nite_david_arnaud3) {
  // david
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud3", vision_utils::BlobSegmenter::FLOODFILL_EDGE_CLOSER, 1);
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud3", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, 1);
  // arnaud
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud3", vision_utils::BlobSegmenter::FLOODFILL_EDGE_CLOSER, 2);
  blob_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud3", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, 2);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, blobs_vs_nite_empty_img) {
  vision_utils::BlobSegmenter segmenter;
  cv::Mat1f depth;
  std::vector< vision_utils::DisjointSets2::Comp > components_pts;
  std::vector<cv::Rect> boundingBoxes;
  ASSERT_TRUE(segmenter.find_all_blobs(depth, components_pts, boundingBoxes));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, blobs_vs_nite_one_blob) {
  int cols = 100;
  float dist = 1.f;
  cv::Mat1f depth(cols, cols, vision_utils::NAN_DEPTH);
  std::vector< vision_utils::DisjointSets2::Comp > components_pts;
  std::vector<cv::Rect> boundingBoxes;
  cv::Mat1b user_mask(cols, cols, (uchar) 0);
  cv::circle(user_mask, cv::Point(cols/2, cols/3), cols/4, 255, -1);
  depth.setTo(dist, user_mask);

  if (display) {
    cv::imshow("depth", vision_utils::depth2viz(depth));
    cv::imshow("user_mask", user_mask);
    cv::waitKey(0);
  } // end if display

  vision_utils::BlobSegmenter segmenter;
  ASSERT_TRUE(segmenter.find_all_blobs(depth, components_pts, boundingBoxes));
  ASSERT_TRUE(components_pts.size()== boundingBoxes.size());
  ASSERT_TRUE(components_pts.size() == 1);

  // check with min dist
  ASSERT_TRUE(segmenter.find_all_blobs(depth, components_pts, boundingBoxes,
                                       vision_utils::BlobSegmenter::NONE, NULL, dist+1, dist+2));
  ASSERT_TRUE(components_pts.size()== boundingBoxes.size());
  ASSERT_TRUE(components_pts.size() == 0);
}

////////////////////////////////////////////////////////////////////////////////

void generate_user_mask(const std::string & filename_prefix) {
  cv::Mat3b rgb;
  cv::Mat1f depth;
  ASSERT_TRUE(vision_utils::read_rgb_and_depth_image_from_image_file
              (filename_prefix, &rgb, &depth));
  vision_utils::BlobSegmenter segmenter;
  std::vector< vision_utils::DisjointSets2::Comp > components_pts;
  std::vector<cv::Rect> boundingBoxes;
  ASSERT_TRUE(segmenter.find_all_blobs
              (depth, components_pts, boundingBoxes,
               vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, NULL, 1., 3.5, -1, 200, false));

  cv::Mat1b user_mask;
  int nusers = segmenter.all_blobs_to_user_img
      (depth.size(), components_pts, user_mask);
  printf("Found %i users\n", nusers);
  ASSERT_TRUE(nusers > 0);
  user_mask = (user_mask > 0); // normalization
  // vision_utils::imwrite_debug(vision_utils::timestamp()+"_mask.png", user_mask, vision_utils::MONOCHROME);
}

TEST(TestSuite, generate_user_mask_juggling1) { generate_user_mask(vision_utils::IMG_DIR() + "depth/juggling1");}
TEST(TestSuite, generate_user_mask_juggling2) { generate_user_mask(vision_utils::IMG_DIR() + "depth/juggling2");}
TEST(TestSuite, generate_user_mask_juggling3) { generate_user_mask(vision_utils::IMG_DIR() + "depth/juggling3");}
TEST(TestSuite, generate_user_mask_alberto1) { generate_user_mask(vision_utils::IMG_DIR() + "depth/alberto1");}
TEST(TestSuite, generate_user_mask_alberto2) { generate_user_mask(vision_utils::IMG_DIR() + "depth/alberto2");}
TEST(TestSuite, generate_user_mask_alvaro1) { generate_user_mask(vision_utils::IMG_DIR() + "depth/alvaro1");}
TEST(TestSuite, generate_user_mask_alvaro2) { generate_user_mask(vision_utils::IMG_DIR() + "depth/alvaro2");}

////////////////////////////////////////////////////////////////////////////////

typedef std::vector<uchar> IdxVec;
void blobs_vs_nite(const std::string & filename_prefix,
                   vision_utils::BlobSegmenter::CleaningMethod method,
                   const IdxVec & nite_user_indices) {
  cv::Mat3b rgb;
  cv::Mat1f depth;
  cv::Mat1b nite_user_mask, all_comps;
  ASSERT_TRUE(vision_utils::read_rgb_depth_user_image_from_image_file
              (filename_prefix, &rgb, &depth, &nite_user_mask));

  vision_utils::BlobSegmenter segmenter;
  std::vector< vision_utils::DisjointSets2::Comp > components_pts;
  std::vector<cv::Rect> boundingBoxes;
  ASSERT_TRUE(segmenter.find_all_blobs(depth, components_pts, boundingBoxes,
                                       method, NULL, -1, -1, false));
  unsigned int nblobs = components_pts.size();
  ASSERT_TRUE(nblobs > 1);
  ASSERT_TRUE(boundingBoxes.size() == nblobs);
  // paint all components
  ASSERT_TRUE(segmenter.all_blobs_to_user_img(depth.size(), components_pts, all_comps)
              == nblobs);
  if (display) {
    cv::destroyAllWindows();
    cv::imshow("rgb", rgb);
    cv::imshow("depth", vision_utils::depth2viz(depth));
    //cv::imshow("all_comps", all_comps);
    cv::imshow("all_comps2rgb", vision_utils::user_image_to_rgb(all_comps));
    cv::waitKey(0); cv::destroyAllWindows();
  } // end if display

  // now try to find user
  cv::Mat1b user_mask;
  unsigned int nusers = nite_user_indices.size();
  vision_utils::Timer timer;
  ASSERT_TRUE(segmenter.find_all_blobs(depth, components_pts, boundingBoxes,
                                       method, NULL, 1., 3.5, nusers, -1, false));
  timer.printTime("find_all_blobs");
  ASSERT_TRUE(components_pts.size() >= nusers) // we want at least our users
      << "components_pts.size():" << components_pts.size() << ", nusers:" << nusers;
  // there should now only be one user
  ASSERT_TRUE(segmenter.all_blobs_to_user_img(depth.size(), components_pts, user_mask)
              == nusers);
  // paint each user
  int curr_idx = 0;
  for(IdxVec::const_iterator nite_idx = nite_user_indices.begin();
      nite_idx != nite_user_indices.end(); ++nite_idx) {
    ++curr_idx;
    cv::Mat1b frame_diff;
    cv::Mat1b curr_user_mask = (user_mask == curr_idx);
    cv::Mat1b curr_nite_user_mask = (nite_user_mask == *nite_idx);
    double rate_change = vision_utils::rate_of_changes_between_two_images
        (curr_nite_user_mask, curr_user_mask, frame_diff, (uchar) 1);
    if (display) {
      cv::imshow("user_mask2rgb", vision_utils::user_image_to_rgb(user_mask));
      cv::imshow("curr_user_mask", curr_user_mask);
      cv::imshow("curr_nite_user_mask", curr_nite_user_mask);
      cv::waitKey(0);
    } // end if display

    printf("%s, method:%i: rate_change:%g\n",
           filename_prefix.c_str(), method, rate_change);
    ASSERT_NEAR(rate_change, 0, 5E-2);
  } // end loop user_indices
}

TEST(TestSuite, blobs_vs_nite_juggling1) {
  blobs_vs_nite(vision_utils::IMG_DIR() + "depth/juggling1", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, IdxVec(1, 255));
}
TEST(TestSuite, blobs_vs_nite_alberto1) {
  blobs_vs_nite(vision_utils::IMG_DIR() + "depth/alberto1", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, IdxVec(1, 255));
}
TEST(TestSuite, blobs_vs_nite_david_arnaud1) {
  IdxVec vec; vec.push_back(1); vec.push_back(2);
  blobs_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud1", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, vec);
}
TEST(TestSuite, blobs_vs_nite_david_arnaud2) {
  IdxVec vec; vec.push_back(1); vec.push_back(2);
  blobs_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud2", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, vec);
}
TEST(TestSuite, blobs_vs_nite_david_arnaud3) {
  IdxVec vec; vec.push_back(1); vec.push_back(2);
  blobs_vs_nite(vision_utils::IMG_DIR() + "depth/david_arnaud3", vision_utils::BlobSegmenter::GROUND_PLANE_FINDER, vec);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
