/*!
  \file        gtest_filename_prefix2imgs.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/11

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

Some tests for vision_utils::FilenamePrefix2Imgs.
 */
bool display = false;
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "vision_utils/filename_prefix2imgs.h"
#include <vision_utils/img_path.h>

TEST(TestSuite, ctor) {
  vision_utils::FilenamePrefix2Imgs db;
  ASSERT_TRUE(db.get_playlist_size() == 0);
}

////////////////////////////////////////////////////////////////////////////////

void test_load(const std::string & filename_regex,
               unsigned int nframes_exp,
               vision_utils::DatabasePlayer::UserIdx exp_user_idx) {
  vision_utils::FilenamePrefix2Imgs db;
  ASSERT_TRUE(db.from_file(filename_regex));
  unsigned int nframes = db.get_playlist_size();
  ASSERT_TRUE(nframes == nframes_exp);
  ASSERT_TRUE(db.get_user_idx() == exp_user_idx)
      << "db.get_user_idx():" << db.get_user_idx()
      << ", exp_user_idx:" << exp_user_idx;
if (display) {
  db.display();
  cv::waitKey(0);
    } // end if display
  for (unsigned int frame_idx = 0; frame_idx < nframes - 1; ++frame_idx) {
    ASSERT_TRUE(db.go_to_next_frame());
  } // end for (frame_idx)
}

TEST(TestSuite, alberto1)  { test_load(vision_utils::IMG_DIR() + "depth/alberto1_rgb.png", 1, 1); }
TEST(TestSuite, alberto2)  { test_load(vision_utils::IMG_DIR() + "depth/alberto*_rgb.png", 2, 1); }
TEST(TestSuite, juggling)  { test_load(vision_utils::IMG_DIR() + "depth/juggling*_rgb.png", 3, 0); }
TEST(TestSuite, all_depth) { test_load(vision_utils::IMG_DIR() + "depth/*_rgb.png", 12, 1); }
TEST(TestSuite, breast)    { test_load(vision_utils::IMG_DIR() + "breast/*_rgb.png", 22, 4); }

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
