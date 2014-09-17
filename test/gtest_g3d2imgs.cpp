/*!
  \file        gtest_g3d2imgs.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/7

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
//#define DISPLAY

#include <gtest/gtest.h>
#include <vision_utils/databases_io/g3d2imgs.h>

#define G3D_DIR "/home/user/Downloads/0datasets/g3d_kingston/"
//#define G3D_DIR "/home/arnaud/disk/Datasets/g3d_kingston/"
inline bool check_dir_exists() {
  if (system_utils::directory_exists(G3D_DIR))
    return true;
  printf("/!\\ Directory " G3D_DIR " does not exist\n");
  return false;
}

TEST(TestSuite, ctor) {
  G3D2Imgs db;
  ASSERT_TRUE(db.get_playlist_size() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, simple_load) {
  if (!check_dir_exists()) return;
  G3D2Imgs db;
  ASSERT_TRUE(db.from_file(G3D_DIR "Fighting/KinectOutput22/Colour/Colour 12.png"));
  unsigned int nframes = db.n_frames_in_curr_video();
  ASSERT_TRUE(nframes > 400);
  for (unsigned int frame_idx = 0; frame_idx < nframes - 1; ++frame_idx) {
#ifdef DISPLAY
    db.display();
    if (frame_idx == 0)
      cv::waitKey(0);
#endif // DISPLAY
    ASSERT_TRUE(db.go_to_next_frame());
  } // end for (frame_idx)
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
