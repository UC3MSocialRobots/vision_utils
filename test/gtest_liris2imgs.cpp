/*!
  \file        gtest_liris2imgs.cpp
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
#include <databases_io/liris2imgs.h>

//#define LIRIS_DIR "/home/user/Downloads/0datasets/liris/training-validation/"
#define LIRIS_DIR "/home/arnaud/disk/Datasets/liris/training-validation/"
inline bool check_dir_exists() {
  if (system_utils::directory_exists(LIRIS_DIR))
    return true;
  printf("/!\\ Directory " LIRIS_DIR " does not exist\n");
  return false;
}

TEST(TestSuite, ctor) {
  Liris2Imgs db;
  ASSERT_TRUE(db.get_playlist_size() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, simple_load) {
  if (!check_dir_exists()) return;
  Liris2Imgs db;
  ASSERT_TRUE(db.from_file(LIRIS_DIR "vid0001.xml"));
  unsigned int nframes = db.n_frames_in_curr_video();
  ASSERT_TRUE(nframes == 401);
  ASSERT_TRUE(db.n_annotated_frames_in_curr_video() == 0);

#ifdef DISPLAY
  db.display();
  cv::waitKey(0);
#endif // DISPLAY
  for (unsigned int frame_idx = 0; frame_idx < nframes - 1; ++frame_idx) {
    ASSERT_TRUE(db.go_to_next_frame());
  } // end for (frame_idx)
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, simple_load2) {
  if (!check_dir_exists()) return;
  Liris2Imgs db;
  ASSERT_TRUE(db.from_file(LIRIS_DIR "vid0002.xml"));
  unsigned int nframes = db.n_frames_in_curr_video();
  ASSERT_TRUE(nframes == 574);
  ASSERT_TRUE(db.n_annotated_frames_in_curr_video() == 300);
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

TEST(TestSuite, simple_load3) {
  if (!check_dir_exists()) return;
  Liris2Imgs db;
  std::vector<std::string> xml_files;
  system_utils::all_files_in_dir(LIRIS_DIR, xml_files, ".xml", true);
  for (unsigned int file_idx = 0; file_idx < xml_files.size(); ++file_idx) {
    ASSERT_TRUE(db.from_file(xml_files[file_idx]));
#ifdef DISPLAY
    db.display();
    cv::waitKey(0);
#endif // DISPLAY
  } // end loop file_idx
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
