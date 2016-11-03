/*!
  \file        dgaitdb.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/12

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

#include "vision_utils/dgaitdb.h"
#include "vision_utils/dgaitdb_filename.h"
#include "vision_utils/write_rgb_depth_user_to_image_file.h"

int video_reader(int argc, char **argv) {
  if (argc < 1) {
    printf("Synopsis: %s [onifile]\n", argv[0]);
    return -1;
  }
  std::string filename = argv[1];
  vision_utils::DGaitDB reader;
  if (!reader.from_file(filename))
    return -1;
  while(true) {
    vision_utils::Timer timer;
    if (!reader.go_to_next_frame())
      break;
    timer.printTime("go_to_next_frame()");
    char c = reader.display();
    if ((int) c == 27)
      break;
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////

int generate_training_data() {
  vision_utils::DGaitDBFilename f("/home/user/Downloads/0datasets/DGaitDB_imgs/");
  assert(f.directory_exists());
  vision_utils::DGaitDB reader;
  cv::Mat3b user_illus;

  for (unsigned int file_idx = 1; file_idx <= vision_utils::DGaitDBFilename::ONI_FILES;
       ++file_idx) {
    std::ostringstream oni_filename;
    oni_filename << "/home/user/Downloads/0datasets/DGaitDB/" << file_idx << ".oni";;
    printf("generate_training_data():current file: '%s'\n", oni_filename.str().c_str());
    if (!reader.from_file(oni_filename.str()))
      continue;
    unsigned int nfiles_train_written = 0, nfiles_test_written = 0, nfiles_total_written = 0;
    while(reader.go_to_next_frame()) {
      reader.display();
      if (reader.nusers() != 1) {
        printf("There are %i users, skipping.\n", reader.nusers());
        continue;
      }
      // determine if write test or training
      bool write_test = (nfiles_test_written < vision_utils::DGaitDBFilename::NFILES_TEST
                         && nfiles_total_written % 2 == 0);
      // write the wanted files
      std::string img_prefix = (write_test ? f.filename_test(file_idx, nfiles_test_written+1)
                                           : f.filename_train(file_idx, nfiles_train_written+1));
      printf("Writing '%s'...\n", img_prefix.c_str());
      if (!vision_utils::write_rgb_depth_user_to_image_file
          (img_prefix, &reader.get_bgr(), &reader.get_depth(), &reader.get_user(),
           &user_illus, vision_utils::FILE_PNG, false))
        return false;
      if (write_test)
        nfiles_test_written++;
      else
        nfiles_train_written++;
      ++nfiles_total_written;
      // check if we need to go to next file
      if (nfiles_test_written >= vision_utils::DGaitDBFilename::NFILES_TEST
          && nfiles_train_written >= vision_utils::DGaitDBFilename::NFILES_TRAIN)
        break;
    } // end while(reader.go_to_next_frame())
  } // end loop file_idx
  return 0;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  return video_reader(argc, argv);
  //return generate_training_data();
}
