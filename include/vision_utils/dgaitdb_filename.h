/*!
  \file        dgaitdb_filename.h
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

The filenames for the files generated with database available in
\link http://www.cvc.uab.es/DGaitDB/Summary.html .
 */

#ifndef DGAITDB_FILENAME_H
#define DGAITDB_FILENAME_H

#include "vision_utils/directory_exists.h"

namespace vision_utils {

class DGaitDBFilename {
public:
  static const unsigned int ONI_FILES = 55, NFILES_TRAIN = 20, NFILES_TEST = 10;
  static const uchar USER_IDX = 1;
  //! ctor
  DGaitDBFilename(const std::string & input_directory) : _input_directory(input_directory) {
    if (!directory_exists())
      printf("DGaitDBFilename: directory '%s' does not exist!\n", _input_directory.c_str());
  }

  //! \return true if the image directory exists
  bool directory_exists() const { return vision_utils::directory_exists(_input_directory); }

  //////////////////////////////////////////////////////////////////////////////

  //! \arg oni_idx in [1, ONI_FILES], test_idx in [1, NFILES_TEST]
  inline std::string filename_test(const unsigned int oni_idx,
                                   const unsigned int test_idx) const {
    std::ostringstream filename;
    filename << _input_directory << oni_idx << "test" << test_idx;
    return filename.str();
  }
  //! \arg oni_idx in [1, ONI_FILES], train_idx in [1, NFILES_TRAIN]
  inline std::string filename_train(const unsigned int oni_idx,
                                    const unsigned int train_idx) const {
    std::ostringstream filename;
    filename << _input_directory << oni_idx << "train" << train_idx;
    return filename.str();
  }

  //! \arg oni_idx in [1, ONI_FILES]
  inline bool is_man(const unsigned int oni_idx) {
    static const bool _isman[ONI_FILES] = {1,0,0,1,0,1,0,0,0,1,  0,0,1,1,0,1,1,1,1,0,
                                           0,1,1,0,1,1,1,1,1,1,  1,0,1,1,0,0,0,0,1,1,
                                           1,0,1,1,0,1,1,1,1,1,  1,1,1,1,1};
    return (oni_idx >= 1 && oni_idx <= ONI_FILES ? _isman[oni_idx] : false);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline std::vector<std::string> all_filenames_test() {
    std::vector<std::string> ans;
    ans.reserve(ONI_FILES * NFILES_TEST);
    for (unsigned int oni_idx = 1; oni_idx <= ONI_FILES; ++oni_idx)
      for (unsigned int test_idx = 1; test_idx <= NFILES_TEST; ++test_idx)
        ans.push_back(filename_test(oni_idx, test_idx));
    return ans;
  }
  inline std::vector<std::string> all_filenames_train() {
    std::vector<std::string> ans;
    ans.reserve(ONI_FILES * NFILES_TRAIN);
    for (unsigned int oni_idx = 1; oni_idx <= ONI_FILES; ++oni_idx)
      for (unsigned int train_idx = 1; train_idx <= NFILES_TRAIN; ++train_idx)
        ans.push_back(filename_train(oni_idx, train_idx));
    return ans;
  }

  //////////////////////////////////////////////////////////////////////////////

  template<class T>
  inline std::vector<T> all_genders_test(T man_value, T woman_value) {
    std::vector<T> ans;
    ans.reserve(ONI_FILES * NFILES_TEST);
    for (unsigned int oni_idx = 1; oni_idx <= ONI_FILES; ++oni_idx)
        ans.insert(ans.end(), NFILES_TEST, is_man(oni_idx) ? man_value:woman_value);
    return ans;
  }
  template<class T>
  inline std::vector<T> all_genders_train(T man_value, T woman_value) {
    std::vector<T> ans;
    ans.reserve(ONI_FILES * NFILES_TRAIN);
    for (unsigned int oni_idx = 1; oni_idx <= ONI_FILES; ++oni_idx)
        ans.insert(ans.end(), NFILES_TRAIN, is_man(oni_idx) ? man_value:woman_value);
    return ans;
  }

  std::string _input_directory;
}; // end class DGaitDBFilename

} // end namespace vision_utils

#endif // DGAITDB_FILENAME_H
