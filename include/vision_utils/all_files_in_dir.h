/*!
  \file        all_files_in_dir.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/2
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

#ifndef ALL_FILES_IN_DIR_H
#define ALL_FILES_IN_DIR_H
// std includes
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
#include <dirent.h>

namespace vision_utils {

/*!
 * Return the filename of all files in a given folder
 * \param folder
 *    the relative or absolute folder
 * \param ans
 *    the vector that will contain all answers
 * \param pattern
 *    the pattern of files to keep, for instance ".kpg"
 * \param add_folder_to_filename
 *    if true, \a folder will be added to each filename, f.ex. "/tmp/foo.dat",
 *    otherwise, just the raw filename is kept, f.ex. "foo.dat"
 * \return
 *    the number of files found.
 */
inline int all_files_in_dir(const std::string & folder,
                            std::vector<std::string> & ans,
                            const std::string pattern = "",
                            bool add_folder_to_filename = true) {
  ans.clear();
  std::string folder_and_slash = folder + std::string("/");

#if 1
  DIR *dir = opendir (folder.c_str());
  struct dirent *ent;
  if (dir == NULL) {
    printf("all_files_in_dir:could not open directory:'%s'\n", folder.c_str());
    return -1;
  }

  /* print all the files and directories within directory */
  while ((ent = readdir (dir)) != NULL) {
    //printf ("%s\n", ent->d_name);
    std::string filename(ent->d_name);
    if (pattern.size() > 0 && filename.find(pattern) == std::string::npos)
      continue;
    if (add_folder_to_filename)
      ans.push_back(folder_and_slash + filename);
    else
      ans.push_back(filename);
  }
  closedir (dir);
#elif 0
  std::ostringstream order;
  order << "ls -1 " << folder_and_slash << "*" << pattern << "*";
  std::string ls_res = exec_system_get_output(order.str().c_str());
  // string split
  ans.clear();
  if (ls_res == "")
    return 0;
  size_t delim_pos, search_pos = 0;
  while (search_pos <= ls_res.size() - 1) {
    delim_pos = ls_res.find("\n", search_pos);
    //printf("delim_pos:%i, search_pos:%i", delim_pos, search_pos);
    if (delim_pos == std::string::npos) { // no more delim
      ans.push_back(ls_res.substr(search_pos));
      break;
    }
    if (delim_pos > 0) // == 0 only happens if str starts with delim
      ans.push_back(ls_res.substr(search_pos, delim_pos - search_pos));
    search_pos = delim_pos + 1; // size of delim
    // quit if we reached the end of the std::string or std::string empty
  }
#else // https://stackoverflow.com/questions/4283546/cboostfilesystem-how-can-i-get-a-list-of-files-in-a-folder-in-which-the-fi
  boost::filesystem::path someDir(folder);
  boost::filesystem::directory_iterator end_iter;

  typedef std::multimap<std::time_t, boost::filesystem::path> result_set_t;
  result_set_t result_set;

  if ( boost::filesystem::exists(someDir) && boost::filesystem::is_directory(someDir)) {
    for( boost::filesystem::directory_iterator dir_iter(someDir) ; dir_iter != end_iter ; ++dir_iter) {
      if (boost::filesystem::is_regular_file(dir_iter->status()) ) {
        result_set.insert(result_set_t::value_type(boost::filesystem::last_write_time(dir_iter->status()), *dir_iter);
      }
    }
  }
#endif
  return ans.size();
} // end all_files_in_dir()

} // end namespace vision_utils

#endif // ALL_FILES_IN_DIR_H
