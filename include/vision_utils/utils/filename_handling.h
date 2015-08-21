/*!
  \file        filename_handling.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/2

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

Some useful functions for playing with filenames:
changing extensions, extracting folder, etc.

 */

#ifndef FILENAME_HANDLING_H
#define FILENAME_HANDLING_H

#include <set>
#include <iomanip>
#include "vision_utils/utils/system_utils.h"
#include "vision_utils/utils/string_split.h"

namespace StringUtils {
typedef std::set<std::string> StringSet;

/*!
 \param path
 \return std::string
 \example path="/tmp/foo/bar.dat", returns "bar.dat"
 \example path="/bar.dat", returns "bar.dat"
 \example path="bar.dat", returns "bar.dat"
*/
inline std::string extract_filename_from_full_path(const std::string & path) {
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos == std::string::npos)
    return path;
  return path.substr(slash_pos + 1);
} // end extract_filename_from_full_path()

////////////////////////////////////////////////////////////////////////////////

/*!
 \param path
 \return std::string
 \example path="/tmp/foo/bar.dat", returns "/tmp/foo/"
 \example path="foo/bar.dat", returns "foo/"
 \example path="/bar.dat", returns "/"
 \example path="bar.dat", returns ""
*/
inline std::string extract_folder_from_full_path(const std::string & path) {
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos == std::string::npos)
    return "";
  return path.substr(0, 1 + slash_pos);
} // end extract_folder_from_full_path()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Retrieve the extension from a filename
 * \param path
 *  the full path
 * \example
 *  "/foo/bar" -> ""
 *  "/foo/bar.dat" -> ".dat"
 *  "/foo.zim/bar.dat" -> ".dat"
 *  "/foo.zim/bar" -> ""
 */
inline std::string get_filename_extension
(const std::string & path) {
  std::string::size_type dot_pos = path.find_last_of('.');
  if (dot_pos == std::string::npos)
    return "";
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos != std::string::npos && slash_pos > dot_pos) // dot before slash
    return "";
  return path.substr(dot_pos);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Remove the extension from a filename
 * \param path
 *  the full path
 * \example
 *  "/foo/bar" -> "/foo/bar"
 *  "/foo/bar.dat" -> "/foo/bar"
 *  "/foo.zim/bar.dat" -> "/foo.zim/bar"
 *  "/foo.zim/bar" -> "/foo.zim/bar"
 */
inline std::string remove_filename_extension
(const std::string & path) {
  std::string::size_type dot_pos = path.find_last_of('.');
  if (dot_pos == std::string::npos)
    return path;
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos != std::string::npos && slash_pos > dot_pos) // dot before slash
    return path;
  return path.substr(0, dot_pos);
}

////////////////////////////////////////////////////////////////////////////////

inline std::string change_filename_extension
(const std::string & path,
 const std::string & new_extension) {
  return remove_filename_extension(path) + new_extension;
}

////////////////////////////////////////////////////////////////////////////////

/*!
  Add a suffix to a filename
 \param path
    The absolute or relative path to filename
 \param suffix
    The string to be added to path, before the file extension
 \return std::string
 \example ("/foo/bar.dat", "_out") returns "/foo/bar_out.dat"
          ("/foo/bar", "_out") returns "/foo/bar_out"
*/
inline std::string add_suffix_before_filename_extension
(const std::string & path, const std::string & suffix = "out") {
  std::string::size_type dot_pos = path.find_last_of('.');
  std::ostringstream out;
  if (dot_pos == std::string::npos) {
    out << path << suffix;
  }
  else {
    std::string path_before_dot = path.substr(0, dot_pos);
    std::string path_after_dot = path.substr(dot_pos + 1);
    out << path_before_dot << suffix << "." << path_after_dot;
  }
  return out.str();
} // end add_suffix_before_filename_extension()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Transform a regular expression into a list of files
 * \param files semicolor-separated list
 * \exampel
 * \return
 */
inline bool resolve_file_regex(const std::string & files_regex,
                               std::vector<std::string> & files,
                               bool check_file_exists = true) {
  // split with ';'
  std::vector<std::string> regex_words;
  StringUtils::StringSplit(files_regex, ";", &regex_words);
  // convert it into a set for removing repetiions
  StringSet files_set;
  files_set.insert(regex_words.begin(), regex_words.end());

  // resolve the wildcards
  StringSet::iterator files_it = files_set.begin();
  // cannot modify files_set direcly (iterators not valid)
  StringSet to_insert, to_erase;
  while (files_it != files_set.end()) {
    if (files_it->find('*') == std::string::npos) {
      ++files_it;
      continue;
    }
    std::string regex = *files_it;
    to_erase.insert(regex);
    std::ostringstream cmd; cmd << "ls -1 " << regex;// << " 2> /dev/null"; // quiet
    std::string ls = system_utils::exec_system_get_output(cmd.str().c_str());
    std::vector<std::string> ls_files;
    StringUtils::StringSplit(ls, "\n", &ls_files);
    to_insert.insert(ls_files.begin(), ls_files.end());
    ++files_it;
  } // end while (files_it)
  files_set.insert(to_insert.begin(), to_insert.end());
  // remove empty strings from files_set and what needs to be removed
  files_set.erase(" ");
  files_set.erase("");
  for(StringSet::const_iterator it = to_erase.begin(); it != to_erase.end(); ++it)
    files_set.erase(*it);

  files_it = files_set.begin();
  // check the files existence
  while (check_file_exists && files_it != files_set.end()) {
    if (!system_utils::file_exists(*files_it)) {
      printf("resolve_file_regex(): file '%s' does not exist!\n", files_it->c_str());
      files_set.erase(*files_it);
    }
    ++files_it;
  } // end while (files_it)

  // convert files_set -> answer vector
  files.clear();
  files.reserve(files_set.size());
  std::copy(files_set.begin(), files_set.end(), std::back_inserter(files));
  return true;
} // end resolve_file_regex()

} // end namespace StringUtils

#endif // FILENAME_HANDLING_H
