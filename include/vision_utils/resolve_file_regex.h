/*!
  \file        resolve_file_regex.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/12
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

#ifndef RESOLVE_FILE_REGEX_H
#define RESOLVE_FILE_REGEX_H
#include <sstream>
#include <set>
#include "vision_utils/string_split.h"
#include "vision_utils/file_exists.h"

namespace vision_utils {

typedef std::set<std::string> StringSet;

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
  StringSplit(files_regex, ";", &regex_words);
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
    std::string ls = exec_system_get_output(cmd.str().c_str());
    std::vector<std::string> ls_files;
    StringSplit(ls, "\n", &ls_files);
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
    if (!file_exists(*files_it)) {
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

} // end namespace vision_utils

#endif // RESOLVE_FILE_REGEX_H
