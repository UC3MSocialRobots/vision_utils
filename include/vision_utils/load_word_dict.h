/*!
  \file        load_word_dict.h
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

#ifndef LOAD_WORD_DICT_H
#define LOAD_WORD_DICT_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
// vision_utils
#include <vision_utils/retrieve_file_split.h>

namespace vision_utils {

typedef std::set<std::string> WordDict;

bool load_word_dict(const std::string & dict_filename, WordDict & dict) {
  printf("load_word_dict('%s')\n", dict_filename.c_str());

  // read the file as a vector of strings
  std::vector<std::string> dict_vector;
  if (!vision_utils::retrieve_file_split(dict_filename, dict_vector))
    return false;
  // push all the words in the set
  for (std::vector<std::string>::const_iterator word = dict_vector.begin();
       word != dict_vector.end() ; ++word)
    dict.insert(*word);
  return true;
}

} // end namespace vision_utils

#endif // LOAD_WORD_DICT_H
