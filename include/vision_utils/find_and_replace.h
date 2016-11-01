/*!
  \file        find_and_replace.h
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

Find and replace a given string into another string.
 */

#ifndef FIND_AND_REPLACE_H
#define FIND_AND_REPLACE_H

#include <string>
#include <ros/io.h>

namespace vision_utils {
/*! find all the iterations of a pattern in a string and replace
 * them with another pattern
 * \param stringToReplace the string to change
 * \param pattern what we want to replace
 * \param patternReplacement what we replace with
 * \return the number of times we replaced pattern
 */
inline int find_and_replace(std::string& stringToReplace,
                            const std::string & pattern,
                            const std::string & patternReplacement) {
  size_t j = 0;
  int nb_found_times = 0;
  for (; (j = stringToReplace.find(pattern, j)) != std::string::npos;) {
    //cout << "found " << pattern << endl;
    stringToReplace.replace(j, pattern.length(), patternReplacement);
    j += patternReplacement.length();
    ++ nb_found_times;
  }
  return nb_found_times;
}

////////////////////////////////////////////////////////////////////////////////

/*! remove the spaces at the end of a word
 * \param content
 */
inline void remove_trailing_spaces(std::string & content) {
  while (content.length() > 0 && (content[content.length() - 1] == ' '
                                  || content[content.length() - 1] == '\n'
                                  || content[content.length() - 1] == '\t'))
    content = content.substr(0, content.length() - 1);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * remove the spaces at the beginning of a word
 * \param content
 */
inline void remove_beginning_spaces(std::string & content) {
  while (content.length() > 0 && (content[0] == ' ' || content[0] == '\n'
                                  || content[0] == '\t'))
    content = content.substr(1);
}

////////////////////////////////////////////////////////////////////////////////

/*! extract an info from some tags
 * \param content
 * \param block_begin
 * \param block_end
 * \param initial_search_pos
 * \return "" if not found
 */
std::string extract_from_tags(const std::string & content,
                              const std::string & block_begin,
                              const std::string & block_end,
                              int & initial_search_pos) {
  // find the beginning
  size_t pos_begin = content.find(block_begin, initial_search_pos);
  if (pos_begin == std::string::npos) {
      //printf("block_begin '%s' could not be found", block_begin.c_str());
      return "";
    }
  // remove the block at the beginning
  pos_begin += block_begin.length();
  // find the end
  size_t pos_end = content.find(block_end, pos_begin);
  if (pos_end == std::string::npos) {
      //printf("block_end '%s' could not be found", block_end.c_str());
      return "";
    }
  // update the index
  initial_search_pos = pos_end + block_end.length();
  // extract the substring
  //printf("value:'%s'", content.substr(pos_begin, pos_end - pos_begin).c_str());
  std::string ans = content.substr(pos_begin, pos_end - pos_begin);
  //cout << ans << endl;
  /*
     * clean the string
     */
  find_and_replace(ans, "&nbsp;", " ");
  remove_beginning_spaces(ans);
  remove_trailing_spaces(ans);
  return ans;
}

} // end namespace vision_utils
#endif // FIND_AND_REPLACE_H
