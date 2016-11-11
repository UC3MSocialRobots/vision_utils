/*!
  \file        translateToGibberishLanguage.h
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

#ifndef TRANSLATETOGIBBERISHLANGUAGE_H
#define TRANSLATETOGIBBERISHLANGUAGE_H
// std includes
#include <string>
#include <vector>
// vision_utils
#include <vision_utils/string_split.h>

namespace vision_utils {

/** Traduce una frase a Gibberish Language
      The general process you'll want to follow is:
      - split the string
      - loop through each word
      - for each word, move the first letter to the end, and all other letters up one position in the string.
      - put all the words back together as a single string.
      **/
std::string translateToGibberishLanguage(const std::string & stringSource){

  std::string manip = "",newstring = "";
  std::vector<std::string> words;

  vision_utils::StringSplit(stringSource, " ", &words);

  for(unsigned int i=0;i<words.size();i++)
  {
    if(i!=(words.size()-1))
    {
      manip = words[i].substr(1) + words[i].substr(0,1);
      words[i] = manip;
    }
    else
    {
      manip = words[i].substr(1,(words[i].length()-2)) + words[i].substr(0,1)+ words[i].substr((words[i].length()-1),1);
      words[i] = manip;
    }
  }
  for(unsigned int i=0;i<words.size();i++)
  {
    newstring += words[i];
    if(i!=(words.size()-1))
      newstring += " ";
  }

  return newstring;
}

} // end namespace vision_utils

#endif // TRANSLATETOGIBBERISHLANGUAGE_H
