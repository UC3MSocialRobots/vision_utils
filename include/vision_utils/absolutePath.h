/*!
  \file        absolutePath.h
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

#ifndef ABSOLUTEPATH_H
#define ABSOLUTEPATH_H
#include <sstream>
#include <string>
#include <vector>

namespace vision_utils {

/*! \brief   returns the absolute path from a relative path.
 * For instance, absolutePath("../3", "C:/1/2/") = "C:/1/3"
 *
 * \param   filename the relative filename, after prefix
 * \param   prefix the beginning of the path
 * \return  the absolute path
 */
std::string absolutePath(const char* filename,
                         const std::string prefix) {
  std::string abs_path = prefix + filename;
  std::vector<std::string> folders;
  StringSplit(abs_path, "/", &folders);

  /* eliminate the ".." from the vector 'folders' */
  std::vector<std::string> foldersLeft;
  for (unsigned int i = 0; i < folders.size(); i++) {
      //cout << "current word:" << folders.at(i) << endl;
      if (folders.at(i) == "..")
        foldersLeft.pop_back();
      else
        foldersLeft.push_back(folders.at(i));
    }

  /* copy the result in a std::string */
  std::ostringstream rep;
  for (unsigned int i = 0; i < foldersLeft.size(); i++)
    rep << "/" << foldersLeft.at(i);
  //cout << "abs path:" << abs_path << "\t path:" << rep.str() << endl;
  //printf("absolutePath('%s', '%s') = '%s'", filename, prefix.c_str(), rep.str().c_str());

  return (char*) rep.str().c_str();
}
//inline std::string absolutePath(const char* filename);

} // end namespace vision_utils

#endif // ABSOLUTEPATH_H
