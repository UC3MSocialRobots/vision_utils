/*!
  \file        to_yaml.h
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

#ifndef TO_YAML_H
#define TO_YAML_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

template<class _T>
inline void write(const std::vector<_T> & phs,
                  cv::FileStorage &fs,
                  const std::string & key) {
  fs << key << "[";
  for( unsigned int i = 0; i < phs.size(); i++ ) {
    printf("write key '%s' #%i\n", key.c_str(), i);
    fs << "{";
    //fs << phs[i];
    phs[i].write(fs);
    fs << "}";
  }
  fs << "]";
} // end write()

/*!
 * Save file to YAML / XML
 * \param yaml_filename_prefix
 */
template<class _T>
inline void to_yaml(_T & obj,
                    const std::string & yaml_filename_prefix,
                    const std::string & key) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_" << key << ".yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::WRITE);
  obj.write(fs);
  fs.release();
  printf("Succesfully written '%s' to file '%s'\n",
         key.c_str(), full_filename.str().c_str());
} // end to_yaml();

} // end namespace vision_utils

#endif // TO_YAML_H
