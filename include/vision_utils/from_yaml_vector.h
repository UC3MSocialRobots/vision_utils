/*!
  file
  author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  date        2016/11/3
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

  odo Description of the file
 */

#ifndef FROM_YAML_VECTOR_H
#define FROM_YAML_VECTOR_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
#include <vision_utils/from_yaml.h>

namespace vision_utils {

// https://github.com/UC3MSocialRobots/vision_utils/blob/af1ca8abf21a49ef53e38a4d96bc8eb797f88fdc/include/vision_utils/io.h
template<class _T>
inline void read(std::vector<_T> & phs,
                 const cv::FileNode &fn,
                 const std::string & key) {
  cv::FileNode hist_nodes = fn[key];
  phs.resize(hist_nodes.size());
  for (unsigned int hist_idx = 0; hist_idx < hist_nodes.size(); ++hist_idx)
    hist_nodes[hist_idx] >> phs[hist_idx];
} // end write

////////////////////////////////////////////////////////////////////////////////

/*!
 * Load from YAML / XML file
 * \param yaml_filename_prefix
 */
template<class _T>
inline void from_yaml_vector(std::vector<_T> & obj,
                             const std::string & yaml_filename_prefix,
                             const std::string & key) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_" << key << ".yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::READ);
  //read(obj, fs.getFirstTopLevelNode());
  fs[key] >> obj;
  fs.release();
  printf("Succesfully read vector<_T> from file '%s'\n",
         full_filename.str().c_str());
} // end from_yaml_vector();
} // end namespace vision_utils

#endif // FROM_YAML_VECTOR_H

