/*!
  \file        from_yaml.h
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

#ifndef FROM_YAML_H
#define FROM_YAML_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {
/*!
 * It’s possible to serialize this through the OpenCV I/O XML/YAML interface
 *(just as in case of the OpenCV data structures) by adding a read and
 * a write function inside and outside of your class. Cf:
 * http://docs.opencv.org/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html
 * \param node
 * \param x
 * \param default_value
 *
 */
template<class _T>
void read(const cv::FileNode & node, _T & x, const _T& default_value = _T()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
} //end read()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Load from YAML / XML file
 * \param yaml_filename_prefix
 */
template<class _T>
inline void from_yaml(_T & obj,
                      const std::string & yaml_filename_prefix,
                      const std::string & key) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_" << key << ".yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::READ);
  read(fs.root(), obj, _T());
  fs.release();
  printf("Succesfully read '%s' from file '%s'\n",
         key.c_str(), full_filename.str().c_str());
} // end from_yaml();

} // end namespace vision_utils

#endif // FROM_YAML_H
