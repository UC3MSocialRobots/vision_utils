/*!
  \file        read_vec.h
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

#ifndef READ_VEC_H
#define READ_VEC_H
// std includes
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>

namespace vision_utils {

template<class _T>
inline void read(std::vector<_T> & phs,
                 const cv::FileNode &fn,
                 const std::string & key) {
  cv::FileNode hist_nodes = fn[key];
  phs.resize(hist_nodes.size());
  for (unsigned int hist_idx = 0; hist_idx < hist_nodes.size(); ++hist_idx)
    hist_nodes[hist_idx] >> phs[hist_idx];
} // end write

} // end namespace vision_utils

#endif // READ_VEC_H
