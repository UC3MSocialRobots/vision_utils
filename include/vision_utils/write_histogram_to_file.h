/*!
  \file        write_histogram_to_file.h
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

#ifndef WRITE_HISTOGRAM_TO_FILE_H
#define WRITE_HISTOGRAM_TO_FILE_H
// std includes
#include <opencv2/core/core.hpp>
#include <string>

namespace vision_utils {

inline void write_histogram_to_file(const Histogram & hist,
                                    const std::string & output_filename) {
  cv::FileStorage fs(output_filename, cv::FileStorage::WRITE);
  fs << "hist" << hist;
  fs.release();
}

} // end namespace vision_utils

#endif // WRITE_HISTOGRAM_TO_FILE_H
