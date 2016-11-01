/*!
  \file
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/4/14

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

\todo Description of the file
 */
#ifndef CUT_STRING_INTO_CHUNKS_H
#define CUT_STRING_INTO_CHUNKS_H

#include <string>
#include <vector>

namespace vision_utils {
static inline bool cut_with_string_at_maxpos(const std::string & text,
                                             const std::string & delimiter,
                                             const std::string::size_type & max_size,
                                             std::vector<std::string> & chunks) {
    chunks.clear();
    // if string short enough, do nothing
    if (text.size() <= max_size) {
        chunks.push_back(text);
        return true;
    }

    // first search near the half
    std::string::size_type tag_pos = text.rfind(delimiter,
                                                max_size);

    // if there is no tag, fail miserably
    if (tag_pos == std::string::npos)
        return false;

    // push both halves
    chunks.push_back(text.substr(0, tag_pos));
    if (tag_pos + delimiter.size() < text.size())
        chunks.push_back(text.substr(tag_pos + delimiter.size()));
    return true;
}

//////////////////////////////////////////////////////////////////////////////

//! try to cut the text into chunks by dichotomy
static bool cut_string_into_chunks(const std::string & text,
                                    const std::string::size_type & max_size,
                                    std::vector<std::string> & chunks) {
    std::string text_left = text;
    std::vector<std::string> minichunks;
    while(true) {
        minichunks.clear();
        bool success = cut_with_string_at_maxpos(text_left, ". ", max_size, minichunks);
        if (!success)
            success = cut_with_string_at_maxpos(text_left, ", ", max_size, minichunks);
        if (!success)
            success = cut_with_string_at_maxpos(text_left, " ", max_size, minichunks);
        if (!success) {
            printf("Could not cut the chunk '%s'\n", text_left.c_str());
            return false;
        }
        // add the first chunk
        chunks.push_back(minichunks.front());
        if (minichunks.size() == 1) // only one word -> we finished!
            return true;
        text_left = minichunks.back();
    } // end while true
} // end cut_string_into_chunks
} // end namespace vision_utils

#endif // CUT_STRING_INTO_CHUNKS_H

