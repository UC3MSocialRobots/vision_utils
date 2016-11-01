/*!
  \file        _curr_prefix2imgs.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/11

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
#ifndef _curr_prefix2IMGS_H
#define _curr_prefix2IMGS_H

#include "vision_utils/database_player.h"
#include "vision_utils/io.h"
#include "vision_utils/find_and_replace.h"

namespace vision_utils {

class FilenamePrefix2Imgs : public DatabasePlayer {
public:
  typedef std::map<std::string, std::vector<cv::Rect> > AnnotMap;
  FilenamePrefix2Imgs() {}

  //////////////////////////////////////////////////////////////////////////////

  //! load the images into _bgr, _depth32f, _user8
  virtual bool go_to_next_frame() {
    return go_to_next_file();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! parse a possible xml file, etc.
  virtual bool load_single_video(const std::string & filename) {
    _curr_prefix = filename;
    find_and_replace(_curr_prefix, "_depth.png", "");
    find_and_replace(_curr_prefix, "_depth_params.yaml", "");
    find_and_replace(_curr_prefix, "_rgb.png", "");
    find_and_replace(_curr_prefix, "_user_mask.png", "");
    if (!read_rgb_depth_user_image_from_image_file
        (_curr_prefix, &_bgr, &_depth32f, &_user8))
      return false;
    _has_rgb = _has_depth = _has_user = true;
    return true;
  } // end load_single_video()

  //////////////////////////////////////////////////////////////////////////////

  //! \return the user idx, useful for labelling and machine learning
  virtual inline UserIdx get_user_idx() {
    if (_curr_prefix.find("juggling") != std::string::npos)
      return 0;
    if (_curr_prefix.find("alberto") != std::string::npos)
      return 1;
    if (_curr_prefix.find("alvaro") != std::string::npos)
      return 2;
    if (_curr_prefix.find("david_arnaud") != std::string::npos)
      return 3;
    if (_curr_prefix.find("breast") != std::string::npos)
      return 4;
    // another file -> unique ID
    return 5 + get_playlist_idx();
  }

private:
  std::string _curr_prefix;
}; // end class FilenamePrefix2Imgs

} // end namespace vision_utils

#endif // _curr_prefix2IMGS_H
