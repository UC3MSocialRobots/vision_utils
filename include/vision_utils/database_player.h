/*!
  \file        database_player.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/7

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

A template class for iteratively access RGB+depth+user databases.
 */
#ifndef DATABASE_PLAYER_H
#define DATABASE_PLAYER_H

#include <opencv2/highgui/highgui.hpp>
// AD
#include "vision_utils/depth_image_to_vizualisation_color_image.h"
#include "vision_utils/imwrite_debug.h"
#include "vision_utils/iterable_to_string.h"
#include "vision_utils/resolve_file_regex.h"
#include "vision_utils/timestamp.h"
#include "vision_utils/user_image_to_rgb.h"

namespace vision_utils {

class DatabasePlayer {
public:
  typedef unsigned int UserIdx;

  DatabasePlayer() {}

  //////////////////////////////////////////////////////////////////////////////

  //! load the images into _bgr, _depth32f, _user8
  virtual bool go_to_next_frame() = 0;

  //////////////////////////////////////////////////////////////////////////////

  bool from_file(const std::string & filename_regex,
                 bool want_repeat_playlist_ = true) {
    printf("DatabasePlayer::from_file('%s')\n", filename_regex.c_str());
    resolve_file_regex(filename_regex, _playlist, true);
    if (_playlist.size() == 0) {
      printf("DatabasePlayer: regex '%s' corresponds to no existing file\n",
             filename_regex.c_str());
      return false;
    }
    printf("DatabasePlayer: playlist %s\n",
           iterable_to_string(_playlist).c_str());
    set_repeat_playlist(want_repeat_playlist_);
    _playlist_idx = 0;
    return load_single_video(_playlist[_playlist_idx]);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \return false if playlist over
  bool go_to_next_file() {
    if (!_want_repeat_playlist && _playlist_idx == _playlist.size()-1) // playlist over
      return false;
    _playlist_idx = (_playlist_idx+1) % _playlist.size();
    return load_single_video(_playlist[_playlist_idx]);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \return the user idx, useful for labelling and machine learning
  virtual inline UserIdx get_user_idx() {
    return get_playlist_idx();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_repeat_playlist(bool repeat) { _want_repeat_playlist = repeat; }
  inline unsigned int get_playlist_idx() const { return _playlist_idx; }
  inline unsigned int get_playlist_size() const { return _playlist.size(); }

  //////////////////////////////////////////////////////////////////////////////

  inline bool has_rgb() const { return _has_rgb; }
  inline bool has_depth() const { return _has_depth; }
  inline bool has_user() const { return _has_user; }
  inline const cv::Mat3b & get_bgr() const { return _bgr; }
  inline const cv::Mat1f & get_depth() const { return _depth32f; }
  inline const cv::Mat1b & get_user() const { return _user8; }

  //////////////////////////////////////////////////////////////////////////////

  virtual char display() {
    cv::namedWindow("bgr");
    cv::namedWindow("depth");
    cv::namedWindow("user");
    if (_has_rgb)
      cv::imshow("bgr", _bgr);
    if (_has_depth) {
      depth_image_to_vizualisation_color_image
          //(_depth32f, depth_illus, FULL_RGB_STRETCHED);
          //(_depth32f, depth_illus, FULL_RGB_SCALED, 2.5, 4);
          (_depth32f, depth_illus, FULL_RGB_SCALED, 1, 6);
      cv::imshow("depth", depth_illus);
    }
    if (_has_user) {
      user_image_to_rgb(_user8, user_illus, 8);
      cv::imshow("user", user_illus);
    }
    char c = cv::waitKey(5);
    if (c == 's') { // save images
      std::string timestamp = vision_utils::timestamp();
      std::ostringstream filename;
      filename << "rgb_" << timestamp << ".png";
      imwrite_debug(filename.str(), _bgr, COLOR_24BITS);
      filename.str(""); filename << "depth_" << timestamp << ".png";
      imwrite_debug(filename.str(), depth_illus, COLORS256);
      filename.str(""); filename << "user_" << timestamp << ".png";
      imwrite_debug(filename.str(), user_illus, COLORS256);
    }
    return c;
  } // end display();

  ////////////////////////////////////////////////////////////////////////////////

  //! a static function to test the DatabasePlayer with command line args
  static int static_player(DatabasePlayer & player, int argc, char **argv) {
    if (argc < 1) {
      printf("Synopsis: %s [onifile]\n", argv[0]);
      return -1;
    }
    std::ostringstream files;
    for (int argi = 1; argi < argc; ++argi) // 0 is the name of the exe
      files << argv[argi]<< ";";
    if (!player.from_file(files.str()))
      return -1;
    while (player.go_to_next_frame()) {
      char c = player.display();
      if (c == 'n' && !player.go_to_next_file())
        return 0;
    }
    return 0;
  } // end player()

protected:
  //! parse a possible xml file, etc.
  virtual bool load_single_video(const std::string & filename) = 0;

  //////////////////////////////////////////////////////////////////////////////

  bool _has_rgb;
  cv::Mat3b _bgr;
  bool _has_depth;
  cv::Mat1f _depth32f;
  cv::Mat3b depth_illus;
  bool _has_user;
  cv::Mat1b _user8;
  cv::Mat3b user_illus;

  std::vector<std::string> _playlist;
  bool _want_repeat_playlist;
  unsigned int _playlist_idx;
}; // end class DatabasePlayer

} // end namespace vision_utils

#endif // DATABASE_PLAYER_H
