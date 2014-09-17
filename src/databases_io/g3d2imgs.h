/*!
  \file        g3d2imgs.h
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

\todo Description of the file
http://dipersec.king.ac.uk/G3D/
 */
#ifndef G3D2IMGS_H
#define G3D2IMGS_H

#include <databases_io/database_player.h>
#include <xml/XmlDocument.h>

class G3D2Imgs : public DatabasePlayer {
public:
  typedef unsigned short DepthType;
  G3D2Imgs() {}

  //////////////////////////////////////////////////////////////////////////////

  //! load the images into _bgr, _depth32f, _user8
  virtual bool go_to_next_frame() {
    ++_files_idx;
    if (_files_idx >= _files.size())
      return go_to_next_file();
    return load_current_frame();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! parse a possible xml file, etc.
  virtual bool load_single_video(const std::string & filename) {
    _files.clear();
    std::string folder = StringUtils::extract_folder_from_full_path(filename);
    StringUtils::find_and_replace(folder, "Colour", "");
#if 0
    // get all files in "Colour"
    std::vector<std::string> colour_files;
    if (!system_utils::all_files_in_dir(folder + "/Colour", colour_files, ".png", true)) {
      printf("G3D2Imgs: no file found in '%s'\n", _imgs_folder.c_str());
      return false;
    }

    // for each colour_file
    for (unsigned int colour_idx = 0; colour_idx < colour_files.size(); ++colour_idx) {
      std::string bgr_file = colour_files[colour_idx];
      std::string depth_file = bgr_file;
      StringUtils::find_and_replace(depth_file, "Colour", "Depth");
      if (!system_utils::file_exists(depth_file)) {
        printf("Non existing depth_file:'%s'\n", depth_file.c_str());
        continue;
      }
      _files.push_back(std::make_pair(bgr_file, depth_file));
    } // end loop colour_idx
    // check if the depth_file exists
    if (_files.size() == 0) {
      printf("G3D2Imgs: no file found in '%s'\n", _imgs_folder.c_str());
      return false;
    }
#else
    // get index of last files in "Colour"
    std::ostringstream order;
    order << "ls -1 " << folder << "Colour/*.png"
          << " | grep --only-matching \"[0-9]*.png\""
          << " | grep -o \"[0-9]*\" "
          << " | awk 'BEGIN {max = 0} {if ($0>max) max=$0} END {print max}'";
    printf("order:'%s'\n", order.str().c_str());
    std::string last_colour_idx_str =
        system_utils::exec_system_get_output(order.str().c_str());
    unsigned int last_colour_idx = StringUtils::cast_from_string<int>(last_colour_idx_str);
    printf("last_colour_idx:%i\n", last_colour_idx);
    for (unsigned int colour_idx = 0; colour_idx <=last_colour_idx; ++colour_idx) {
      std::ostringstream bgr_file;
      bgr_file << folder << "Colour/Colour " << colour_idx << ".png";
      if (!system_utils::file_exists(bgr_file.str()))
        continue;
      std::ostringstream depth_file;
      depth_file << folder << "Depth/Depth " << colour_idx << ".png";
      if (!system_utils::file_exists(depth_file.str())) {
        // printf("Non existing depth_file:'%s'\n", depth_file.str().c_str());
        continue;
      }
      _files.push_back(std::make_pair(bgr_file.str(), depth_file.str()));
    } // end loop colour_idx
#endif

    // set frame index to 0
    _files_idx = 0;
    return load_current_frame();
  } // end load_single_video()

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int n_frames_in_curr_video() const { return _files.size(); }

private:

  //////////////////////////////////////////////////////////////////////////////
  bool load_current_frame() {
    std::string bgr_filename = _files[_files_idx].first;
    std::string depth_filename = _files[_files_idx].second;
    printf("load_current_frame('%s', '%s')\n",
           bgr_filename.c_str(), depth_filename.c_str());

    _has_rgb = _has_depth = _has_user = true;
    _bgr = cv::imread(bgr_filename, CV_LOAD_IMAGE_COLOR);
    if (_bgr.empty()) {
      printf("G3D2Imgs: could not load '%s'\n", bgr_filename.c_str());
      return false;
    }
    // printf("_bgr:'%s'\n", image_utils::infosImage(_bgr).c_str());

    _depth16 = cv::imread(depth_filename, CV_LOAD_IMAGE_UNCHANGED);
    if (_depth16.empty()) {
      printf("G3D2Imgs: could not load '%s'\n", depth_filename.c_str());
      return false;
    }
    // printf("_depth16:'%s'\n", image_utils::infosImage(_depth16).c_str());

    // user img
    _user8.create(_depth16.size());
    _depth32f.create(_depth16.size());
    _user8 = 0;
    unsigned int nvalues = _depth16.cols * _depth16.rows;
    unsigned short* depth16_vals = (unsigned short*) _depth16.ptr();
    unsigned char* user8_vals = _user8.ptr();
    float* depth_vals = (float*) _depth32f.ptr();
    for (unsigned int value_idx = 0; value_idx < nvalues; ++value_idx) {
      //The raw depth information contains the depth of each pixel in millimetres
      //and was stored in 16-bit greyscale and the raw colour in 24-bit RGB.
      //The 16-bits of depth data contains 13 bits for depth data
      //and 3 bits to identify the player.
      //The player index can be used to segment the depth maps by user (see Figure 2).
      //unsigned user_idx = depth16_vals[value_idx] % 8;
      unsigned user_idx = depth16_vals[value_idx] & 7;
      if (user_idx > 0)
        user8_vals[value_idx] = user_idx;
      // the shifting of the three user bits of the depth map
      // must be done after building user8
      // also divide by 1000 for conv millimeters -> meters
      //float depth_m = depth16_vals[value_idx] * 0.00025; // 1 / 4000
      float depth_m = (depth16_vals[value_idx] >> 3) * 0.001; // 1 / 1000
      if (depth16_vals[value_idx] < 65500 && depth_m < 15) {
        //if (rand()%1000 == 0) printf("depth:%i -> %g\n", depth16_vals[value_idx], depth_m);
        depth_vals[value_idx] = depth_m;
      }
      else
        depth_vals[value_idx] = image_utils::NAN_DEPTH;
    } // end loop value_idx
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  std::string _imgs_folder;
  std::vector<std::pair<std::string, std::string> > _files;
  unsigned int _files_idx;

  //cv::Mat_<DepthType> _depth16;
  cv::Mat _depth16;
};
#endif // G3D2IMGS_H
