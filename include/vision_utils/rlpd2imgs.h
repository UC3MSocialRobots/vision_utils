/*!
  \file        rlpd2imgs.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/11/20

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
#ifndef RLPD2IMGS_H
#define RLPD2IMGS_H

#include "vision_utils/utils/find_and_replace.h"
#include "vision_utils/database_player.h"
#include "vision_utils/mask.h"
#include "vision_utils/opencv_safe_image_io.h"

static const unsigned int NCOLORS_NITE = 7;
static const cv::Scalar COLORS_NITE[NCOLORS_NITE] = {
  cv::Scalar(0, 0, 0), // black = eraser
  cv::Scalar(0, 255, 0),
  cv::Scalar(0, 0, 255),
  cv::Scalar(0, 255, 255),
  cv::Scalar(255, 0, 255),
  cv::Scalar(255, 255, 0),
  cv::Scalar(255, 160, 0)
};
static const unsigned int NCOLORS_GROUND_TRUTH = 4;
static const cv::Scalar COLORS_GROUND_TRUTH[NCOLORS_GROUND_TRUTH] = {
  cv::Scalar(0, 0, 0), // black = eraser
  cv::Scalar(255, 0, 0),
  cv::Scalar(0, 255, 0),
  cv::Scalar(0, 0, 255)
};

class RLPD2Imgs : public DatabasePlayer {
public:
  RLPD2Imgs() {
    _has_rgb = _has_depth = _has_user = true;
  }

  //! load the images into _bgr, _depth32f, _user8
  virtual bool go_to_next_frame() {
    return go_to_next_file();
  }

  //! parse a possible xml file, etc.
  virtual bool load_single_video(const std::string & filename) {
    // read rgb and depth
    std::string filename_prefix(filename);
    StringUtils::find_and_replace(filename_prefix, "_depth.png", "");
    StringUtils::find_and_replace(filename_prefix, "_rgb.png", "");
    StringUtils::find_and_replace(filename_prefix, "_rgb.jpg", "");
    if (!image_utils::read_rgb_and_depth_image_from_image_file
        (filename_prefix, &_bgr, &_depth32f))
      return false;
    // read NiTE user
    std::ostringstream filename_user;
    filename_user << filename_prefix << "_user_mask_illus.png";
    if (!image_utils::imread_safe(_user_color, filename_user.str(), CV_LOAD_IMAGE_COLOR)
        || !image_utils::color_mask_ncolors(_user_color, NCOLORS_NITE, COLORS_NITE, _user8)) {
      printf("Problem with user image '%s'\n", filename_user.str().c_str());
      return false;
    }
    // read NiTE user
    filename_user.str("");
    filename_user << filename_prefix << "_ground_truth_user.png";
    if (!image_utils::imread_safe(_user_color_ground_truth, filename_user.str(), CV_LOAD_IMAGE_COLOR)
        || !image_utils::color_mask_ncolors
        (_user_color_ground_truth, NCOLORS_GROUND_TRUTH, COLORS_GROUND_TRUTH, _user8_ground_truth)) {
      printf("Problem with ground truth user image '%s'\n", filename_user.str().c_str());
      return false;
    }
    return true;
  }


  inline const cv::Mat1b & get_ground_truth_user() const {
    return _user8_ground_truth;
  }

  virtual char display() {
    user_image_to_rgb(_user8_ground_truth, _user8_ground_truth_illus, 8);
    cv::imshow("user_ground_truth", _user8_ground_truth_illus);
    return DatabasePlayer::display();
  }

private:
  cv::Mat3b _user_color, _user_color_ground_truth, _user8_ground_truth_illus;
  cv::Mat1b _user8_ground_truth;
};

#endif // RLPD2IMGS_H
