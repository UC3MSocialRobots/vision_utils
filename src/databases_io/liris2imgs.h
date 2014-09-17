/*!
  \file        liris2imgs.h
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
http://liris.cnrs.fr/voir/activities-dataset/index.html
 */
#ifndef LIRIS2IMGS_H
#define LIRIS2IMGS_H

#include <vision_utils/databases_io/database_player.h>
#include <src/xml/XmlDocument.h>

class Liris2Imgs : public DatabasePlayer {
public:
  typedef std::map<std::string, std::vector<cv::Rect> > AnnotMap;
  Liris2Imgs() {}

  //////////////////////////////////////////////////////////////////////////////

  //! load the images into _bgr, _depth32f, _user8
  virtual bool go_to_next_frame() {
    ++_frame_it;
    if (_frame_it == _frames.end())
      return go_to_next_file();
    return load_current_frame();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! parse a possible xml file, etc.
  virtual bool load_single_video(const std::string & filename) {
    _frames.clear();
    printf("Oni2Imgs::load_single_video('%s')\n", filename.c_str());
    // find all files
    //_imgs_folder = StringUtils::extract_folder_from_full_path(filename) + vid_name + "/";
    _imgs_folder = StringUtils::remove_filename_extension(filename) + "/";
    std::vector<std::string> img_files;
    if (!system_utils::all_files_in_dir(_imgs_folder, img_files, ".jpg", true)) {
      printf("Liris2Imgs: no file found in '%s'\n", _imgs_folder.c_str());
      return false;
    }
    // convert vector to map
    for (unsigned int img_idx = 0; img_idx < img_files.size(); ++img_idx)
      _frames.insert(std::make_pair(img_files[img_idx], std::vector<cv::Rect>()));

    // find all user nodes
    XmlDocument doc;
    if (!doc.load_from_file(filename))
      return false;
    // std::string vid_name = doc.get_value(doc.root(), "tagset.video.videoName");

    // parse all bboxes
    std::vector<XmlDocument::Node*> action_nodes;
    doc.get_all_nodes_at_direction(doc.root(), "tagset.video.action", action_nodes);
    /* action codes:
      1: Discussion between two or more people
      2: Give an object to another person
      3: Put/take an object into/from a box/desk
      4: Enter/leave a room (pass through a door) without unlocking
      5: Try to enter a room (unsuccessfully)
      6: Unlock and enter (or leave) a room
      7: Leave baggage unattended
      8: Handshaking
      9: Typing on a keyboard
      10: Telephone conversation */
    for (unsigned int action_idx = 0; action_idx < action_nodes.size(); ++action_idx) {
      std::vector<XmlDocument::Node*> bbox_nodes;
      doc.get_all_nodes_at_direction(action_nodes[action_idx], "bbox", bbox_nodes);
      for (unsigned int bbox_idx = 0; bbox_idx < bbox_nodes.size(); ++bbox_idx) {
        int framenr = doc.get_node_attribute<int>
            (bbox_nodes[bbox_idx], "framenr", -1);
        if (framenr < 0 || framenr >= (int) img_files.size())
          continue;
        int x = doc.get_node_attribute<int>
            (bbox_nodes[bbox_idx], "x", -1);
        int y = doc.get_node_attribute<int>
            (bbox_nodes[bbox_idx], "y", -1);
        int width = doc.get_node_attribute<int>
            (bbox_nodes[bbox_idx], "width", -1);
        int height = doc.get_node_attribute<int>
            (bbox_nodes[bbox_idx], "height", -1);
        cv::Rect rect(x, y, width, height);
        _frames[ img_files[framenr] ].push_back(rect);
      } // end loop bbox_idx
    } // end loop action_idx
    _frame_it = _frames.begin();
    return load_current_frame();
  } // end load_single_video()

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int n_annotated_frames_in_curr_video() const {
    unsigned int ans = 0;
    for(AnnotMap::const_iterator it = _frames.begin(); it != _frames.end(); ++it)
      ans += (it->second.size() > 0);
    return ans;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int n_frames_in_curr_video() const { return _frames.size(); }

  char display() {
    cv::Mat3b illus = _bgr.clone();
    for (unsigned int rec_idx = 0; rec_idx < _frame_it->second.size(); ++rec_idx) {
      cv::Rect roi = _frame_it->second[rec_idx];
      cv::rectangle(illus, roi, CV_RGB(255, 0, 0), 2);
    } // end loop rec_idx
    cv::imshow("illus", illus);
    return DatabasePlayer::display();
  }

private:
  //////////////////////////////////////////////////////////////////////////////

  /*! \convert Depth values into meters
    (from Nicolas Burrus' Kinect page)
    Please note that only raw_depth values in range [ 0, 1084 ]
    give coherent distance.
   */
  static inline float raw_depth_to_meters(int raw_depth,
                                          const float & nan_value) {
    // if (raw_depth < 2047)
    if (raw_depth < 1084)
      return 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
    return nan_value;
  }

  //! convert \arg short_depth as loaded by imread() into a metric depth map
  static void raw_depth_to_meters(const cv::Mat1w & short_depth,
                                  cv::Mat1f & metric_depth,
                                  const float & nan_value = std::numeric_limits<float>::quiet_NaN()) {
    // create metric_depth with good dimensions
    metric_depth.create(short_depth.size());
    unsigned int nvalues32 = short_depth.cols * short_depth.rows;
    float* vals32 = (float*) metric_depth.ptr();
    unsigned short* vals16 = (unsigned short*) short_depth.ptr();
    for (unsigned int value_idx = 0; value_idx < nvalues32; ++value_idx) {
      // the 11 bits of depth are the _most_ significant bits of the 16 bit JPEG 2000 Image.
      // This was chosen to minimize the artifacts produced by the lossy compression.
      // Therefore you need to divided each depth value by 32 (= 2^5)
      // to get it into a range of [0,2047].
      float val = raw_depth_to_meters(vals16[value_idx] / 32, nan_value);
      if (val > 15) // meters - some of the incorrect depth measures are in [15,25]
        vals32[value_idx] = nan_value;
      else
        vals32[value_idx] = val;
    } // end loop value_idx
  } // end raw_depth_to_meters()

  //////////////////////////////////////////////////////////////////////////////

  bool load_current_frame() {
    std::string bgr_filename = _frame_it->first;
    std::string depth_filename = bgr_filename;
    StringUtils::find_and_replace(depth_filename, "gray", "depth");
    StringUtils::find_and_replace(depth_filename, "jpg", "jp2");
    std::vector<cv::Rect> rois = _frame_it->second;
    printf("load_current_frame('%s', '%s', %i ROIS)\n",
           bgr_filename.c_str(), depth_filename.c_str(), rois.size());
    _has_rgb = _has_depth = _has_user = true;
    _mono = cv::imread(bgr_filename, CV_LOAD_IMAGE_GRAYSCALE);
    if (_mono.empty()) {
      printf("Liris2Imgs: could not load '%s'\n", bgr_filename.c_str());
      return false;
    }
    cv::cvtColor(_mono, _bgr, CV_GRAY2BGR);

    _depth16 = cv::imread(depth_filename, CV_LOAD_IMAGE_UNCHANGED);
    // printf("_depth16:'%s'\n", image_utils::infosImage(_depth16).c_str());
    if (_depth16.empty()){
      printf("Liris2Imgs: could not load '%s'\n", depth_filename.c_str());
      return false;
    }
    // convert to 32f
    raw_depth_to_meters(_depth16, _depth32f, image_utils::NAN_DEPTH);

    // user img
    _user8.create(_bgr.size());
    _user8 = 0;
    return true;
  } // end load_current_frame()

  //////////////////////////////////////////////////////////////////////////////

  std::string _imgs_folder;
  AnnotMap _frames;
  AnnotMap::const_iterator _frame_it;

  cv::Mat1b _mono;
  //cv::Mat1w _depth16;
  cv::Mat _depth16;
}; // end class Liris2Imgs

#endif // LIRIS2IMGS_H
