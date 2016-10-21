/*!
  \file        oni2imgs.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/12

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

#ifndef ONI2IMGS_H
#define ONI2IMGS_H

#include <XnOpenNI.h>
#include <XnTypes.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#include "vision_utils/database_player.h"

// based on http://openni-discussions.979934.n3.nabble.com/OpenNI-dev-Extracting-RGB-stream-into-AVI-file-from-ONI-file-td4025312.html

#define CHECK_RC(rc, what) \
  if (rc != XN_STATUS_OK) { \
  printf("%s failed: %s\n", what, xnGetStatusString(rc)); \
  return false; \
  }

class Oni2Imgs : public DatabasePlayer {
public:
  Oni2Imgs() {}

  //////////////////////////////////////////////////////////////////////////////

  bool go_to_next_frame() {
    //XnStatus nRetVal = context.WaitNoneUpdateAll();
    XnStatus nRetVal = context.WaitAndUpdateAll();
    if (nRetVal != XN_STATUS_OK) {
      if (nRetVal == XN_STATUS_EOF) {
        printf("File '%s' over\n", _playlist[_playlist_idx].c_str());
        return go_to_next_file();
      }
      //if (context.WaitAndUpdateAll() != XN_STATUS_OK) {
      printf("context.WaitNoneUpdateAll() returned %i\n", nRetVal);
      return false;
    }

    if (_has_rgb) {
      rgb = cv::Mat(rgb_rows, rgb_cols, CV_8UC3, (void*) g_rgb_generator.GetRGB24ImageMap());
      cv::cvtColor(rgb, _bgr, CV_RGB2BGR);
    }
    if (_has_depth) {
      depth16 = cv::Mat(depth_rows, depth_cols, CV_16UC1, (void*) g_depth_generator.GetDepthMap());
      depth16.convertTo(_depth32f, CV_32FC1, 1.0 / 1000.0);
    }
    if (_has_user) {
      user16 = cv::Mat(user_rows, user_cols, CV_16UC1, (void*) g_user_generator.GetData());
      user16.convertTo(_user8, CV_8UC1);
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:

  //////////////////////////////////////////////////////////////////////////////

  bool load_single_video(const std::string & filename) {
    printf("Oni2Imgs::load_single_video('%s')\n", filename.c_str());
    XnStatus nRetVal;
    nRetVal = context.Init();
    CHECK_RC(nRetVal, "Init");
    xn::ProductionNode node;
    nRetVal = context.OpenFileRecording(filename.c_str(), node);
    //nRetVal = xnContextOpenFileRecordingEx(&context, filename.c_str());
    CHECK_RC(nRetVal, "Open input file");

    // https://groups.google.com/forum/#!topic/openni-dev/iZU7uFInloY
    bool oni_has_player = (context.FindExistingNode(XN_NODE_TYPE_PLAYER, player) == XN_STATUS_OK);
    if (oni_has_player) {
      bool want_repeat_file = (_playlist.size() == 1 && _want_repeat_playlist);
      player.SetRepeat(want_repeat_file);
    }
    else
      printf("Oni2Imgs::from_file('%s'): could not set repeat mode!\n",
             filename.c_str());

    _has_rgb = (context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_rgb_generator) == XN_STATUS_OK);
    // get rows, cols
    XnMapOutputMode rgbMapMode;
    g_rgb_generator.GetMapOutputMode(rgbMapMode);
    rgb_rows = rgbMapMode.nYRes;
    rgb_cols = rgbMapMode.nXRes;

    _has_depth = (context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth_generator) == XN_STATUS_OK);
    // get rows, cols
    XnMapOutputMode depthMapMode;
    g_depth_generator.GetMapOutputMode(depthMapMode);
    depth_rows = depthMapMode.nYRes;
    depth_cols = depthMapMode.nXRes;

    _has_user = (context.FindExistingNode(XN_NODE_TYPE_USER, g_user_generator) == XN_STATUS_OK);
    // get rows, cols
    xn::SceneMetaData userMD;
    if (_has_user)
      g_user_generator.GetUserPixels(0, userMD);
    user_cols = userMD.XRes();
    user_rows = userMD.YRes();

    //  xn::SceneAnalyzer g_scene_generator;
    //  bool has_scene = (context.FindExistingNode(XN_NODE_TYPE_SCENE, g_scene_generator) == XN_STATUS_OK);
    //  printf("has_scene:%i\n", has_scene);

    printf("Oni2Imgs: file:'%s', rgb:%i, depth::%i, user:%i\n",
           filename.c_str(), _has_rgb, _has_depth, _has_user);
    return true;
  } // end load_single_video()

  int rgb_cols, rgb_rows;
  cv::Mat3b rgb;

  int depth_cols, depth_rows;
  cv::Mat1w depth16;

  int user_cols, user_rows;
  cv::Mat1w user16;

protected:
  xn::Context context;
  xn::Player player;
  xn::ImageGenerator g_rgb_generator;
  xn::DepthGenerator g_depth_generator;
  xn::UserGenerator g_user_generator;
}; // end class Oni2Imgs

#endif // ONI2IMGS_H
