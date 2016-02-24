/*!
  \file        ppl2ms.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/23

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

A class for drawing peoplemsgs::PeoplePoseList
on a MiniStage visualizer.

\section Parameters
  - \b "~reprojection_mode"
        [string] (default: "xy")
        The reprojection mode, i.e. the coordinates to keep for reprojecting to 2D.
        Valid values: "xy" or "xz".

  - \b "~width", "~height"
        [int, pixels] (default: 800, 600)
        The dimensions in pixels of the window

  - \b "~xmin, ~xmax, ~ymin, ~ymax"
        [double, meters] (default: -4..4, -4..4)
        The visible window

  - \b "~draw_track_trails"
        [bool] (default: true)
        False for only drawing detections and not their trails.

  - \b "~draw_track_images"
        [bool] (default: true)
        True for drawing the RGB images associated with trails.

  - \b "~method2trail_color"
        [bool] (default: true)
        If true, the color of a user trail is determined by PP method.
        Otherwise, determined by PP name.

  - \b "~trail_history_size"
        [int] (default: DEFAULT_TRAIL_HISTORY_SIZE)
        If ~draw_track_images activated, number of points for the trail length.
 */
#ifndef PPL2MS_H
#define PPL2MS_H

#include "vision_utils/ppl_attributes.h"
#include "vision_utils/mini_stage.h"
#include "vision_utils/utils/pt_utils.h"
#include "vision_utils/utils/map_utils.h"
#include "vision_utils/color_utils.h"
#include <cv_bridge/cv_bridge.h>
#include "vision_utils/atan3D.h"

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

class PPL2MS {
public:
  typedef people_msgs::PeoplePose PP;
  typedef people_msgs::PeoplePoseList PPL;
  typedef std::string UserName;
  typedef std::string MethodName;
  typedef cv::Point3d Pt3d;
  typedef cv::Point   Pt2d;
  typedef std::vector<Pt3d> Track3d;
  typedef std::vector<Pt2d> Track2d;
  //! unique identifier pair (method_name, user_name)
  struct UserTrack {
    MethodName method_name;
    UserName user_name;
    Track3d track3d;
    Track2d track2d;
  };

  static const unsigned int MAX_PERSON_HEIGHT = 100; // pixels
  static const int DEFAULT_TRAIL_HISTORY_SIZE = 100; // vector size

  PPL2MS() {
    ros::NodeHandle nh_private("~");
    width = 800; height = 600; xmin = -4; xmax = 4; ymin = -4; ymax = 4;
    params_applied_to_ms = false;
    nh_private.param("width", width, width);
    nh_private.param("height", height, height);
    nh_private.param("xmin", xmin, xmin);
    nh_private.param("xmax", xmax, xmax);
    nh_private.param("ymin", ymin, ymin);
    nh_private.param("ymax", ymax, ymax);
    nh_private.param("draw_track_trails", _draw_track_trails, true);
    nh_private.param("draw_track_images", _draw_track_images, true);
    nh_private.param("method2trail_color", _method2trail_color, true);
    const int DEFAULT_TRAIL_HISTORY_SIZE_auxConst = DEFAULT_TRAIL_HISTORY_SIZE;
    nh_private.param("trail_history_size", _trail_history_size, DEFAULT_TRAIL_HISTORY_SIZE_auxConst);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! prepair background
  inline void clear_bg(MiniStage & ms) {
    ms.clear();
    if (!params_applied_to_ms) {
      ms.set_dims(width, height);
      ms.set_visible_window(xmin, xmax, ymin, ymax);
      params_applied_to_ms = true;
    }
    //ms.set_origin(cv::Point2f(0, 0));
    ms.set_heading(0, false);
    ms.draw_grid(1., 160);
    ms.draw_axes(2);
    //ms.set_scale(1 / 50.f);
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Add a new PPL to the history and
   * draw all the previously received PPL
   * \brief draw_new_ppl
   * \param ms
   * \param new_measure
   * \param want_clear_before
   */
  void add_ppl_and_redraw_all(MiniStage & ms,
                              const PPL & new_measure,
                              bool want_clear_before = true) {
    MethodName method  = new_measure.method;
    if (method.empty()) {
      printf("PPLViewer:PPL with no method specified, cant process it!\n");
      return;
    }
    DEBUG_PRINT("PPLViewer::add_ppl_and_redraw_all('%s')\n", method.c_str());
    // insert method in the list of seen methods
    for (unsigned int detec_idx = 0; detec_idx < new_measure.poses.size(); ++detec_idx) {
      const PP* pp = &(new_measure.poses[detec_idx]);
      UserTrack* curr_track = get_track(method, pp->person_name);
      // add detections 3D points to tracks
      Pt3d user_pos;
      pt_utils::copy3(pp->head_pose.position, user_pos);
      Track3d* user_track3d = &(curr_track->track3d);
      Track2d* user_track2d = &(curr_track->track2d);
      user_track3d->push_back(user_pos);
      user_track2d->push_back(world2pixel(ms, user_pos));

      // clean old points
      int nelems_to_delete = user_track3d->size() - DEFAULT_TRAIL_HISTORY_SIZE;
      if (nelems_to_delete > 0) {
        user_track3d->erase(user_track3d->begin(), user_track3d->begin() + nelems_to_delete);
        user_track2d->erase(user_track2d->begin(), user_track2d->begin() + nelems_to_delete);
      }
    } // end loop detec_idx

    // store this PPL
    _method2ppl[method] =  new_measure;

    // prepair background
    if (want_clear_before)
      clear_bg(ms);

    // draw all tracks
    if (_draw_track_trails)
      redraw_track_trails(ms);

    // redraw all PPL
    std::map<MethodName, PPL>::const_iterator it = _method2ppl.begin();
    while (it != _method2ppl.end()) {
      draw_one_ppl(ms, it->second);
      ++it;
    }
    DEBUG_PRINT("end draw_new_ppl()\n");
  } // end draw_new_ppl();

  //////////////////////////////////////////////////////////////////////////////

  void redraw_blobs(MiniStage & ms,
                    const PPL::ConstPtr & ppl) {
    // draw blobs
    unsigned int n_blobs = ppl->poses.size();
    for (unsigned int blob_idx = 0; blob_idx < n_blobs; ++blob_idx) {
      const PP* pp = &(ppl->poses[blob_idx]);
      geometry_msgs::Point blob_position = pp->head_pose.position;
      double blob_confidence = pp->confidence;
      // draw blob center
      cv::circle(ms.get_viz(), world2pixel(ms, blob_position),
                 3, CV_RGB(0, 0, 255), -1);
      // draw blob radius
      cv::circle(ms.get_viz(), world2pixel(ms, blob_position),
                 30 * blob_confidence, CV_RGB(0, 0, 255), 2);
      // draw confidence
      std::ostringstream text; text << "conf:" << blob_confidence;
      cv::putText(ms.get_viz(), text.str(),
                  world2pixel(ms, blob_position),
                  CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 255));
      // draw assigns
#if 0
      for (unsigned int assign_id = 0;
           assign_id < unassociated_poses_to_blobs_assignments[blob_idx].size();
           ++assign_id) {
        int uap_idx = unassociated_poses_to_blobs_assignments[blob_idx][assign_id];
        geometrymsgs::Point head_position =
            unassociated_poses[uap_idx].head_pose.position;
        cv::line(ms.get_viz(),
                 world2pixel(blob_position.x, blob_position.y),
                 world2pixel(head_position.x, head_position.y),
                 CV_RGB(0, 0, 255), 1);
      } // end loop assign_id
#endif
    } // end loop blob_idx
  } //end redraw_blobs();

  //////////////////////////////////////////////////////////////////////////////

  //! \return the number of unique PPL methods that we effectively received
  inline unsigned int get_nb_received_methods() const { return _method2ppl.size(); }

  //! \return the name of all unique PPL methods received
  inline std::vector<MethodName> get_all_received_methods() const {
    std::vector<MethodName> ans;
    map_utils::map_keys_to_container(_method2ppl, ans);
    return ans;
  }

  //! \return the number of tracks for a specific PPL method
  inline unsigned int get_nb_received_tracks(const MethodName & method) const {
    int ans = 0;
    for (unsigned int track_idx = 0; track_idx < _user_tracks.size(); ++track_idx)
      if (_user_tracks[track_idx].method_name  == method)
        ++ans;
    return ans;
  }

  //! \return the total number of tracks (unique (user, PPL method) identifier)
  inline unsigned int get_nb_total_received_tracks() const {
    return _user_tracks.size();
  }

  //////////////////////////////////////////////////////////////////////////////

private:

  //////////////////////////////////////////////////////////////////////////////

  void draw_one_ppl(MiniStage & ms,
                    const PPL & new_measure) {
    DEBUG_PRINT("PPL2MS::draw_one_ppl('%s')\n", new_measure.method.c_str());

    // find color
    MethodName method  = new_measure.method;

    for (unsigned int detec_idx = 0; detec_idx < new_measure.poses.size(); ++detec_idx) {
      const PP* pp = &(new_measure.poses[detec_idx]);
      std::string key = (_method2trail_color ? method : pp->person_name);
      cv::Scalar color = pp2color(key);
      cv::Point user_center = world2pixel(ms, pp->head_pose.position);
      UserName user_name = pp->person_name;
      cv::circle(ms.get_viz(), user_center, 15, cv::Scalar::all(0), -1);
      cv::circle(ms.get_viz(), user_center, 15, color, 5);

      // add the image
      if (_draw_track_images
          && pp->rgb.width > 0
          && pp->rgb.height > 0
          && pp->user.width == pp->rgb.width
          && pp->user.height == pp->rgb.height) {
        boost::shared_ptr<void const> tracked_object;
        try {
          cv_bridge::CvImageConstPtr pp_rgb = cv_bridge::toCvShare(pp->rgb, tracked_object);
          cv_bridge::CvImageConstPtr pp_user = cv_bridge::toCvShare(pp->user, tracked_object);
          pp_rgb->image.copyTo(_rgb_buffer);
          pp_user->image.copyTo(_user_buffer);
        }
        // DEBUG_PRINT("Channels:rgb:%i, user:%i\n", pp_rgb->image.channels(), pp_user->image.channels());
        catch (cv_bridge::Exception e) {
          printf("PPL2MS: exception in cv_bridge: '%s'\n", e.what());
          return;
        }
        image_utils::resize_if_bigger(_rgb_buffer, _rgb_buffer,
                                      MAX_PERSON_HEIGHT, MAX_PERSON_HEIGHT);
        image_utils::resize_if_bigger(_user_buffer, _user_buffer,
                                      MAX_PERSON_HEIGHT, MAX_PERSON_HEIGHT);
        int x = user_center.x + 10, y = user_center.y + 10;
        image_utils::paste_img(_rgb_buffer, ms.get_viz(), x, y, &_user_buffer);
      }

      // draw method name
      std::ostringstream label;
      label << method << ":" << user_name;
      std::string face_name;
      if (ppl_utils::get_attribute_readonly(*pp, "face_name", face_name))
        label << "='" << face_name << "'";
      cv::putText(ms.get_viz(), label.str(), user_center + cv::Point(10, -10),
                  CV_FONT_HERSHEY_PLAIN, 1, color, 1);

      // draw detections
      //image_utils::drawCross(ms.get_viz(), world2pixel(ms, user_pos), 5, pp_color, 2);
    } // end loop detec_idx
  } // end draw_one_ppl();

  //////////////////////////////////////////////////////////////////////////////

  void redraw_track_trails(MiniStage & ms) {
    DEBUG_PRINT("redraw_track_trails()\n");
    if (_user_tracks.empty())
      return;

    for (unsigned int track_idx = 0; track_idx < _user_tracks.size(); ++track_idx) {
      UserTrack* curr_track = &(_user_tracks[track_idx]);
      MethodName method  = curr_track->method_name;
      UserName user_name = curr_track->user_name;
      Track3d* user_track3d = &(curr_track->track3d);
      Track2d* user_track2d = &(curr_track->track2d);

      std::string key = (_method2trail_color ? method : user_name);
      cv::Scalar pp_color = pp2color(key);
      cv::Vec3b pp_color3d(pp_color[0], pp_color[1], pp_color[2]);
      // draw track2d
      unsigned int npts = user_track3d->size();
      if (user_track2d->size() != npts) { // reconvert 3D -> 2D if needed
        DEBUG_PRINT("PPLViewer:recomputing track2d for method '%s', user '%s'\n",
                   method.c_str(), user_name.c_str());
        user_track2d->clear();
        user_track2d->reserve(npts);
        for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
          user_track2d->push_back(world2pixel(ms, (*user_track3d)[pt_idx]));
      }
      image_utils::drawPolygon(ms.get_viz(), *user_track2d, false, pp_color, 2);
    } // end loop (track_idx)


#if 0
    // draw affectations
    for (unsigned int affec_idx = 0; affec_idx < affectations.size(); ++affec_idx) {
      int detec_idx = affectations[affec_idx].first;
      int track_idx = affectations[affec_idx].second;
      Pt3d track_pos;
      _method2user2data[track_idx].get_position(track_pos);
      cv::line(ms.get_viz(),
               world2pixel(ms, new_measure.poses[detec_idx].head_pose.position),
               world2pixel(ms, track_pos),
               pp_color, 1);
    } // end loop affec_idx

    // draw unassociated_poses
    for (unsigned int uap_idx = 0; uap_idx < _unassociated_poses.size(); ++uap_idx) {
      geometrymsgs::Point head_position =
          _unassociated_poses[uap_idx].head_pose.position;
      cv::circle(ms.get_viz(), world2pixel(ms, head_position.x, head_position.y),
                 3, pp_color, 2);
    } // end loop uap_idx
#endif
  } // end redraw_track_trails()

  //////////////////////////////////////////////////////////////////////////////

  UserTrack* get_track(const MethodName &  method, const UserName & user) {
    unsigned int ntracks = _user_tracks.size();
    for (unsigned int track_idx = 0; track_idx < ntracks; ++track_idx) {
      UserTrack* curr_track = &(_user_tracks[track_idx]);
      if (curr_track->method_name == method && curr_track->user_name == user)
        return curr_track;
    } // end loop track_idx
    UserTrack new_track;
    new_track.method_name = method;
    new_track.user_name = user;
    _user_tracks.push_back(new_track);
    return &(_user_tracks.back());
  }

  //////////////////////////////////////////////////////////////////////////////

  template<class Pt3>
  inline cv::Point world2pixel(const MiniStage & ms, const Pt3 & pt3) {
    double x2D, y2D;
    _mode.convert3Dto2D(pt3.x, pt3.y, pt3.z, x2D, y2D);
    return ms.world2pixel(x2D, y2D);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline cv::Scalar pp2color(const std::string & key) {
    cv::Scalar color;
    if (map_utils::direct_search(_pp2color, key, color))
      return color;
    color = color_utils::color_scalar<cv::Scalar>(_pp2color.size());
    _pp2color[key] = color;
    return color;
  } // end

  //////////////////////////////////////////////////////////////////////////////

  std::map<MethodName, cv::Scalar> _pp2color;
  cv::Mat3b _rgb_buffer;
  cv::Mat1b _user_buffer;
  std::vector<UserTrack> _user_tracks;
  std::map<MethodName, PPL> _method2ppl;
  ReprojectionMode _mode;
  bool params_applied_to_ms;
  double width, height, xmin, xmax, ymin, ymax;
  bool _draw_track_trails, _draw_track_images;
  int _trail_history_size;
  bool _method2trail_color;
}; // end PPL2MS

#endif // PPL2MS_H
