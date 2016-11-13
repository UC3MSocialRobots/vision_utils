/*!
  \file        ppl_viewer.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/2

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

\section Parameters
  - \b "ppl_topics"
        [string, semicolon separated] (default: "ppl")
        The PPL topics to subscribe to.
        They should be PPL computed by a given algorhithm.

  - \b "scan_topic"
        [string] (default: "")
        A laser scan to subscribe to.
        Leave empty for no scan.

  - \b "reprojection_mode"
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

  - \b "~trail_history_size"
        [int] (default: DEFAULT_TRAIL_HISTORY_SIZE)
        If ~draw_track_trails activated, number of points for the trail length.

  - \b "~draw_track_images"
        [bool] (default: true)
        True for drawing the RGB images associated with tracks.

  - \b "~save_images"
        [bool] (default: false)
        true to save each displayed image as a file on disk

\section Subscriptions
  - \b {ppl_topics}
        [people_msgs::People]
        One or several computed PPL, see above.

\section Publications
    None.
 */

#ifndef PPL_VIEWER_H
#define PPL_VIEWER_H

#include "vision_utils/convert_sensor_data_to_xy.h"
#include "vision_utils/foo_point.h"
#include "vision_utils/multi_subscriber.h"
#include "vision_utils/ppl2ms.h"
#include "vision_utils/timestamp.h"
#include "cvstage/plugins/draw_xy_lines.h"
// ROS
#include <tf/transform_listener.h>

namespace vision_utils {

class PPLViewer {
public:
  typedef people_msgs::Person PP;
  typedef people_msgs::People PPL;
  typedef std::string MethodName;

  PPLViewer() {
    // prepair GUI
    _window_name = "ukf_multimodal";
    //_ms.set_mouse_move_callback(_window_name);
    // cv::namedWindow(_window_name);
    _frame = "/odom";
    set_display(true);
    _ppl2ms.clear_bg(_ms);
    // get topic names
    ros::NodeHandle nh_public, nh_private("~");
    std::string ppl_topics = "ppl", blob_topic = "", scan_topic = "";
    nh_private.param("ppl_topics", ppl_topics, ppl_topics);
    nh_private.param("blob_topic", blob_topic, blob_topic);
    nh_private.param("scan_topic", scan_topic, scan_topic);
    nh_private.param("save_images", _save_images, false);
    // create multi subscriber
    ppl_subscribers = MultiSubscriber::subscribe
        (nh_public, ppl_topics, 1, &PPLViewer::draw_new_ppl, this);
    if (!blob_topic.empty())
      blob_subscriber = nh_public.subscribe(blob_topic, 1, &PPLViewer::redraw_blobs, this);
    if (!scan_topic.empty())
      scan_subscriber = nh_public.subscribe(scan_topic, 1, &PPLViewer::redraw_scans, this);
    printf("PPLViewer: subscribing to PPL topics '%s', blob topic:'%s'\n",
           ppl_subscribers.getTopics(true).c_str(),
           blob_subscriber.getTopic().c_str());
  }

  //////////////////////////////////////////////////////////////////////////////

  ~PPLViewer() {
    // cv::destroyWindow(_window_name);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! the nb of PPL topics we subscribe to, and theoretical number of methods available
  inline unsigned int get_nb_subscribed_methods() const { return ppl_subscribers.nTopics(); }

  /*! the real number of PPL publishers we listen to.
   *  If each subscribed PPL topic corresponds to a real publihser,
   *  then it is = get_nb_subscribed_methods() */
  inline unsigned int get_nb_PPL_publishers() const { return ppl_subscribers.getNumPublishers(); }

  inline void set_display(bool display) { _display = display; }

  //////////////////////////////////////////////////////////////////////////////

  /*** methods comming from PPL2MS */
  inline unsigned int get_nb_received_methods() const {
    return _ppl2ms.get_nb_received_methods();
  }

  inline std::vector<MethodName> get_all_received_methods() const {
    return _ppl2ms.get_all_received_methods();
  }

  inline unsigned int get_nb_received_tracks(const MethodName & method) const {
    return _ppl2ms.get_nb_received_tracks(method);
  }

  inline unsigned int get_nb_total_received_tracks() const {
    return _ppl2ms.get_nb_total_received_tracks();
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  // callback called on reception of a new PPL. Clear the image and redraw all tracks
  void draw_new_ppl(const PPL::ConstPtr & ppl) {
    _frame = ppl->header.frame_id;
    _ppl2ms.add_ppl_and_redraw_all(_ms, *ppl, true);
    // draw scan
    // convert to (x,y) in msg frame
    if (!_last_scan.ranges.empty()) {
      convert_sensor_data_to_xy(_last_scan, _pts_src_frame);
      // double scan_z_dst_frame = 0;
      //    if (convert_xy_vec_frame(_last_scan.header,
      //                                           _pts_src_frame,
      //                                           _tf_listener,
      //                                           _frame,
      //                                           _pts_dst_frame,
      //                                           scan_z_dst_frame)) {
      cvstage_plugins::plot_xy_pts(_ms, _pts_src_frame, CV_RGB(255,0,0), 2);
    }
    save_images_and_display();
  } // end draw_new_ppl();

  //////////////////////////////////////////////////////////////////////////////

  void redraw_blobs(const PPL::ConstPtr & ppl) {
    _ppl2ms.redraw_blobs(_ms, ppl);
    save_images_and_display();
  }
  //////////////////////////////////////////////////////////////////////////////

  void redraw_scans(const sensor_msgs::LaserScan::ConstPtr & scan) {
    _last_scan = *scan;
  }

  //////////////////////////////////////////////////////////////////////////////

  void save_images_and_display() {
    // show stuff
    if (!_display)
      return;
    cv::imshow(_window_name, _ms.get_viz());
    char c = cv::waitKey(50);
    if (c == 's' || _save_images) {
      std::ostringstream filename;
      filename << "/tmp/PPLViewer_" << timestamp() << ".png";
      if (!cv::imwrite(filename.str(), _ms.get_viz()))
        printf("/!\\ Could not write file '%s'\n", filename.str().c_str());
      printf("Succesfully written file '%s'\n", filename.str().c_str());
     }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  bool _display, _save_images;
  MiniStage _ms;
  MultiSubscriber ppl_subscribers;
  ros::Subscriber blob_subscriber;
  ros::Subscriber scan_subscriber;
  std::string _window_name;
  PPL2MS _ppl2ms;
  typedef FooPoint2f SimplePt2D;
  sensor_msgs::LaserScan _last_scan;
  std::vector<SimplePt2D> _pts_src_frame, _pts_dst_frame;
  tf::TransformListener _tf_listener;
  std::string _frame;
}; // end class PPLViewer

} // end namespace vision_utils

#endif // PPL_VIEWER_H
