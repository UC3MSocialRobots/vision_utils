/*!
  \file        pplp_benchmarker.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/6

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

A node that subscribes to a ground truth PPL topic
and one or several computed PPL topics,
and evaluates the mean error of each PPL method.

\section Parameters
  - \b "ground_truth_ppl_topic"
        [string] (default: "ground_truth_ppl")
        The PPL topic that will contain the ground truth PPL
        of the data that is supplied to the PPL algorithms.

  - \b "ppl_topics"
        [string, semicolon separated] (default: "ppl")
        The PPL topics to subscribe to.
        They should be PPL computed by a given algorhithm.

  - \b "errors_display_timeout"
        [double, sec] (default: 5)
        The frequency at which the benchmark results should be printed out,
        in seconds.

  - \b "results_filename"
        [string] (default: "")
        If not empty, the results are saved in this file
        every {errors_display_timeout} seconds.

\section Subscriptions
  - \b {ground_truth_ppl_topic}
        [people_msgs::PeoplePoseList]
        The truth PPL, see above.

  - \b {ppl_topics}
        [people_msgs::PeoplePoseList]
        One or several computed PPL, see above.

\section Publications
  - \b "~ppl"
        [people_msgs::PeoplePoseList]
        The detected users in the mask
 */
#ifndef PPL_BENCHMARKER_H
#define PPL_BENCHMARKER_H

#include <people_msgs/PeoplePoseList.h>
#include <src/ros_utils/multi_subscriber.h>
#include <src/ros_utils/pt_utils.h>
#include <src/time/timer.h>
#include <src/string/file_io.h>
#include <src/combinatorics/assignment_utils.h>
// ros
#include <ros/package.h>

class PPLBenchmarker {
public:
  typedef people_msgs::PeoplePose PP;
  typedef people_msgs::PeoplePoseList PPL;
  typedef std::string MethodName;
  typedef std::string UserName;
  typedef geometry_utils::FooPoint3f Pt3f;
  static const int UNASSIGNED = assignment_utils::UNASSIGNED;

  PPLBenchmarker() {
    // get the topic names
    ros::NodeHandle nh_public, nh_private("~");
    std::string ppl_topics = "ppl", ground_truth_ppl_topic = "ground_truth_ppl";
    nh_private.param("ppl_topics", ppl_topics, ppl_topics);
    nh_private.param("ground_truth_ppl_topic", ground_truth_ppl_topic, ground_truth_ppl_topic);
    _errors_display_timeout = 5;
    nh_private.param("errors_display_timeout", _errors_display_timeout, _errors_display_timeout);
    _results_filename = "";
    nh_private.param("results_filename", _results_filename, _results_filename);

    // subscribers
    _ppl_subs = ros::MultiSubscriber::subscribe
        (nh_public, ppl_topics, 1, &PPLBenchmarker::ppl_cb, this);
    _ground_truth_ppl_sub = nh_public.subscribe
        (ground_truth_ppl_topic, 1, &PPLBenchmarker::ground_truth_ppl_cb, this);
    _last_ground_truth_ppl_was_set = false;
    printf("PPLBenchmarker: getting truth PPL on '%s', computed PPLs on %s, "
           "saving results on file '%s' and displaying every %g secs\n",
           _ground_truth_ppl_sub.getTopic().c_str(), _ppl_subs.getTopics().c_str(),
           _results_filename.c_str(), _errors_display_timeout);
  }

  //////////////////////////////////////////////////////////////////////////////

  ~PPLBenchmarker() {
    save_and_display_results(true);
  } // end dtor

  //////////////////////////////////////////////////////////////////////////////

  inline void save_and_display_results(bool force_display = false) {
    if (!force_display && _errors_timer.getTimeSeconds() < _errors_display_timeout)
      return;
    _errors_timer.reset();
    std::ostringstream ans;
    ans << _errors.size() << " methods:\n";
    std::map<MethodName, MethodError>::const_iterator e_it = _errors.begin();
    while(e_it != _errors.end()) {
      ans << " * " << e_it->first << ":\t "<< e_it->second.to_string() << "\n";
      ++e_it;
    }
    if (!_results_filename.empty())
      StringUtils::save_file(_results_filename, ans.str());
    printf("PPLBenchmarker: results: %s\n", ans.str().c_str());
  } // end save_and_display_results();

  //////////////////////////////////////////////////////////////////////////////

  void ppl_cb(const PPL::ConstPtr & msg) {
    if (!_last_ground_truth_ppl_was_set) {
      printf("PPLBenchmarker: got a computed PPL but no truth PPL, can't compare!\n");
      return;
    }
    MethodName method_name = msg->method;
    if (method_name.size() == 0) {
      printf("PPLBenchmarker: got a computed PPL but the method field is empty, can't compare!\n");
      return;
    }
    unsigned int detec_people_nb = msg->poses.size();
    //ROS_INFO_THROTTLE(5, "PPLBenchmarker:ppl_cb(method:'%s', %i people, truth:%i people)",
    //                  method_name.c_str(), detec_people_nb, _truth_people_nb);
    // compute the error
    MethodError* e = &(_errors[method_name]);
    // process the cases with one of the detections == zero
    if (detec_people_nb == 0 && _truth_people_nb == 0) {
      e->true_negative++;
      return;
    }
    else if (detec_people_nb == 0 && _truth_people_nb != 0) {
      e->false_negative += _truth_people_nb;
      return;
    }
    else if (detec_people_nb != 0 && _truth_people_nb == 0) {
      e->false_positive += detec_people_nb;
      return;
    }
    // (detec_people_nb > 0 && _truth_people_nb > 0)
    // find the best assignment
    double dist2truth_per_pp = 0;
    std::vector<Pt3f> detec_people_pos;
    for (unsigned int people_idx = 0; people_idx < detec_people_nb; ++people_idx) {
      detec_people_pos.push_back(Pt3f());
      pt_utils::copy3(msg->poses[people_idx].head_pose.position, detec_people_pos.back());
    } // end for people_idx
    assignment_utils::MatchList matchlist_truth2detec;
    assignment_utils::assign_and_dists(_truth_people_pos, detec_people_pos, _costs,
                                       matchlist_truth2detec, dist2truth_per_pp);

    // check the MatchList
    bool has_swapped_ids = false;
    for (unsigned int match_idx = 0; match_idx < matchlist_truth2detec.size(); ++match_idx) {
      assignment_utils::Match match = matchlist_truth2detec[match_idx];
      int truth_idx =match.first, detec_idx = match.second;
      if (truth_idx == UNASSIGNED && detec_idx == UNASSIGNED) // shouldnt occur
        e->true_negative++;
      if (truth_idx != UNASSIGNED && detec_idx == UNASSIGNED)
        e->false_negative++;
      else if (truth_idx == UNASSIGNED && detec_idx != UNASSIGNED)
        e->false_positive++;
      if (truth_idx == UNASSIGNED || detec_idx == UNASSIGNED) // nothing more to do
        continue;
      // both are assigned
      e->true_positive++;
      // printf("%s: Checking labels...\n", method_name.c_str());
      UserName curr_label = msg->poses[detec_idx].person_name;
      if (curr_label == people_msgs::PeoplePose::NO_RECOGNITION_MADE)
        continue;
      UserName real_label = _last_ground_truth_ppl.poses[truth_idx].person_name;
      // determine expected label

      NamesMap* method_name_map = &(method2truth2track[method_name]);
      if (method_name_map->find(real_label) == method_name_map->end()) // never seen this track?
        (*method_name_map)[real_label] = curr_label;
      UserName exp_label = (*method_name_map)[real_label];
      // compare expected label and curr label
      if (exp_label != curr_label) {
        has_swapped_ids = true;
        (*method_name_map)[real_label] = curr_label;
      }
      //printf("%s: real_label:'%s' (expecting label '%s'), curr_label:'%s'\n",
      //       method_name.c_str(), real_label.c_str(), exp_label.c_str(), curr_label.c_str());
    } // end loop match_idx

    // only count one swap per full match
    if (has_swapped_ids)
      e->id_swaps++;

    // update the measurement error
    unsigned int n_pps =std::min(_truth_people_nb, detec_people_nb);
    dist2truth_per_pp *= 1. / n_pps; // normalize
    e->add_measure_error(dist2truth_per_pp, n_pps);
  } // end ppl_cb();

  //////////////////////////////////////////////////////////////////////////////

  void ground_truth_ppl_cb(const PPL::ConstPtr & msg) {
    _last_ground_truth_ppl = *msg;
    // store the 3D positions of each user
    _truth_people_nb = msg->poses.size();
    _truth_people_pos.clear();
    for (unsigned int people_idx = 0; people_idx < _truth_people_nb; ++people_idx) {
      _truth_people_pos.push_back(Pt3f());
      pt_utils::copy3(msg->poses[people_idx].head_pose.position, _truth_people_pos.back());
    } // end for people_idx
    _last_ground_truth_ppl_was_set = true;
    save_and_display_results();
  } // end ground_truth_ppl_cb();

  //////////////////////////////////////////////////////////////////////////////

private:
  ros::Subscriber _ground_truth_ppl_sub;
  ros::MultiSubscriber _ppl_subs;
  PPL _last_ground_truth_ppl;
  bool _last_ground_truth_ppl_was_set;
  std::vector<Pt3f> _truth_people_pos;
  unsigned int _truth_people_nb;

  //! error mesasuring and storing /////////////////////////////////////////////
  struct MethodError {
    MethodError() : total_pps(0), avg_pp_dist2truth(0) {
      true_positive = false_positive = true_negative = false_negative = 0;
      id_swaps = 0;
    }

    // https://en.wikipedia.org/wiki/Precision_and_recall#Definition_.28classification_context.29
    void add_measure_error(const double dist2truth_per_pp, const unsigned int n_pps) {
      avg_pp_dist2truth = (total_pps * avg_pp_dist2truth + dist2truth_per_pp)
          / (n_pps + total_pps);
      total_pps += n_pps;
      ++total_ppls;
    } // end add_measure();

    inline std::string to_string() const {
      std::ostringstream out;
      out << true_positive << "-truepos(hit), ";
      out << true_negative << "-trueneg(correct rejection),";
      out << false_positive << "-falsepos(false alarm),";
      out << false_negative << "-falseneg(miss),";
      out << " avg_pp_dist2truth:" << std::setprecision(3)
          << avg_pp_dist2truth;
      out << " hitrate(recall):" << std::setprecision(3)
          << 1. * true_positive / (true_positive + false_negative);
      out << " accuracy:" << std::setprecision(3)
          << 1. * (true_positive + true_negative)
             / (true_positive + false_positive + true_negative + false_negative);
      out << " " << id_swaps << " id_swaps of " << total_ppls << " PPLs";
      return out.str();
    }

    unsigned int true_positive, false_positive, true_negative, false_negative;
    unsigned int total_pps, total_ppls;
    // labelling
    unsigned int id_swaps;
    double avg_pp_dist2truth;
  }; // enc struct MethodError /////////////////////////////////////////////////

  std::string _results_filename;
  std::map<MethodName, MethodError> _errors;
  typedef std::map<UserName, UserName> NamesMap;
  std::map<MethodName, NamesMap> method2truth2track;
  Timer _errors_timer;
  double _errors_display_timeout;
  CMatrix<assignment_utils::Cost> _costs;
}; // end class PPLBenchmarker

#endif // PPL_BENCHMARKER_H
