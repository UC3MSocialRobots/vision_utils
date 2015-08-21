/*!
  \file        ppl_testing.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/12/3

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

#ifndef PPL_TESTING_H
#define PPL_TESTING_H

#include <gtest/gtest.h>
// AD
#include <people_msgs/PeoplePoseList.h>
#include "vision_utils/utils/string_casts_stl.h"
#include "vision_utils/utils/pt_utils.h"
#include "vision_utils/utils/assignment_utils.h"
#include "vision_utils/people_pose_list_utils.h"

#define ASSERT_TRUE_TIMEOUT(cond, timeout) { Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

namespace ppl_utils {


template<class Pt3>
inline void ppl_factory(people_msgs::PeoplePoseList & ppl,
                        const std::vector<Pt3> & exp_users_pos,
                        double pos_error_std_dev = 0,
                        ros::Time stamp = ros::Time::now()) {
  unsigned int nusers = exp_users_pos.size();
  ppl.header.stamp = stamp;
  ppl.poses.clear();
  for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
    people_msgs::PeoplePose pp;
    pp.header = ppl.header;
    pp.person_name = StringUtils::cast_to_string(user_idx);
    pp.std_dev = .1;
    pt_utils::copy3(exp_users_pos[user_idx], pp.head_pose.position);
    combinatorics_utils::add_gaussian_noise(pp.head_pose.position, pos_error_std_dev);
    pp.confidence = 1;
    ppl.poses.push_back(pp);
  } // end loop user_idx
} // end ppl_factory();

////////////////////////////////////////////////////////////////////////////////

inline void check_ppl_equals(const people_msgs::PeoplePoseList & truth_ppl,
                             const people_msgs::PeoplePoseList & tracks_ppl,
                             double pos_error_std_dev = 0) {
  unsigned int ntracks = tracks_ppl.poses.size();
  ASSERT_TRUE(truth_ppl.poses.size() == ntracks);
  // match the tracks with the truth PP
  std::vector<geometry_msgs::Point>
      truth_pos = ppl_utils::ppl2points<geometry_msgs::Point>(truth_ppl),
      tracks_pos = ppl_utils::ppl2points<geometry_msgs::Point>(tracks_ppl);
  assignment_utils::MatchList matchlist_truth2track;
  CMatrix<assignment_utils::Cost> costs;
  assignment_utils::Cost best_cost;
  ASSERT_TRUE(assignment_utils::assign_and_dists
      (truth_pos, tracks_pos, costs, matchlist_truth2track, best_cost));

  // now check each match
  for (unsigned int match_idx = 0; match_idx < matchlist_truth2track.size(); ++match_idx) {
    assignment_utils::Match match = matchlist_truth2track[match_idx];
    int truth_idx = match.first, track_idx = match.second;
    const people_msgs::PeoplePose *truth_pp = &(truth_ppl.poses[truth_idx]),
        *track_pp = &(tracks_ppl.poses[track_idx]);
    // check 3D position
    double dist = geometry_utils::distance_points3
                  (truth_pp->head_pose.position,
                   track_pp->head_pose.position);
    ASSERT_TRUE(dist <= 1E-3 + 5 * pos_error_std_dev) // 1E-3 for rounding errors
      << "track_idx " << track_idx
                << ", users_pos:" << pt_utils::print_point(track_pp->head_pose.position)
                << ", expected:" << pt_utils::print_point(truth_pp->head_pose.position)
                << ", dist:" <<dist;
    // check person_name
    ASSERT_TRUE(truth_pp->person_name == track_pp->person_name)
      << "Truth name #" << truth_idx << " = '" << truth_pp->person_name << "' should be equal to "
                << " track name #" << track_idx << " = '" << track_pp->person_name << "',"
                   // << ", truth:" << ppl_utils::pp2string(*truth_pp)
                   // << ", track:" << ppl_utils::pp2string(*track_pp)
                << ", all truths:" << ppl_utils::ppl2string(truth_ppl)
                << ", all tracks:" << ppl_utils::ppl2string(tracks_ppl);
  } // end for match_idx
} // end check_ppl_equals();

} // end namespace ppl_utils

#endif // PPL_TESTING_H
