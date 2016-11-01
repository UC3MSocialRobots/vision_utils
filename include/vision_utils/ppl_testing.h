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
#include <people_msgs/People.h>
#include "vision_utils/string_casts_stl.h"
#include "vision_utils/pt_utils.h"
#include "vision_utils/assignment_utils.h"
#include "vision_utils/people_pose_list_utils.h"

#define ASSERT_TRUE_TIMEOUT(cond, timeout) { ::vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

namespace vision_utils {


template<class Pt3>
inline void ppl_factory(people_msgs::People & ppl,
                        const std::vector<Pt3> & exp_users_pos,
                        double pos_error_std_dev = 0,
                        ros::Time stamp = ros::Time::now()) {
  unsigned int nusers = exp_users_pos.size();
  ppl.header.stamp = stamp;
  ppl.people.clear();
  for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
    people_msgs::Person pp;
    pp.header = ppl.header;
    pp.name = cast_to_string(user_idx);
    pp.std_dev = .1;
    copy3(exp_users_pos[user_idx], pp.position);
    add_gaussian_noise(pp.position, pos_error_std_dev);
    pp.reliability = 1;
    ppl.people.push_back(pp);
  } // end loop user_idx
} // end ppl_factory();

////////////////////////////////////////////////////////////////////////////////

inline void check_ppl_equals(const people_msgs::People & truth_ppl,
                             const people_msgs::People & tracks_ppl,
                             double pos_error_std_dev = 0) {
  unsigned int ntracks = tracks_ppl.people.size();
  ASSERT_TRUE(truth_ppl.people.size() == ntracks);
  // match the tracks with the truth PP
  std::vector<geometry_msgs::Point>
      truth_pos = ppl2points<geometry_msgs::Point>(truth_ppl),
      tracks_pos = ppl2points<geometry_msgs::Point>(tracks_ppl);
  MatchList matchlist_truth2track;
  CMatrix<Cost> costs;
  Cost best_cost;
  ASSERT_TRUE(assign_and_dists
      (truth_pos, tracks_pos, costs, matchlist_truth2track, best_cost));

  // now check each match
  for (unsigned int match_idx = 0; match_idx < matchlist_truth2track.size(); ++match_idx) {
    Match match = matchlist_truth2track[match_idx];
    int truth_idx = match.first, track_idx = match.second;
    const people_msgs::Person *truth_pp = &(truth_ppl.people[truth_idx]),
        *track_pp = &(tracks_ppl.people[track_idx]);
    // check 3D position
    double dist = distance_points3
                  (truth_pp->position,
                   track_pp->position);
    ASSERT_TRUE(dist <= 1E-3 + 5 * pos_error_std_dev) // 1E-3 for rounding errors
      << "track_idx " << track_idx
                << ", users_pos:" << print_point(track_pp->position)
                << ", expected:" << print_point(truth_pp->position)
                << ", dist:" <<dist;
    // check name
    ASSERT_TRUE(truth_pp->name == track_pp->name)
      << "Truth name #" << truth_idx << " = '" << truth_pp->name << "' should be equal to "
                << " track name #" << track_idx << " = '" << track_pp->name << "',"
                   // << ", truth:" << pp2string(*truth_pp)
                   // << ", track:" << pp2string(*track_pp)
                << ", all truths:" << ppl2string(truth_ppl)
                << ", all tracks:" << ppl2string(tracks_ppl);
  } // end for match_idx
} // end check_ppl_equals();

} // end namespace vision_utils

#endif // PPL_TESTING_H
