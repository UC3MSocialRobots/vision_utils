/*!
  \file        ppl_user_tracker.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/21

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

\class PPLUserTracker
A useful class for using PPL in end user applications.
It stores the ID of the user the robot will interact with,
then computes if the user is visible,
and if yes gives her position,
at any time.
 */
#ifndef PPL_USER_TRACKER_H
#define PPL_USER_TRACKER_H

// msg
#include <people_msgs/People.h>
#include "vision_utils/foo_point.h"

namespace vision_utils {

template<class P3a, class P3b>
inline double dist3sq(const P3a & a, const P3b & b) {
  return (a.x-b.x) * (a.x-b.x) +
      (a.y-b.y) * (a.y-b.y) +
      (a.z-b.z) * (a.z-b.z);
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt3>
int get_closest_pp(const people_msgs::People & ppl,
                   const Pt3 & center) {
  unsigned int npeople = ppl.people.size();
  if (!npeople) {
    printf("get_closest_pp(): empty PPL!\n");
    return -1;
  }
  int closest_pp_idx = 0;
  double best_dist = dist3sq(center, ppl.people[0].position);
  for (unsigned int pp_idx = 1; pp_idx < npeople; ++pp_idx) {
    double dist = dist3sq(center, ppl.people[pp_idx].position);
    if (best_dist > dist) {
      best_dist = dist;
      closest_pp_idx = pp_idx;
    }
  } // end for pp_idx
  return closest_pp_idx;
} // end get_closest_pp

////////////////////////////////////////////////////////////////////////////////

class PPLUserTracker {
public:
  typedef FooPoint3d Pt3;
  typedef people_msgs::People PPL;

  PPLUserTracker() {}

  //////////////////////////////////////////////////////////////////////////////

  bool set_user_to_nearest_person(const PPL & ppl,
                                  Pt3 center = Pt3(0,0,0)) {
    int pp_idx = get_closest_pp<Pt3>(ppl, center);
    if (pp_idx < 0 || pp_idx >= (int) ppl.people.size()) {
      printf("PPLUserTracker::set_user_to_nearest_person(): could not find closest user!\n");
      return false;
    }
    _user_name = ppl.people[pp_idx].name;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * \param ppl
   *    the PPL where to look for the user
   * \param user_pos
   *    [OUT] optional position for the user, if present
   * \return true if the tracked user is present in the PPL
   */
  bool is_user_present(const PPL & ppl,
                       Pt3* user_pos = NULL) {
    unsigned int npeople = ppl.people.size();
    for (unsigned int pp_idx = 0; pp_idx < npeople; ++pp_idx) {
      if (ppl.people[pp_idx].name != _user_name)
        continue;
      if (user_pos != NULL) { // store position
        user_pos->x = ppl.people[pp_idx].position.x;
        user_pos->y = ppl.people[pp_idx].position.y;
        user_pos->z = ppl.people[pp_idx].position.z;
      }
      return true;
    } // end loop pp_idx
    return false;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \ return -1 if user not present
  double user_dist_sq(const PPL & ppl,
                   Pt3 center = Pt3(0,0,0)) {
    Pt3 user_pos;
    if (!is_user_present(ppl, &user_pos))
      return -1;
    return dist3sq(center, user_pos);
  }

  //////////////////////////////////////////////////////////////////////////////

private:
  std::string _user_name;
}; // end PPLUserTracker

} // end namespace vision_utils

#endif // PPL_USER_TRACKER_H
