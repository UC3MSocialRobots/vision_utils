/*!
  \file        pplm_testing.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/2/1

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

Some useful classes for benchmarking PPLMatcherTemplate children classes.
 */
#ifndef PPLM_TESTING_H
#define PPLM_TESTING_H

#include <gtest/gtest.h>
#include <ros/service_client.h>
// AD
#include "vision_utils/database_player.h"
#include "vision_utils/pplm_template.h"

#include "vision_utils/images2ppl.h"
#include "vision_utils/timer.h"
#include <vision_utils/rosmaster_alive.h>
#include <vision_utils/img_path.h>

#define ASSERT_TRUE_TIMEOUT(cond, timeout) { ::vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

namespace vision_utils {

/*! create a PPL from a given filename_prefix.
 * \arg nusers
 *  -1 for not modifying the original number of users
 *  0 for removing them,
 *  any positive number: the existing users will be cloned
 */
inline bool create_ppl
(people_msgs::People & ppl,
 const int nusers,
 const std::string & filename_prefix = IMG_DIR "depth/juggling1")
{
  // read user, depth, rgb files
  Images2PPL file;
  if (!file.convert(filename_prefix))
    return false;
  ppl = file.get_ppl();
  if (nusers > 0 && ppl.people.size() == 0) {
    printf("pplm_testing: No user found in '%s'\n", filename_prefix.c_str());
    return false;
  }
  if (nusers >= 0) {
    while ((int) ppl.people.size() > nusers) // remove users if too big
      ppl.people.pop_back();
    while ((int) ppl.people.size() < nusers) // clone last user if too small
      ppl.people.push_back(ppl.people.back());
  }
  // set foo coordinates
  for (int user_idx = 0; user_idx < nusers; ++user_idx) {
    ppl.people[user_idx].position.position.x = cos(user_idx*2*M_PI/nusers);
    ppl.people[user_idx].position.position.y = sin(user_idx*2*M_PI/nusers);
    ppl.people[user_idx].position.position.z = 1;
  } // end for (user_idx)
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void test_sizes(PPLMatcherTemplate & skill,
                const std::string & filename_prefix = IMG_DIR "depth/juggling1") {
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  skill.start();

  people_recognition_vision::MatchPPLRequest req;
  people_recognition_vision::MatchPPLResponse res;
  ros::ServiceClient client = nh_public.serviceClient<people_recognition_vision::MatchPPL>
                              (skill.get_match_service_name());
  ASSERT_TRUE_TIMEOUT(client.exists(), 1);
  unsigned int ntimes = 10;

  for (unsigned int i = 0; i < ntimes; ++i) {
    unsigned int ntracks = rand() % 10, ncurr_users = rand() % 10;
    if (i == 0)
      ntracks = ncurr_users = 0;
    ASSERT_TRUE(create_ppl(req.tracks, ntracks, filename_prefix));
    ASSERT_TRUE(create_ppl(req.new_ppl, ncurr_users, filename_prefix));
    ASSERT_TRUE(client.call(req, res));
    // check sizes
    ASSERT_TRUE(res.match_success);
    ASSERT_TRUE(res.costs.size() == ntracks * ncurr_users)
        << "Costs exp size:" << ntracks * ncurr_users
        << ", real size:" << res.costs.size();
  } // end for (i)
  skill.stop();
} // end test_sizes();

////////////////////////////////////////////////////////////////////////////////

void test_same_msg(PPLMatcherTemplate & skill,
                   unsigned int nusers,
                   double max_cost_self = 1E-2,
                   const std::string & filename_prefix = IMG_DIR "depth/juggling1") {
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  skill.start();

  // prepair request
  people_recognition_vision::MatchPPLRequest req;
  people_recognition_vision::MatchPPLResponse res;
  ros::ServiceClient client = nh_public.serviceClient<people_recognition_vision::MatchPPL>
                              (skill.get_match_service_name());
  ASSERT_TRUE_TIMEOUT(client.exists(), 1);
  ASSERT_TRUE(create_ppl(req.tracks, nusers, filename_prefix));
  ASSERT_TRUE(create_ppl(req.new_ppl, nusers, filename_prefix));

  unsigned int ntimes = 1;
  for (unsigned int i = 0; i < ntimes; ++i) {
    ASSERT_TRUE(client.call(req, res));
    // check sizes
    ASSERT_TRUE(res.match_success);
    ASSERT_TRUE(res.costs.size() == nusers * nusers)
        << "Costs exp size:" << nusers * nusers
        << ", real size:" << res.costs.size();
    for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
      double cost_self = res.costs[user_idx * nusers + user_idx];
      ASSERT_TRUE(fabs(cost_self) < max_cost_self)
          <<"nusers:" << nusers << ", user " << user_idx << ": cost=" << cost_self;
    } // end for (user_idx)
  } // end for (i)
  skill.stop();
} // end test_sizes();

////////////////////////////////////////////////////////////////////////////////

void test_two_frames_matching
(PPLMatcherTemplate & skill,
 const std::string & filename_prefix1 = IMG_DIR "depth/david_arnaud1",
 const std::string & filename_prefix2 = IMG_DIR "depth/david_arnaud2")
{
  ros::NodeHandle nh_public;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  skill.start();

  // prepair request
  people_recognition_vision::MatchPPLRequest req;
  people_recognition_vision::MatchPPLResponse res;
  ros::ServiceClient client = nh_public.serviceClient<people_recognition_vision::MatchPPL>
                              (skill.get_match_service_name());
  ASSERT_TRUE_TIMEOUT(client.exists(), 1);
  ASSERT_TRUE(create_ppl(req.tracks, -1, filename_prefix1));
  ASSERT_TRUE(create_ppl(req.new_ppl, -1, filename_prefix2));

  ASSERT_TRUE(client.call(req, res));
  // check sizes
  ASSERT_TRUE(res.match_success);
  unsigned int ntracks = req.tracks.poses.size(), ncurr_users = req.new_ppl.people.size();
  ASSERT_TRUE(res.costs.size() == ntracks * ncurr_users)
      << "Costs exp size:" << ntracks * ncurr_users
      << ", real size:" << res.costs.size();
  // solve with LAP
  MatchList matches;
  Cost best_cost;
  ASSERT_TRUE(linear_assign_from_cost_vec(res.costs, ncurr_users, ntracks, matches, best_cost));
  clean_assign(matches);
  for (unsigned int match_idx = 0; match_idx < matches.size(); ++match_idx) {
    ASSERT_TRUE(matches[match_idx].first == matches[match_idx].second)
        << "Incorrect match:"
        << matches[match_idx].first << "=>" << matches[match_idx].second;
  } // end loop match_idx
  skill.stop();
} // end test_sizes();

////////////////////////////////////////////////////////////////////////////////

typedef DatabasePlayer::UserIdx UserIdx;

void pplm_benchmark(PPLMatcherTemplate & skill,
                    people_msgs::People & ref_ppl,
                    const std::vector<UserIdx> & ref_indices,
                    people_msgs::People & test_ppl,
                    const std::vector<UserIdx> & exp_test_indices) {
  unsigned int nusers_ref = ref_ppl.people.size();
  unsigned int nusers_test = test_ppl.people.size();
  printf("pplm_benchmark(%i ref users, %i test uers)\n", nusers_ref, nusers_test);
  ASSERT_TRUE(nusers_ref == ref_indices.size());
  ASSERT_TRUE(nusers_test == exp_test_indices.size());

  skill.start();
  for (unsigned int user_test_idx = 0; user_test_idx < nusers_test; ++user_test_idx) {
    if (user_test_idx % 100 == 0)
      printf("pplm_benchmark(already %i/%i tested users)...\n", user_test_idx, nusers_test);
    people_msgs::People curr_ppl;
    curr_ppl.header.stamp = ros::Time::now();
    curr_ppl.people.push_back(test_ppl.people[user_test_idx]);
    set_method(curr_ppl, "pplm_benchmark");
    std::vector<double> costs;
    people_msgs::People
        new_ppl_added_attributes,tracks_added_attributes;
    if (!skill.match(curr_ppl, ref_ppl, costs,
                     new_ppl_added_attributes, tracks_added_attributes)) {
      printf("pplm_benchmark(): failed on test PP #%i\n", user_test_idx);
      continue;
    }
    ASSERT_TRUE(nusers_ref == costs.size());
    // https://stackoverflow.com/questions/182957/position-in-vector-using-stl
    unsigned int min_idx = std::distance(costs.begin(),
                                         std::min_element(costs.begin(), costs.end()));
    UserIdx min_index = ref_indices[min_idx], exp_index = exp_test_indices[user_test_idx];
    printf("exp_index:%i, min_index:%i\n", exp_index, min_index);
  } // end loop user_test_idx
  skill.stop();
} // end pplm_benchmark();

////////////////////////////////////////////////////////////////////////////////

//! version for building the PPL from a DatabasePlayer
void pplm_benchmark(PPLMatcherTemplate & skill,
                    DatabasePlayer & db_player,
                    const unsigned int pp_per_ref_user = 1) {
  people_msgs::People ref_ppl, test_ppl;
  std::map<UserIdx, unsigned int> user2ref_nb;
  std::vector<UserIdx> ref_indices, exp_test_indices;
  Images2PPL ppl_conv;
  db_player.set_repeat_playlist(false); // avoid loops
  unsigned int pp_counter = 0;
  while(db_player.go_to_next_frame()) {
    if (pp_counter++ % 100 == 0)
      printf("pplm_benchmark(): already transformed %i dataset images into PP...\n",
             pp_counter);
    ASSERT_TRUE(db_player.has_rgb() && db_player.has_depth()  && db_player.has_user());
    if (!ppl_conv.convert(db_player.get_bgr(), db_player.get_depth(), db_player.get_user()))
      continue;
    unsigned int npps = ppl_conv.get_ppl().poses.size();
    // add randomly to ref or test
    people_msgs::People* target_ppv = &ref_ppl.people;
    std::vector<UserIdx>* target_indexv = &ref_indices;
    UserIdx curr_user_idx = db_player.get_user_idx();
    if ((user2ref_nb[curr_user_idx]++) >= pp_per_ref_user) {
      target_ppv = &test_ppl.people;
      target_indexv = &exp_test_indices;
    }
    target_ppv->insert(target_ppv->end(),
                       ppl_conv.get_ppl().poses.begin(),
                       ppl_conv.get_ppl().poses.end());
    for (unsigned int i = 0; i < npps; ++i)
      target_indexv->push_back(curr_user_idx);
  } // end while(player.go_to_next_frame())

  pplm_benchmark(skill, ref_ppl, ref_indices, test_ppl, exp_test_indices);
} // end pplm_benchmark();

} // end namespace vision_utils

#endif // PPLM_TESTING_H
