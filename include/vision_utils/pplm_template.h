/*!
  \file        ppl_matcher.h
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

PeopleMatcher, also known under the sweet name of PPLM:

\section Parameters, subscriptions, publications
  None

\section Services
  - \b "~match_ppl"
        [people_recognition_vision/MatchPPL]
        Match a detected PPL against a reference one.
 */
#ifndef PPL_MATCHER_H
#define PPL_MATCHER_H

#include "vision_utils/nano_skill.h"
#include <people_recognition_vision/MatchPPL.h>
#include <ros/service_server.h>
#include "vision_utils/ppl_attributes.h"

#ifndef DEBUG_PRINT
#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   //printf_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)
#endif

namespace vision_utils {


class PPLMatcherTemplate : public NanoSkill {
public:
  typedef people_msgs::Person PP;
  typedef people_msgs::People PPL;

  PPLMatcherTemplate (const std::string & start_topic,
                      const std::string & stop_topic)
    : NanoSkill(start_topic, stop_topic)
  {
    _srv_name = _nh_private.resolveName("match_ppl");
  }

  //////////////////////////////////////////////////////////////////////////////

  inline std::string get_match_service_name() const {
    return _srv_name;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! to be implemented by children classes.
     You should at least resize \arg costs to |new_ppl| * |tracks|.
     ie new_ppl.people.size() * tracks.poses.size().
     The costs are then obtained by costs[detected_pp_idx][track_idx].
   */
  virtual bool match(const PPL & new_ppl, const PPL & tracks,
                     std::vector<double> & costs,
                     people_msgs::People & new_ppl_added_attributes,
                     people_msgs::People & tracks_added_attributes) = 0;

  //////////////////////////////////////////////////////////////////////////////

protected:

  //////////////////////////////////////////////////////////////////////////////

  //! inherited from NanoSkill, implemented by sons
  inline void create_subscribers_and_publishers() {
    _srv_server = _nh_private.advertiseService
        (_srv_name, &PPLMatcherTemplate::match_cb, this);
  }
  inline void shutdown_subscribers_and_publishers() {
    _srv_server.shutdown();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! converts a vector into an array-like string representation
  inline static std::string costs_vec2string(std::vector<double> & costs,
                                             unsigned int ncurr_users,
                                             unsigned int ntracks) {
    if (costs.size() != ncurr_users * ntracks) {
      printf("PPLMatcherTemplate::costs_vec2string(): "
             "costs of incorrect size %li (should be %i)!\n",
             costs.size(), ncurr_users * ntracks);
      return "";
    }
    std::ostringstream out;
    unsigned int data_idx = 0;
    for (unsigned int curr_idx = 0; curr_idx < ncurr_users; ++curr_idx) {
      for (unsigned int track_idx = 0; track_idx < ntracks; ++track_idx)
        out << std::setw(10) << std::setprecision(4) << costs[data_idx++];
      out << std::endl;
    } // end loop curr_idx
    return out.str();
  } // end costs_vec2string()

private:

  //////////////////////////////////////////////////////////////////////////////

  //! callback for the service
  virtual bool match_cb(people_recognition_vision::MatchPPL::Request& request,
                        people_recognition_vision::MatchPPL::Response& response) {
    unsigned int npps = request.new_ppl.people.size(),
        ntracks = request.tracks.poses.size();
    response.new_ppl_added_attributes.resize(npps);
    response.tracks_added_attributes.resize(ntracks);
    if (request.new_ppl.people.empty()
        || request.tracks.poses.empty()) {
      response.costs.clear();
      response.match_success = true;
      return true;
    }
    bool ok = match(request.new_ppl, request.tracks,
                    response.costs,
                    response.new_ppl_added_attributes,
                    response.tracks_added_attributes);
    response.match_success = ok;
    return ok;
  }

  //////////////////////////////////////////////////////////////////////////////

  std::string _srv_name;
  ros::ServiceServer _srv_server;
}; // end class PPLMatcherTemplate

} // end namespace vision_utils

#endif // PPL_MATCHER_H
