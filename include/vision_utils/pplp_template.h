/*!
  \file        pplp_template.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/20

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

PeoplePoseListPublisher, also known under the sweet name of PPLP:
A template for a skill
that can be activated on demand and
that can publish PeoplePoseLists.

Its subscribers are inactive when the skill is constructed.
They are initalized
when receiving a message on a given start topic
and shutdown when receiving a message on a given stop topic.

Both start and stop topics are parameters given to the constructor of the class.

\section Parameters
  None.

\section Subscriptions
  - \b {start_topic}
        [std_msgs/Int16.]
        The skill is started when a message is received on that topic.

  - \b {stop_topic}
        [std_msgs/Int16.]
        The skill is stopped when a message is received on that topic.

\section Publications
  - \b "~ppl"
        [people_msgs_rl::PeoplePoseList]
        The detected users in the mask
 */
#ifndef TEMPLATE2PPL_H
#define TEMPLATE2PPL_H

// msg
#include "vision_utils/nano_skill.h"
#include <people_msgs_rl/PeoplePoseList.h>
#include <std_msgs/Int16.h>

#ifndef DEBUG_PRINT
#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   ROS_WARN(__VA_ARGS__)
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)
#endif

class PPLPublisherTemplate : public NanoSkill {
public:
  typedef people_msgs_rl::PeoplePoseList PPL;

  PPLPublisherTemplate(const std::string & start_topic,
                       const std::string & stop_topic) :
    NanoSkill(start_topic, stop_topic),
    _ppl_sent_nb(0)
  {
    _ppl_resolved_topic = _nh_private.resolveName("ppl");
    // TODO change advertise in create_subscribers_and_publishers()
    _ppl_pub = _nh_public.advertise<PPL>(_ppl_resolved_topic, 1);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual ~PPLPublisherTemplate() {
    // stop if needed - hum, generate segfault
    // std_msgs::Int16 foo;
    // stop_cb(std_msgs::Int16ConstPtr(&foo));
  }

  //////////////////////////////////////////////////////////////////////////////

  // inherited from NanoSkill: should be implemented by sons
  // virtual void create_subscribers_and_publishers() = 0;
  // virtual void shutdown_subscribers_and_publishers() = 0;

  //////////////////////////////////////////////////////////////////////////////

  inline void publish_PPL(const PPL & ppl) {
    if (!is_running() || _ppl_pub.getTopic() == "") {
      printf("PPLPublisherTemplate: cant publish ppl before calling start()!\n");
      return;
    }
    _last_ppl = ppl;
    _ppl_pub.publish(_last_ppl);
    ++_ppl_sent_nb;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int get_ppl_published_nb() const { return _ppl_sent_nb; }
  inline unsigned int get_ppl_num_subscribers() const { return _ppl_pub.getNumSubscribers(); }
  inline std::string get_ppl_topic()  const { return _ppl_resolved_topic; }
  inline const PPL & get_last_PPL()  const { return _last_ppl; }

private:

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  PPL _last_ppl;
  std::string _ppl_resolved_topic;
  ros::Publisher _ppl_pub;
  unsigned int _ppl_sent_nb;
}; // end class PPLPublisherTemplate

#endif // TEMPLATE2PPL_H
