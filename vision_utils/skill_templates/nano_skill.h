/*!
  \file        nano_skill.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/15

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
  None.

\section Subscriptions
  - \b {start_topic}
        [std_msgs/Int16.]
        The skill is started when a message is received on that topic.

  - \b {stop_topic}
        [std_msgs/Int16.]
        The skill is stopped when a message is received on that topic.


\section Publications
  None
 */

#ifndef NANO_SKILL_H
#define NANO_SKILL_H

// msg
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <std_msgs/Int16.h>

class NanoSkill {
public:
  enum Status {
    STOPPED = 0,
    RUNNING = 1
  };

  NanoSkill(const std::string & start_topic,
            const std::string & stop_topic) :
    _start_topic(start_topic),
    _stop_topic(stop_topic),
    _nh_private("~"),
    _status(STOPPED)
  {
    _start_sub = _nh_public.subscribe(_start_topic, 1, &NanoSkill::start_cb, this);
    _stop_sub = _nh_public.subscribe(_stop_topic, 1, &NanoSkill::stop_cb, this);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual ~NanoSkill() {
    // stop if needed - hum, generate segfault
    // std_msgs::Int16 foo;
    // stop_cb(std_msgs::Int16ConstPtr(&foo));
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual inline void start() {
    if (_status == RUNNING) {
      printf("NanoSkill already running!\n");
      return;
    }
    _status = RUNNING;
    create_subscribers_and_publishers();
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual inline void stop() {
    if (_status == STOPPED) {
      printf("NanoSkill already stopped!\n");
      return;
    }
    _status = STOPPED;
    shutdown_subscribers_and_publishers();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool is_running() const {
    return (_status == RUNNING);
  }
  //////////////////////////////////////////////////////////////////////////////

  /*! is called when the skill is activated.
      Should create any ROS publisher/subscriber that is needed.
      The subscribers needed for the activation/desactivation events
      are automatically handled.
  */
  virtual void create_subscribers_and_publishers() = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! is called when the skill is desactivated.
      Should shutdown all subs/pubs created in create_subscribers_and_publishers().
      The subscribers needed for the activation/desactivation events
      are automatically handled.
 */
  virtual void shutdown_subscribers_and_publishers() = 0;

  //////////////////////////////////////////////////////////////////////////////

  inline std::string get_start_stopic() const { return _start_sub.getTopic(); }
  inline std::string get_stop_stopic()  const { return _stop_sub.getTopic(); }

protected:

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std::string _start_topic, _stop_topic;
  ros::NodeHandle _nh_public, _nh_private;

  //////////////////////////////////////////////////////////////////////////////

  inline void start_cb(const std_msgs::Int16ConstPtr &) { start(); }
  inline void stop_cb (const std_msgs::Int16ConstPtr &) { stop();  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  Status _status;
  ros::Subscriber _start_sub, _stop_sub;
}; // end class NanoSkill

#endif // NANO_SKILL_H
