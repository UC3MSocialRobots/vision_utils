/*!
  \file        multi_subscriber.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/4/15

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

A subscriber to multiple topics at once.
Based on http://answers.ros.org/question/56168/subscribing-to-multiple-topics-of-the-same-type/

 */

#ifndef MULTI_SUBSCRIBER_H
#define MULTI_SUBSCRIBER_H

#include <ros/ros.h>
#include "vision_utils/string_split.h"
#include "vision_utils/find_and_replace.h"
#include "vision_utils/string_casts_stl.h"

namespace vision_utils {

/*!
 * \brief The MultiSubscriber class subscribes to a list of topics pointing to
 * the same callback.
 *
 * It checks for repeated topics in the list, and only subscribes once to them.
 */
class MultiSubscriber {
public:

    //! empty ctor
    MultiSubscriber() {
        // nothing to do
    }

    /*!
   * Subscribe to a list of topics, separated by ";".
   * \example topics_list="/foo;/bar"
   * All these topics must have the same type.
   *
   * version for class member function with bare pointer
   * \see ros::NodeHandle::subscribe(),
   *  version for class member function with bare pointer (node_handle.h:340)
   */
    template<class M, class T>
    static MultiSubscriber subscribe(ros::NodeHandle & nh_public,
                                     const std::string& topics_list,
                                     uint32_t queue_size,
                                     void(T::*fp)(M),
                                     T* obj,
                                     const ros::TransportHints& transport_hints = ros::TransportHints())
    {
        MultiSubscriber sub;
        MultiSubscriber::split_topics_list(topics_list, sub._topics);
        //printf("MultiSubscriber: subscribing to topic list:'%s'\n", sub.getTopics(false).c_str());
        sub._subs.clear();
        for (unsigned int topic_idx = 0; topic_idx < sub._topics.size(); ++topic_idx)
            sub._subs.push_back(nh_public.subscribe
                                (sub._topics[topic_idx], queue_size, fp, obj, transport_hints));
        return sub;
    }

    //////////////////////////////////////////////////////////////////////////////

    /*!
   * Subscribe to a list of topics, separated by ";".
   * \example topics_list="/foo;/bar"
   * All these topics must have the same type.
   *
   * version for bare function
   * \see ros::NodeHandle::subscribe(),
   *  version for bare function (node_handle.h:580)
   */
    template<class M>
    static MultiSubscriber subscribe(ros::NodeHandle & nh_public,
                                     const std::string& topics_list,
                                     uint32_t queue_size,
                                     void(*fp)(M),
                                     const ros::TransportHints& transport_hints = ros::TransportHints())
    {
        MultiSubscriber sub;
        MultiSubscriber::split_topics_list(topics_list, sub._topics);
        printf("MultiSubscriber: subscribing to topic list:'%s'\n", sub.getTopics(false).c_str());
        sub._subs.clear();
        for (unsigned int topic_idx = 0; topic_idx < sub._topics.size(); ++topic_idx)
            sub._subs.push_back(nh_public.subscribe
                                (sub._topics[topic_idx], queue_size, fp, transport_hints));
        return sub;
    }

    //////////////////////////////////////////////////////////////////////////////

    /*! get a list of the subscribed topics
   * \param resolved
   *    if false, return the raw list of topics as provided
   *    if true, return the resolved topics (full names with namespaces)
   * \see ros::Subscriber::getTopic()
   */
    inline std::string getTopics(bool resolved = true) const {
        if (!resolved)
            return accessible_to_string(_topics);
        std::ostringstream topics_str;
        topics_str << "[";
        for (unsigned int topic_idx = 0; topic_idx < _subs.size(); ++topic_idx)
            topics_str << _subs[topic_idx].getTopic()
                       <<  (topic_idx < _subs.size() - 1 ? "; " : "");
        topics_str << "]";
        return topics_str.str();
    }

    //////////////////////////////////////////////////////////////////////////////

    //! \return the number of subscribed topics
    inline unsigned int nTopics() const {
        return _subs.size();
    }

    //////////////////////////////////////////////////////////////////////////////

    //! \return the number of publishers available for all topics
    inline unsigned int getNumPublishers() const {
        unsigned int sum = 0;
        for (unsigned int topic_idx = 0; topic_idx < _subs.size(); ++topic_idx)
            sum += _subs[topic_idx].getNumPublishers();
        return sum;
    }

    //////////////////////////////////////////////////////////////////////////////

    //! shutdown all subscribers
    inline void strip_non_connected() {
        for (unsigned int topic_idx = 0; topic_idx < _subs.size(); ++topic_idx) {
            if (_subs[topic_idx].getNumPublishers() > 0)
                continue;
            printf("Stripping non-connected subscriber '%s'\n",
                     _subs[topic_idx].getTopic().c_str());
            _subs.erase(_subs.begin() + topic_idx);
            --topic_idx;
        }
    }

    //////////////////////////////////////////////////////////////////////////////

    //! shutdown all subscribers
    inline void shutdown() {
        for (unsigned int topic_idx = 0; topic_idx < _subs.size(); ++topic_idx)
            _subs[topic_idx].shutdown();
    }

    //////////////////////////////////////////////////////////////////////////////

    /*! split a concatenated list of topics into \a _topics
   *  remove repeated topics
   */
    inline static void split_topics_list(const std::string & topics_list,
                                         std::vector<std::string> & topics) {
        // first just split, keeping repetitions
        std::vector<std::string> topics_with_repetitions;
        StringSplit(topics_list, ";", &topics_with_repetitions);
        // remove spaces, in case of
        for (unsigned int topic_idx = 0; topic_idx < topics_with_repetitions.size(); ++topic_idx)
            find_and_replace(topics_with_repetitions[topic_idx], " ", "");
        // convert it into a set for removing repetiions
        std::set<std::string> topics_set;
        topics_set.insert(topics_with_repetitions.begin(), topics_with_repetitions.end());
        // remove empty strings from topics_set
        topics_set.erase("");
        topics_set.erase(" ");
        // copy the set, which is without repetitions, into the vector "_topics"
        // from http://stackoverflow.com/questions/5034211/c-copy-set-to-vector
        topics.clear();
        std::copy(topics_set.begin(), topics_set.end(), std::back_inserter(topics));
    } // end split_topics_list();

private:
    //! containing the list of topics
    std::vector<std::string> _topics;
    //! containing the list of subscribers
    std::vector<ros::Subscriber> _subs;
}; // end class MultiSubscriber

} // end namespace vision_utils

#endif // MULTI_SUBSCRIBER_H
