/*!
  \file        ppl_tags.h
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

\todo Description of the file
 */

#ifndef PPL_tagS_H
#define PPL_tagS_H

// msg
#include <people_msgs/People.h>
// AD
#include "vision_utils/string_casts.h"

namespace vision_utils {

//! \return false if the number of tag names and values are different
bool check_tags_not_corrupted(const people_msgs::Person & p) {
  if (p.tagnames.size() == p.tags.size())
    return true;
  printf("Pose tags corrupted, %li names, %li values!\n",
         p.tagnames.size(), p.tags.size());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
bool set_tag(people_msgs::Person & p,
             const std::string & tag_name,
             const T & tag_value) {
  if (!check_tags_not_corrupted(p))
    return false;
  unsigned int ntags = p.tagnames.size();
  // search the tag name
  std::string tag_value_str = cast_to_string(tag_value);
  for (unsigned int tag_idx = 0; tag_idx < ntags; ++tag_idx) {
    if (p.tagnames[tag_idx] == tag_name) {
      p.tags[tag_idx] = tag_value_str;
      return true; // success
    }
  } // end loop tag_idx
  // insert at end
  p.tagnames.push_back(tag_name);
  p.tags.push_back(tag_value_str);
  return true; // success
} // end set_tag()

////////////////////////////////////////////////////////////////////////////////

template<class T>
bool set_tag_people(people_msgs::People & ppl,
                    const std::string & tag_name,
                    const T & tag_value) {
  bool ok = true;
  unsigned int nppl = ppl.people.size();
  for (unsigned int i = 0; i < nppl; ++i) {
    ok = ok && set_tag(ppl.people[i], tag_name, tag_value);
  }
  return true; // success
} // end set_tag()

////////////////////////////////////////////////////////////////////////////////

//! \return true if the pose has a valid tag with name \arg tag_name
bool has_tag(const people_msgs::Person & p,
             const std::string & tag_name) {
  if (!check_tags_not_corrupted(p))
    return false;
  return (std::find(p.tagnames.begin(),
                    p.tagnames.end(),
                    tag_name) != p.tagnames.end());
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
bool get_tag(const people_msgs::Person & p,
             const std::string & tag_name,
             T & tag_value) {
  if (!check_tags_not_corrupted(p))
    return false;
  unsigned int ntags = p.tagnames.size();
  // search the tag name
  for (unsigned int tag_idx = 0; tag_idx < ntags; ++tag_idx) {
    if (p.tagnames[tag_idx] != tag_name)
      continue;
    bool success;
    tag_value = cast_from_string<T>(p.tags[tag_idx], success);
    return success;
  } // end loop tag_idx
  return false;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
T get_tag_default(const people_msgs::Person & p,
                  const std::string & tag_name,
                  const T & default_value) {
  T ans = default_value;
  return (get_tag(p, tag_name, ans) ? ans : default_value);
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
bool get_tag_people(const people_msgs::People & ppl,
                    const std::string & tag_name,
                    T & tag_value) {
  unsigned int nppl = ppl.people.size();
  for (unsigned int i = 0; i < nppl; ++i) {
    if (get_tag(ppl.people[i], tag_name, tag_value))
      return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
T get_tag_people_default(const people_msgs::People & ppl,
                         const std::string & tag_name,
                         const T & default_value) {
  T ans = default_value;
  unsigned int nppl = ppl.people.size();
  for (unsigned int i = 0; i < nppl; ++i) {
    if (get_tag(ppl.people[i], tag_name, ans))
      return ans;
  }
  return default_value;
}

////////////////////////////////////////////////////////////////////////////////
// special functions for the "method" tag

inline bool set_method(people_msgs::Person & pp,
                       const std::string & method) {
  return set_tag(pp, "method", method);
}
inline bool set_method(people_msgs::People & ppl,
                       const std::string & method) {
  return set_tag_people(ppl, "method", method);
}
inline std::string get_method(const people_msgs::Person & pp) {
  return get_tag_default(pp, "method", std::string());
}
inline std::string get_method(const people_msgs::People & ppl) {
  return get_tag_people_default(ppl, "method", std::string());
}

////////////////////////////////////////////////////////////////////////////////

bool copy_tags(const people_msgs::Person & src,
               people_msgs::Person & dst) {
  if (!check_tags_not_corrupted(src) || !check_tags_not_corrupted(dst))
    return false;
  // copy all other tags
  unsigned int ntags = std::min(src.tagnames.size(),
                                src.tags.size());
  for (unsigned int tag_idx = 0; tag_idx < ntags; ++tag_idx) {
    if (!set_tag(dst, src.tagnames[tag_idx],
                               src.tags[tag_idx]))
      return false;
  } // end for (tag_idx)
  return true;
} // end copy_tags()

////////////////////////////////////////////////////////////////////////////////

bool copy_tags(const people_msgs::People & ppl_src,
               people_msgs::People & ppl_dst) {
  unsigned int nppls = ppl_src.people.size();
  if (ppl_dst.people.size() != nppls) {
    printf("copy_tags(): cannot copy, tags vector of size %i,"
           "dest PPL of size %li\n", nppls, ppl_dst.people.size());
  }
  for (unsigned int i = 0; i < nppls; ++i) {
    if (!copy_tags(ppl_src.people[i], ppl_dst.people[i]))
      return false;
  } // end for (i)
  return true;
} // end copy_tags()

} // end namespace vision_utils

#endif // PPL_tagS_H
