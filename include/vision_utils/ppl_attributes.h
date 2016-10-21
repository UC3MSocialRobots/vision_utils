/*!
  \file        ppl_attributes.h
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

#ifndef PPL_ATTRIBUTES_H
#define PPL_ATTRIBUTES_H

// msg
#include <people_msgs_rl/PeoplePoseList.h>
// AD
#include "vision_utils/utils/string_casts.h"

namespace ppl_utils {

//! \return false if the number of attribute names and values are different
bool check_attributes_not_corrupted(const people_msgs_rl::PeoplePoseAttributes & attrs) {
  if (attrs.names.size() == attrs.values.size())
    return true;
  printf("Pose attributes corrupted, %li names, %li values!\n",
         attrs.names.size(), attrs.values.size());
  return false;
}

//! \return false if the number of attribute names and values are different
bool check_attributes_not_corrupted(const people_msgs_rl::PeoplePose & pose) {
  return check_attributes_not_corrupted(pose.attributes);
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
bool set_attribute(people_msgs_rl::PeoplePoseAttributes & attrs,
                   const std::string & attr_name,
                   const T & attr_value) {
  if (!check_attributes_not_corrupted(attrs))
    return false;
  unsigned int nattrs = attrs.names.size();
  // search the attribute name
  std::string attr_value_str = string_utils::cast_to_string(attr_value);
  for (unsigned int attr_idx = 0; attr_idx < nattrs; ++attr_idx) {
    if (attrs.names[attr_idx] == attr_name) {
      attrs.values[attr_idx] = attr_value_str;
      return true; // success
    }
  } // end loop attr_idx
  // insert at end
  attrs.names.push_back(attr_name);
  attrs.values.push_back(attr_value_str);
  return true; // success
} // end set_attribute()

////////////////////////////////////////////////////////////////////////////////

template<class T>
bool set_attribute(people_msgs_rl::PeoplePose & pose,
                   const std::string & attr_name, const T & attr_value) {
  return set_attribute<T>(pose.attributes, attr_name, attr_value);
}

////////////////////////////////////////////////////////////////////////////////

//! \return true if the pose has a valid attribute with name \arg attr_name
bool has_attribute(const people_msgs_rl::PeoplePose & pose,
                   const std::string & attr_name) {
  if (!check_attributes_not_corrupted(pose))
    return false;
  return (std::find(pose.attributes.names.begin(),
                    pose.attributes.names.end(),
                    attr_name) != pose.attributes.names.end());
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
bool get_attribute_readonly(const people_msgs_rl::PeoplePose & pose,
                            const std::string & attr_name,
                            T & attr_value) {
  if (!check_attributes_not_corrupted(pose))
    return false;
  unsigned int nattrs = pose.attributes.names.size();
  // search the attribute name
  for (unsigned int attr_idx = 0; attr_idx < nattrs; ++attr_idx) {
    if (pose.attributes.names[attr_idx] != attr_name)
      continue;
    bool success;
    attr_value = string_utils::cast_from_string<T>(pose.attributes.values[attr_idx], success);
    return success;
  } // end loop attr_idx
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool copy_attributes(const people_msgs_rl::PeoplePoseAttributes & src,
                     people_msgs_rl::PeoplePoseAttributes & dst) {
  if (!check_attributes_not_corrupted(src) || !check_attributes_not_corrupted(dst))
    return false;
  // copy all other attributes
  unsigned int nattributes = std::min(src.names.size(),
                                      src.values.size());
  for (unsigned int attr_idx = 0; attr_idx < nattributes; ++attr_idx) {
    if (!ppl_utils::set_attribute(dst, src.names[attr_idx],
                                  src.values[attr_idx]))
      return false;
  } // end for (attr_idx)
  return true;
} // end copy_attributes()

////////////////////////////////////////////////////////////////////////////////

bool copy_attributes(const people_msgs_rl::PeoplePose & pp_src,
                     people_msgs_rl::PeoplePose & pp_dst) {
  return copy_attributes(pp_src.attributes, pp_dst.attributes);
} // end copy_attributes()

bool copy_attributes(const std::vector<people_msgs_rl::PeoplePoseAttributes> & attrs_src,
                     people_msgs_rl::PeoplePoseList & ppl_dst) {
  unsigned int nppls = attrs_src.size();
  if (ppl_dst.poses.size() != nppls) {
    printf("copy_attributes(): cannot copy, attributes vector of size %i,"
           "dest PPL of size %li\n", nppls, ppl_dst.poses.size());
  }
  for (unsigned int i = 0; i < nppls; ++i) {
    if (!copy_attributes(attrs_src[i], ppl_dst.poses[i].attributes))
      return false;
  } // end for (i)
  return true;
} // end copy_attributes()

} // end namespace ppl_utils

#endif // PPL_ATTRIBUTES_H
