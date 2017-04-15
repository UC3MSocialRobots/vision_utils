#ifndef REPLACE_FIND_TAGS_H
#define REPLACE_FIND_TAGS_H

#include <string>
#include <ros/package.h>

namespace vision_utils {

inline std::string replace_find_tags(const std::string & path) {
  std::string out = path, pattern_begin = "$(find ", pattern_end = ")";
  size_t begin = 0, end = 0, pbs = pattern_begin.size();
  while(true) {
    begin = out.find(pattern_begin, begin);
    end   = out.find(pattern_end, begin);
    if (begin == std::string::npos || end == std::string::npos)
      return out;
    std::string pkgname = out.substr(begin+pbs, end-begin-pbs);
    //printf("pkgname:'%s'\n", pkgname.c_str());
    std::string pkgpath = ros::package::getPath(pkgname);
    out.replace(begin, end-begin+1, pkgpath);
    begin += pkgpath.length();
  }
}

} // end namespace vision_utils

#endif // REPLACE_FIND_TAGS_H
