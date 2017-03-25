#ifndef ALL_SUBFOLDERS_H
#define ALL_SUBFOLDERS_H

// C++
#include <string>
#include <boost/filesystem.hpp>

namespace vision_utils {

// https://stackoverflow.com/questions/5043403/listing-only-folders-in-directory
inline int all_subfolders(const std::string & dir_path,
                          std::vector<std::string> & ans) {
  ans.clear();
  if (!boost::filesystem::exists( dir_path ))
    return -1;
  boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
  for ( boost::filesystem::directory_iterator itr( dir_path ); itr != end_itr; ++itr ) {
    if (!boost::filesystem::is_directory(itr->status()))
      continue;
    ans.push_back(boost::filesystem::basename(itr->path()));
  }
  std::sort(ans.begin(), ans.end()); // alphabetical sort
  return ans.size();
} // end all_subfolders()

} // end namespace vision_utils

#endif // ALL_SUBFOLDERS_H
