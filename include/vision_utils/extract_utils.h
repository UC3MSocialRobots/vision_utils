#ifndef EXTRACT_UTILS_H
#define EXTRACT_UTILS_H

#include <string>

namespace vision_utils {

inline std::string extract(const std::string & in,
                           const unsigned int max_length = 50) {
  if (in.size() < max_length)
    return in;
  return in.substr(0, max_length) + std::string("…");
}

} // end namespace vision_utils

#endif // EXTRACT_UTILS_H
