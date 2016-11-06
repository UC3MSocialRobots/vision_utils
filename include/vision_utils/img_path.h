#ifndef IMG_PATH_H_
#define IMG_PATH_H_

#include <ros/package.h>

namespace vision_utils {
inline std::string IMG_DIR() {
  return ros::package::getPath("vision_utils/") + "/data/images/";
}
} // end namespace vision_utils

#endif
