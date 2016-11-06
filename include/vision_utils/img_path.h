#ifndef IMG_PATH_H_
#define IMG_PATH_H_

#include <ros/package.h>

inline  std::string IMG_DIR { return ros::package::getPath("vision_utils/")
                                      + "/data/images/"; }

#endif
