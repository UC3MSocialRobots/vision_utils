/*!
  \file        dgait2ppl.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/6

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

Read a DGait ONI file and
converts it into 3 image channels (rgb, depth, user)
and a ground truth PPL.

\section Parameters
  - \b "~display"
      [bool, default: true]

\section Publications
  - \b "rgb", "depth", "user"
        [sensor_msgs/Image]
        The image

  - \b "ground_truth_ppl"
        [people_msgs::PeoplePoseList]
        The ground truth positions
 */

#include "vision_utils/dgaitdb.h"
//#include "vision_utils/dgaitdb_filename.h"
#include <image_transport/image_transport.h>
// people_msgs
#include "people_utils/images2ppl.h"

typedef people_msgs::PeoplePose PP;
typedef people_msgs::PeoplePoseList PPL;
static const int QUEUE_SIZE = 10;

int dgait2ppl(int argc, char **argv) {
  if (argc < 2) {
    printf("Synopsis: %s [onifile]\n", argv[0]);
    return -1;
  }
  std::string filename = argv[1];
  DGaitDB reader;
  bool repeat = false;
  if (!reader.from_file(filename, repeat))
    return -1;

  // create publishers
  ros::NodeHandle nh_public, nh_private("~");
  image_transport::ImageTransport transport(nh_public);
  image_transport::Publisher
      rgb_pub = transport.advertise("rgb", QUEUE_SIZE),
      depth_pub = transport.advertise("depth", QUEUE_SIZE),
      user_pub = transport.advertise("user", QUEUE_SIZE);
  ros::Publisher ppl_pub = nh_public.advertise<PPL>("ground_truth_ppl", 1);
  // PPL stuff
  cv_bridge::CvImage depth_bridge, user_bridge, rgb_bridge;
  ppl_utils::Images2PPL _ppl_conv;

  printf("dgait2ppl: publishing rgb on '%s', depth on '%s', user on '%s', truth PPL on '%s'\n",
         rgb_pub.getTopic().c_str(), depth_pub.getTopic().c_str(), user_pub.getTopic().c_str(),
         ppl_pub.getTopic().c_str());

  while (ros::ok()) {
    Timer timer;
    if (!reader.go_to_next_frame()) {
      printf("dgait2ppl: couldn't go_to_next_frame()!\n");
      break;
    }
    ROS_INFO_THROTTLE(10, "Time for go_to_next_frame(): %g ms.", timer.getTimeMilliseconds());
    const cv::Mat3b & bgr = reader.get_bgr();
    const cv::Mat1f & depth = reader.get_depth();
    const cv::Mat1b & user = reader.get_user();

    std_msgs::Header curr_header;
    curr_header.frame_id = "openni_rgb_optical_frame";
    curr_header.stamp = ros::Time::now();
    // publish rgb, depth, user
    rgb_bridge.header = depth_bridge.header = user_bridge.header = curr_header;
    rgb_bridge.image = bgr;
    rgb_bridge.encoding = sensor_msgs::image_encodings::BGR8;
    depth_bridge.image = depth;
    depth_bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    user_bridge.image = user;
    user_bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    rgb_pub.publish(rgb_bridge.toImageMsg());
    depth_pub.publish(depth_bridge.toImageMsg());
    user_pub.publish(user_bridge.toImageMsg());

    // transform to PPL
    if (_ppl_conv.convert(bgr, depth, user, NULL, &curr_header)) {
      _ppl_conv.get_ppl().method = "ground_truth";
      ppl_pub.publish(_ppl_conv.get_ppl());
    }

    // display
    // reader.display();
    ros::spinOnce();
  } // end while(ros::ok())
  return 0;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "dgait2ppl");
  return dgait2ppl(argc, argv);
}
