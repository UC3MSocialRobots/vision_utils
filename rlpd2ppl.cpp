/*!
  \file        rlpd2ppl.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/11/20

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


Read a rlpd ONI file and
converts it into 3 image channels (rgb, depth, user)
and a ground truth PPL.

\section Parameters
  - \b "~display"
      [bool, default: true]

  - \b "~rate"
      [int, Hz, default: 1]

\section Publications
  - \b "rgb", "depth", "user"
        [sensor_msgs/Image]
        The image

  - \b "ground_truth_ppl"
        [people_msgs::PeoplePoseList]
        The ground truth positions
 */

#include <databases_io/rlpd2imgs.h>
#include <image_transport/image_transport.h>
// people_msgs
#include <ppl_utils/images2ppl.h>
#include <time/timer.h>

typedef people_msgs::PeoplePose PP;
typedef people_msgs::PeoplePoseList PPL;
static const int QUEUE_SIZE = 10;

int rlpd2ppl(int argc, char **argv) {
  // get input files
  if (argc < 2) {
    printf("Synopsis: %s [RGB files]\n", argv[0]);
    return -1;
  }
  std::ostringstream files;
  for (int argi = 1; argi < argc; ++argi) // 0 is the name of the exe
    files << argv[argi]<< ";";
  RLPD2Imgs reader;
  bool repeat = false;
  if (!reader.from_file(files.str(), repeat))
    return -1;

  // get params
  ros::NodeHandle nh_public, nh_private("~");
  bool display = false;
  nh_private.param("display", display, display);
  double rate_hz = 1;
  nh_private.param("rate", rate_hz, rate_hz);
  ros::Rate rate(rate_hz);

  // create publishers
  image_transport::ImageTransport transport(nh_public);
  image_transport::Publisher
      rgb_pub = transport.advertise("rgb", QUEUE_SIZE),
      depth_pub = transport.advertise("depth", QUEUE_SIZE),
      user_pub = transport.advertise("user", QUEUE_SIZE);
  ros::Publisher ground_truth_ppl_pub = nh_public.advertise<PPL>("ground_truth_ppl", 1);
  // PPL stuff
  cv_bridge::CvImage depth_bridge, user_bridge, rgb_bridge;
  ppl_utils::Images2PPL _ground_truth_ppl_conv;

  printf("rlpd2ppl: publishing rgb on '%s', depth on '%s', user on '%s', "
         "ground truth PPL on '%s', every %g s\n",
         rgb_pub.getTopic().c_str(), depth_pub.getTopic().c_str(), user_pub.getTopic().c_str(),
         ground_truth_ppl_pub.getTopic().c_str(),
         rate.expectedCycleTime().toSec());

  while (ros::ok()) {
    Timer timer;
    if (!reader.go_to_next_frame()) {
      printf("rlpd2ppl: couldn't go_to_next_frame()!\n");
      break;
    }
    ROS_INFO_THROTTLE(10, "Time for go_to_next_frame(): %g ms.", timer.getTimeMilliseconds());
    const cv::Mat3b & bgr = reader.get_bgr();
    const cv::Mat1f & depth = reader.get_depth();
    const cv::Mat1b & user = reader.get_user(),
        user_ground_truth = reader.get_ground_truth_user();

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
    if (_ground_truth_ppl_conv.convert(bgr, depth, user_ground_truth, NULL, &curr_header)) {
      _ground_truth_ppl_conv.get_ppl().method = "ground_truth";
      ground_truth_ppl_pub.publish(_ground_truth_ppl_conv.get_ppl());
    }

    // display
    if (display)
      reader.display();
    ros::spinOnce();
    rate.sleep();
  } // end while(ros::ok())
  return 0;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "rlpd2ppl");
  return rlpd2ppl(argc, argv);
}
