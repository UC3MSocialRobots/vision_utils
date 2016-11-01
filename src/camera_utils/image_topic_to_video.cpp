/*!
  \file        input_image_topic_to_video.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/4/16

  ______________________________________________________________________________

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
  ______________________________________________________________________________

  A node to record an image node into a video.

\section Parameters
  - \b fps
    [double] (default: 10)
    The frame rate of the video
  - \b input_image_topic
    [string] (default: "/input_image_topic")
    The topic to listen to.
  - \b output_video_filename
    [string] (default: "<timestamp>_<input_image_topic>.avi")
    The filename where to save the images.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// AD

cv_bridge::CvImageConstPtr _bridge_img_ptr;
cv::VideoWriter _video_writer;
std::string _input_image_topic = "input_image_topic";
std::string _output_video_filename;
double _fps = 10;
int _codec =
    //CV_FOURCC('D', 'I', 'V', 'X'); // MPEG-4 codec
//CV_FOURCC('P','I','M','1') ; // MPEG-1 codec
CV_FOURCC('M','J','P','G') ; // motion-jpeg codec (does not work well)
//CV_FOURCC('M', 'P', '4', '2'); // MPEG-4.2 codec
//CV_FOURCC('D', 'I', 'V', '3'); // MPEG-4.3 codec
//CV_FOURCC('U', '2', '6', '3'); // H263 codec
//CV_FOURCC('I', '2', '6', '3'); // H263I codec
//CV_FOURCC('F', 'L', 'V', '1'); // avi1 codec
int _frames_nb = 0;

////////////////////////////////////////////////////////////////////////////////

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
//  ROS_INFO_THROTTLE(3, "'%s' writer: image_callback()",
//                    _input_image_topic.c_str());
  //Timer timer;
  try {
    _bridge_img_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // init the writer if needed
  if (!_video_writer.isOpened()) {
    _video_writer.open(_output_video_filename,
                       _codec,
                       _fps,
                       _bridge_img_ptr->image.size(),
                       true);
  } // end if (!writer.isOpened())

  // append the image
  _video_writer << _bridge_img_ptr->image;
  ++_frames_nb;

  ROS_INFO_THROTTLE(3, "'%s' writer: %i frames (%f seconds)",
                    _input_image_topic.c_str(),
                    _frames_nb, 1.f * _frames_nb / _fps);
} // end image_callback();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_topic_to_video");
  ros::NodeHandle nh_public, nh_private("~");
  // get params
  nh_private.param("fps", _fps, _fps);
  nh_private.param("input_image_topic", _input_image_topic, _input_image_topic);
  // make a default filename for the output video,
  // type "<timestamp>_<input_image_topic>.avi"
  std::ostringstream output_video_filename_stream;
  std::string _input_image_topic_clean = _input_image_topic;
  vision_utils::find_and_replace(_input_image_topic_clean, "/", "_");
  output_video_filename_stream << vision_utils::timestamp() << "_"
                               << _input_image_topic_clean << ".avi";
  // now get the param
  nh_private.param("output_video_filename", _output_video_filename,
           output_video_filename_stream.str());

  ROS_WARN("input_image_topic_to_video: listening to topic '%s', "
           "writing to file '%s'",
           _input_image_topic.c_str(), _output_video_filename.c_str());

  // subscribe to image topic
  image_transport::ImageTransport transport(nh_public);
  image_transport::Subscriber sub = transport.subscribe
      (_input_image_topic, 1, image_callback);

  // spin!
  ros::spin();
  return 0;
}

