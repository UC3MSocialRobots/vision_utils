/*!
  \file        test_kinect_openni_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/6

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

Some tests for the utils with Kinect images.
 */


#include "vision_utils/timer.h"
// opencv
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>

/*!
  A simple example for transforming the RGB stream of a camera
  into a ROS visualization_msgs::Marker
 \param capture
    the access to the camera stream
 \param calibration_filename
    the file containing the intrinsic and extrinsic parameters of the camera
*/
//#if ( ROS_VERSION_MINIMUM(1, 7, 0) ) // fuerte code
#if 0




void reproject_image_ros(const std::string & rgb_depth_filename_prefix,
                         const std::string & kinect_serial_number) {
  int data_step = 3;
  float pt_size = .01 * data_step;

  int argc = 0;
  ros::init(argc, NULL, "reproject_image_ros");
  vision_utils::Timer timer;

  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  vision_utils::read_camera_model_files
      (kinect_serial_number, depth_camera_model, rgb_camera_model);

  // read depth and rgb files
  cv::Mat rgb_img, depth_img;
  vision_utils::read_rgb_and_depth_image_from_image_file
      (rgb_depth_filename_prefix, &rgb_img, &depth_img);
  timer.printTime("read_rgb_and_depth_image_from_image_file()");

  // kinect viewer
  std::vector<cv::Point3f> pointcloud;
  std::vector<cv::Vec3b> pointcloud_RGB;
  vision_utils::pixel2world_rgb_color255
      (rgb_img, depth_img, depth_camera_model,
       pointcloud, pointcloud_RGB,
       data_step, cv::Mat(), true);
  //vision_utils::view_rgb_cloud(pointcloud, pointcloud_RGB);

  // create markers
  ros::NodeHandle nh_public;
  visualization_msgs::Marker marker;
  vision_utils::make_header(marker, visualization_msgs::Marker::POINTS,
                            "reproject_image_ros", pt_size, 1, 0, 0, 1,
                            rgb_camera_model.tfFrame());
  ros::Publisher marker_pub =
      nh_public.advertise<visualization_msgs::Marker>("reproject_image_ros_marker", 1);
  ROS_INFO("Publishing markers on '%s'", marker_pub.getTopic().c_str());

  // reproject depth and colors
  vision_utils::pixel2world_rgb_color_rgba
      (rgb_img, depth_img, depth_camera_model,
       marker.points, marker.colors,
       data_step, cv::Mat(), true);
  printf("%i 3D points, %i colors\n",
         marker.points.size(), marker.colors.size());
  cv::imshow("rgb_img", rgb_img);
  cv::imshow("depth_img_illus", vision_utils::depth_image_to_vizualisation_color_image(depth_img));

  // publish marker periodically
  while (true) {
    // printf("acquiring image...\n");
    marker.header.stamp = ros::Time::now();
    marker_pub.publish(marker);
    ros::spinOnce();

    // show resulting images
    char c = cv::waitKey(15);
    // printf("c:%i\n", (int) c);
    if ((int) c == 27)
      break;
  }
} // end reproject_image_ros();
#endif

////////////////////////////////////////////////////////////////////////////////

bool are_camera_infos_equal(const sensor_msgs::CameraInfo & i1,
                            const sensor_msgs::CameraInfo & i2) {
  return (i1.width == i2.width) &&
      (i1.height == i2.height) &&
      (i1.distortion_model == i2.distortion_model) &&
      (i1.D == i2.D) &&
      (i1.K == i2.K) &&
      (i1.R == i2.R) &&
      (i1.P == i2.P);
}

void camera_info_bag_to_binary_file(const std::string & kinect_serial_number) {
  sensor_msgs::CameraInfo depth_camera_info, rgb_camera_info;
  vision_utils::read_camera_info_bag_files
      (kinect_serial_number, depth_camera_info, rgb_camera_info);
  sensor_msgs::CameraInfo depth_camera_info2, rgb_camera_info2;

  std::string depth_str = vision_utils::ros_object_to_string(depth_camera_info);
  vision_utils::ros_object_from_string(depth_str, depth_camera_info2);

  for (unsigned int i = 0; i < 100; ++i) {
    vision_utils::ros_object_to_file("depth_camera_info.dat", depth_camera_info);
    vision_utils::ros_object_to_file("rgb_camera_info.dat", rgb_camera_info);
    vision_utils::ros_object_from_file("depth_camera_info.dat", depth_camera_info2);
    vision_utils::ros_object_from_file("rgb_camera_info.dat", rgb_camera_info2);
  } // end loop i

  printf("depth_camera_info=== depth_camera_info2? %i\n",
         are_camera_infos_equal(depth_camera_info, depth_camera_info2));
  printf("rgb_camera_info=== rgb_camera_info2? %i\n",
         are_camera_infos_equal(rgb_camera_info, rgb_camera_info2));
  // image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
}

////////////////////////////////////////////////////////////////////////////////

double distance_points3(const cv::Point3d & A, const cv::Point3d & B) {
  return sqrt((A.x - B.x) * (A.x - B.x)
              +  (A.y - B.y) * (A.y - B.y)
              +  (A.z - B.z) * (A.z - B.z));
}

inline void test_reproject
(const cv::Point3d A3d,
 const image_geometry::PinholeCameraModel & depth_camera_model)
{
  std::cout << std::endl;
  ROS_WARN("test_reproject(A3d:(%g,%g,%g)", A3d.x, A3d.y, A3d.z);
  cv::Point2d A2d = vision_utils::world2pixel<cv::Point2d>
      (A3d, depth_camera_model);
  ROS_WARN("A2d:(%g,%g)", A2d.x, A2d.y);
  cv::Point3d A3d_back = vision_utils::pixel2world_depth<cv::Point3d>
      (A2d, depth_camera_model, A3d.z);
  double dist = distance_points3(A3d, A3d_back);
  ROS_WARN("A3d_back:(%g,%g,%g), dist %g < 0.1:%i",
           A3d.x, A3d.y, A3d.z, dist, (dist < .1));
}

void test_reproject(const std::string & kinect_serial_number) {
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  vision_utils::read_camera_model_files
      (kinect_serial_number, depth_camera_model, rgb_camera_model);
  test_reproject(cv::Point3d(0, 0, 0), depth_camera_model);
  test_reproject(cv::Point3d(0, 0, 1), depth_camera_model);
  test_reproject(cv::Point3d(0, 1, 0), depth_camera_model);
  test_reproject(cv::Point3d(0, 1, 1), depth_camera_model);
  test_reproject(cv::Point3d(1, 0, 0), depth_camera_model);
  test_reproject(cv::Point3d(1, 0, 1), depth_camera_model);
  test_reproject(cv::Point3d(1, 1, 0), depth_camera_model);
  test_reproject(cv::Point3d(1, 1, 1), depth_camera_model);
  for (int pt = 0; pt < 10; ++pt)
    test_reproject(cv::Point3d(-2 + 4 * drand48(),
                               -2 + 4 * drand48(),
                               4 * drand48()), depth_camera_model);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  srand(time(NULL));
  srand48(time(NULL));
  std::string IMG_DIR = ros::package::getPath("vision_utils") + std::string("/data/images/");
#if 0
  camera_info_bag_to_binary_file(KINECT_SERIAL_LAB());
  camera_info_bag_to_binary_file(KINECT_SERIAL_ARNAUD());
  return 0;
#endif

  int idx = 1;
  printf("%i: reproject_image_ros(IMG_DIR depth/inside1, KINECT_SERIAL_LAB())\n", idx++);
  printf("%i: reproject_image_ros(IMG_DIR depth/juggling1, KINECT_SERIAL_LAB())\n", idx++);
  printf("%i: reproject_image_ros(IMG_DIR depth/juggling2, KINECT_SERIAL_LAB())\n", idx++);
  printf("%i: reproject_image_ros(IMG_DIR depth/juggling3, KINECT_SERIAL_LAB())\n", idx++);
  printf("%i: reproject_image_ros(IMG_DIR depth/alberto1, KINECT_SERIAL_LAB())\n", idx++);
  printf("%i: reproject_image_ros(IMG_DIR depth/alberto2, KINECT_SERIAL_LAB())\n", idx++);
  printf("%i: reproject_image_ros(IMG_DIR depth/alvaro1, KINECT_SERIAL_LAB())\n", idx++);
  printf("%i: reproject_image_ros(IMG_DIR depth/alvaro2, KINECT_SERIAL_LAB())\n", idx++);

  printf("%i: test_reproject(DEFAULT_KINECT_SERIAL())\n", idx++);

  int choice = 10;
  std::cin >> choice;
  idx = 1;
//  if (choice == idx++)
//    reproject_image_ros(IMG_DIR + "depth/inside1", KINECT_SERIAL_LAB());
//  else if (choice == idx++)
//    reproject_image_ros(IMG_DIR + "depth/juggling1", KINECT_SERIAL_LAB());
//  else if (choice == idx++)
//    reproject_image_ros(IMG_DIR + "depth/juggling2", KINECT_SERIAL_LAB());
//  else if (choice == idx++)
//    reproject_image_ros(IMG_DIR + "depth/juggling3", KINECT_SERIAL_LAB());
//  else if (choice == idx++)
//    reproject_image_ros(IMG_DIR + "depth/alberto1", KINECT_SERIAL_LAB());
//  else if (choice == idx++)
//    reproject_image_ros(IMG_DIR + "depth/alberto2", KINECT_SERIAL_LAB());
//  else if (choice == idx++)
//    reproject_image_ros(IMG_DIR + "depth/alvaro1", KINECT_SERIAL_LAB());
//  else if (choice == idx++)
//    reproject_image_ros(IMG_DIR + "depth/alvaro2", KINECT_SERIAL_LAB());


  if (choice == idx++)
    test_reproject(DEFAULT_KINECT_SERIAL());
  return 0;
}
