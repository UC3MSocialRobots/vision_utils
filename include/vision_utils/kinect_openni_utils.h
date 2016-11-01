/*!
  \file        kinect_openni_utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/29

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

\namespace vision_utils
useful functions for working with a Kinect
 */

#ifndef KINECT_OPENNI_UTILS_H
#define KINECT_OPENNI_UTILS_H

// boost
#include <boost/foreach.hpp>
// ROS
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/package.h>
#include <ros/ros.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include "vision_utils/color_utils_ros.h"
#include "vision_utils/file_exists.h"
#include "vision_utils/is_nan_depth.h"
#include "vision_utils/ros_object_from_file.h"
#include "vision_utils/std_utils.h"


inline std::string KINECT_SERIAL_ARNAUD() { return "A00365A10630110A"; }
inline std::string KINECT_SERIAL_LAB() { return "A00365A82054035A"; }
inline std::string DEFAULT_KINECT_SERIAL() { return KINECT_SERIAL_LAB(); }

namespace vision_utils {

//cut:read_camera_info_bag_files

/*!
 * Read the camera info objects from ROS bag files.
 * \param kinect_serial_number
 *   The serial number of the Kinect, for instance KINECT_SERIAL_ARNAUD()
 * \param depth_camera_info, rgb_camera_info
 *    Output: the depth and rgb camera info, filled by reading the bags.
 * \return true if success
 */
bool read_camera_info_bag_files(const std::string & kinect_serial_number,
                                sensor_msgs::CameraInfo & depth_camera_info,
                                sensor_msgs::CameraInfo & rgb_camera_info) {
  // load bag files
  // from http://www.ros.org/doc/api/rosbag/html/c++/
  std::ostringstream prefix;
  prefix << ros::package::getPath("kinect") << "/data/" << kinect_serial_number;

  // read the bag for depth image
  std::ostringstream depth_bag_name;
  depth_bag_name << prefix.str() << "_depth.bag";
  if (!file_exists(depth_bag_name.str())) {
    printf("The depth camera info bag '%s' does not exist!\n",
           depth_bag_name.str().c_str());
    return false;
  }
  rosbag::Bag depth_bag(depth_bag_name.str());
  rosbag::View depth_view
      (depth_bag, rosbag::TopicQuery("/kinect_only_camera/depth_registered/camera_info"));
  bool depth_success = false;
  // BOOST_FOREACH(rosbag::MessageInstance const m, depth_view) {
  //  sensor_msgs::CameraInfo::ConstPtr i =
  //      m.instantiate<sensor_msgs::CameraInfo>();
  sensor_msgs::CameraInfo::ConstPtr i =
      depth_view.begin()->instantiate<sensor_msgs::CameraInfo>();
  if (i != NULL) {
    printf("depth camera info succesfully read from '%s'\n",
           depth_bag_name.str().c_str());
    depth_camera_info = *i;
    depth_success = true;
    // break;
  }
  // } end BOOST_FOREACH
  depth_bag.close();
  if (!depth_success)
    return false;

  // read the bag for rgb image
  std::ostringstream rgb_bag_name;
  rgb_bag_name << prefix.str() << "_rgb.bag";
  if (!file_exists(rgb_bag_name.str())) {
    printf("The rgb camera info bag '%s' does not exist!\n",
           rgb_bag_name.str().c_str());
    return false;
  }
  rosbag::Bag rgb_bag(rgb_bag_name.str());
  rosbag::View rgb_view
      (rgb_bag, rosbag::TopicQuery("/kinect_only_camera/rgb/camera_info"));
  bool rgb_success = false;
  //  BOOST_FOREACH(rosbag::MessageInstance const m, rgb_view) {
  //    sensor_msgs::CameraInfo::ConstPtr i = m.instantiate<sensor_msgs::CameraInfo>();
  i = rgb_view.begin()->instantiate<sensor_msgs::CameraInfo>();
  if (i != NULL) {
    printf("rgb camera info succesfully read from '%s'\n",
           rgb_bag_name.str().c_str());
    rgb_camera_info = *i;
    rgb_success = true;
    // break;
  }
  // } end BOOST_FOREACH
  rgb_bag.close();
  if (!rgb_success)
    return false;

  // if we are here, success
  printf("read_camera_info_bag_files(): reading from '%s' and '%s' was a success.\n",
         depth_bag_name.str().c_str(), rgb_bag_name.str().c_str());
  return true;
} // end read_camera_info_bag_files()

////////////////////////////////////////////////////////////////////////////////

//cut:read_camera_info_binary_files

/*!
 * Read the camera info objects from ROS binary files.
 * These binary files can be obtained for instance using
 * ros_object_to_file().
 * \param kinect_serial_number
 *   The serial number of the Kinect, for instance KINECT_SERIAL_ARNAUD()
 * \param depth_camera_info, rgb_camera_info
 *    Output: the depth and rgb camera info, filled by reading the bags.
 * \return true if success
 */
bool read_camera_info_binary_files(const std::string & kinect_serial_number,
                                   sensor_msgs::CameraInfo & depth_camera_info,
                                   sensor_msgs::CameraInfo & rgb_camera_info) {
  // load binary files
  std::ostringstream prefix;
  prefix << ros::package::getPath("kinect") << "/data/"  << kinect_serial_number;

  // read the binary for depth image
  std::ostringstream depth_binary_name;
  depth_binary_name << prefix.str() << "_depth.dat";
  if (!file_exists(depth_binary_name.str())) {
    printf("The depth camera info binary '%s' does not exist!\n",
           depth_binary_name.str().c_str());
    return false;
  }
  ros_object_from_file(depth_binary_name.str(), depth_camera_info);

  // read the binary for rgb image
  std::ostringstream rgb_binary_name;
  rgb_binary_name << prefix.str() << "_rgb.dat";
  if (!file_exists(rgb_binary_name.str())) {
    printf("The rgb camera info binary '%s' does not exist!\n",
           rgb_binary_name.str().c_str());
    return false;
  }
  ros_object_from_file(rgb_binary_name.str(), rgb_camera_info);

  // if we are here, success
  //  printf("read_camera_info_binary_files(): reading from '%s' and '%s' was a success.\n",
  //            depth_binary_name.str().c_str(), rgb_binary_name.str().c_str());
  return true;
} // end read_camera_info_binary_files()

////////////////////////////////////////////////////////////////////////////////

//cut:try_to_get_kinect_serial_number

/*!
 * try to obtain the serial nb of the kinect from the parameter server
 * \param nh_public
 *   a node handle to access the param server
 * \param kinect_serial_number
 *   where to store the result.
 * \return
 *   true if success.
 *   In that case, \a kinect_serial_number contains the serial number of the Kinect.
 */
bool try_to_get_kinect_serial_number(const ros::NodeHandle & nh_public,
                                     std::string & kinect_serial_number) {
  kinect_serial_number = "";
  std::string name_resolved = nh_public.resolveName("kinect_serial_number");
  bool success = nh_public.getParam(name_resolved, kinect_serial_number);
  if (!success) {
    printf("Could not get kinect_serial_number param on '%s'!\n",
           name_resolved.c_str());
  }
  return success;
}

////////////////////////////////////////////////////////////////////////////////

//cut:read_camera_model_files

/*!
 * \brief read_camera_model_files
 * \param kinect_serial_number
 * \param depth_camera_model, rgb_camera_model
 *    Output: the depth and rgb camera model, filled by reading the bags.
 * \param is_binary_file
 *    true if the camera info are stored into binary files
 *    (\see \a read_camera_info_binary_files()),
 *    false if they are into bag files
 *    (\see \a read_camera_info_bag_files())
 * \return
 *    true if success
 */
bool read_camera_model_files(const std::string & kinect_serial_number,
                             image_geometry::PinholeCameraModel & depth_camera_model,
                             image_geometry::PinholeCameraModel & rgb_camera_model,
                             bool is_binary_file = true) {
  // read camera info
  sensor_msgs::CameraInfo depth_camera_info, rgb_camera_info;
  bool success;
  if (is_binary_file)
    success = read_camera_info_binary_files
              (kinect_serial_number, depth_camera_info, rgb_camera_info);
  else
    success = read_camera_info_bag_files
              (kinect_serial_number, depth_camera_info, rgb_camera_info);
  if (!success)
    return false;
  // convert depth info to model
  success = depth_camera_model.fromCameraInfo(depth_camera_info);
  if (!success) {
    printf("Error while converting depth_camera_info -> depth_camera_model\n");
    return false;
  }
  // convert rgb info to model
  success = rgb_camera_model.fromCameraInfo(rgb_camera_info);
  if (!success) {
    printf("Error while converting rgb_camera_info -> rgb_camera_model\n");
    return false;
  }

  //  printf("Succesfully converted depth_camera_info -> depth_camera_model "
  //         "and rgb_camera_info -> rgb_camera_model\n");
  return true;
} // end read_camera_model_files()

////////////////////////////////////////////////////////////////////////////////
//cut:get_kinect_serial_number_and_read_camera_model_files_if_needed

/*!
 * If the camera models are already initialized, do nothing.
 * Otherwise, try to
 *  1) get the serial number of the kinect from the parameter server
 *  2) read the corresponding binary files for both depth and rgb camera infos.
 * \param nh_public
 *   The node handle that will enable to access the
 *    serial number of the kinect
 *    (\see \a try_to_get_kinect_serial_number())
 * \param depth_camera_model, rgb_camera_model
 *    Output: the depth and rgb camera model, filled by reading the bags.
 * \return
 *    true if success (models already initialized or succesful reading)
 */
bool get_kinect_serial_number_and_read_camera_model_files_if_needed
(const ros::NodeHandle & nh_public,
 image_geometry::PinholeCameraModel & rgb_camera_model,
 image_geometry::PinholeCameraModel & depth_camera_model
 )
{
  // electric:  v1.6.5
#if ( ROS_VERSION_MINIMUM(1, 7, 0) ) // fuerte code
  if (rgb_camera_model.initialized() && depth_camera_model.initialized())
#else // electric code
  if (rgb_camera_model.fx() > 0 && depth_camera_model.fx() > 0)
#endif // USE
    return true;
  std::string kinect_serial_number;
  bool success = try_to_get_kinect_serial_number
                 (nh_public, kinect_serial_number);
  if (!success) {
    printf("Unable to get the serial number of the Kinect, returning!");
    return false;
  }
  //printf("Serial number of the Kinect:'%s'", kinect_serial_number.c_str());
  return read_camera_model_files
      (kinect_serial_number, depth_camera_model, rgb_camera_model);
} // end get_kinect_serial_number_and_read_camera_model_files_if_needed()

////////////////////////////////////////////////////////////////////////////////
//cut:pixel2world_depth

/*!
 * Template version of pixel2world_depth().
 * It calls the non templated version, with cv::Point3d,
 * then converts it.
 */
template<class Pt3>
inline Pt3 pixel2world_depth
(const cv::Point2d & depth_pt,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const double & depth_value)
{
  cv::Point3d ans_cv =
      pixel2world_depth<cv::Point3d>(depth_pt, depth_cam_model, depth_value);
  Pt3 ans;
  ans.x = ans_cv.x;
  ans.y = ans_cv.y;
  ans.z = ans_cv.z;
  return ans;
} // end pixel2world_depth()


////////////////////////////////////////////////////////////////////////////////

/*!
 * The version of pixel2world_depth() needing the least data.
 * It first gets the depth value,
 * then calls the templated version of pixel2world_depth().
 */
template<class Pt3>
inline Pt3 pixel2world_depth
(const cv::Point2d & depth_pt,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const cv::Mat & depth_img) {
  IplImage depth_img_ipl = depth_img;
  if (depth_pt.x < 0 || depth_pt.x >= depth_img.cols
      || depth_pt.y < 0 || depth_pt.y >= depth_img.rows) {
    printf("Depth point (%g,%g) is outside the depth image (%ix%i).\n",
             depth_pt.x, depth_pt.y,
             depth_img.cols, depth_img.rows);
    return Pt3();
  }
  double depth_value = CV_IMAGE_ELEM(&depth_img_ipl, float,
                                     (int) depth_pt.y, (int) depth_pt.x);
  return pixel2world_depth<Pt3>(depth_pt, depth_cam_model, depth_value);
}

////////////////////////////////////////////////////////////////////////////////

static const double NAN_DOUBLE = 0;

/*!
 * The lowest level version, needing the most data.
 * \arg pt_rgb the point in the 2D RGB image, classic image frame
     +--------> x
     |
     |
   y V

   \return pt3d with such (z in direction of the scene):

<scene here>

   ^ z
    \
     +---> x
     |
     |
   y V

 */
template<>
inline cv::Point3d pixel2world_depth
(const cv::Point2d & depth_pt,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const double & depth_value)
{
  if (is_nan_depth(depth_value))
    return cv::Point3d(NAN_DOUBLE, NAN_DOUBLE, NAN_DOUBLE);
  cv::Point3d line_vec = depth_cam_model.projectPixelTo3dRay(depth_pt);
  // double line_vec_norm = norm(line_vec);
  //  if (fabs(line_vec_norm) < 1E-5)
  //    return cv::Point3d(0, 0, 0);
  //    //printf("depth_pt%s, line_vec:%s, line_vec_norm:%g",
  //             printP2(depth_pt).c_str(),
  //             printP(line_vec).c_str(),
  //             line_vec_norm);
  // return line_vec * (depth_value / line_vec_norm);

  if (fabs(line_vec.z) < 1E-5)
    return cv::Point3d(0, 0, 0);
  // return line_vec * (depth_value / line_vec.z);
  return cv::Point3d(line_vec.x * depth_value / line_vec.z,
                     line_vec.y * depth_value / line_vec.z,
                     depth_value);
} // end pixel2world_depth()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Reproject a depth image into a point cloud.
 * \param depth
 *    The depth image to be reprojected.
 * \param depth_cam_model
 *    The model of the camera that enables the reprojection.
 * \param depth_reprojected
 *    The point cloud after reprojection.
 * \param data_step
 *    Use 1 to reproject all pixels of \a depth,
 *    use 2 to reproject 50% of the pixels (every other pixel),
 *    use 3 for every third pixel, etc.
 *    Enables a good speedup while keeping most of the image.
 * \param mask
 *    A mask indicating what needs to be reprojected.
 *    Leave empty to reproject the whole image.
 * \param remove_nans
 *    If true, do not insert into the cloud points that correspond to
 *    an undefined depth (shiny surfaces, etc.)
 */
template<class Pt3>
inline bool pixel2world_depth
(const cv::Mat & depth_img,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 std::vector<Pt3> & depth_reprojected,
 const unsigned int data_step = 1,
 const cv::Mat1b & mask = cv::Mat(),
 bool remove_nans = false)
{
  int rows = depth_img.rows, cols = depth_img.cols;
  if (!depth_cam_model.initialized()) {
    printf("pixel2world_depth(): depth_cam_model not initialized\n");
    return false;
  }

  // dimensions check
  bool use_mask = (!mask.empty());
  if (use_mask && depth_img.size() != mask.size()) {
    printf("pixel2world_depth(): depth_img size (%i, %i) != mask size (%i, %i) -> "
           "no using mask\n", depth_img.cols, depth_img.rows, mask.cols, mask.rows);
    use_mask = false;
  }

  // clear answer
  depth_reprojected.clear();
  depth_reprojected.reserve(cols * rows);
  if (!cols && !rows) {
    printf("pixel2world_depth(): empty image!\n");
    return true;
  }

  // locates matrix header within a parent matrix
  // cf http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  cv::Size size;
  cv::Point offset;
  depth_img.locateROI(size, offset);

  const uchar* mask_ptr;
  for (int row = 0; row < rows; row += data_step) {
    // get the address of row
    const float* depth_ptr = depth_img.ptr<float>(row);
    if (use_mask)
      mask_ptr = mask.ptr<uchar>(row);
    for (int col = 0; col < cols; col += data_step) {
      if (use_mask && !mask_ptr[col])
        continue;
      if (remove_nans && is_nan_depth(depth_ptr[col]))
        continue;
      depth_reprojected.push_back
          ( pixel2world_depth<Pt3>
            (cv::Point2d(col + offset.x, row + offset.y),
             depth_cam_model, depth_ptr[col]) );
    } // end loop col
  } // end loop row
  return true;
} // end pixel2world_depth()

////////////////////////////////////////////////////////////////////////////////
//cut:pixel2world_rgb

/*!
 * Reproject a pixel of the RGB into 3D.
 * Version with a known depth value for the RGB pixel.
 * \param rgb_to_depth_scale_x, rgb_to_depth_scale_y
 *   Enables converting the RGB pixel into a depth pixel.
 */
inline cv::Point3d pixel2world_rgb(const cv::Point2d & pt_rgb,
                                   const image_geometry::PinholeCameraModel & depth_cam_model,
                                   const double & depth_value,
                                   const double rgb_to_depth_scale_x = 1,
                                   const double rgb_to_depth_scale_y = 1) {
  return pixel2world_depth<cv::Point3d>
      (cv::Point2d(rgb_to_depth_scale_x * pt_rgb.x,
                   rgb_to_depth_scale_y * pt_rgb.y),
       depth_cam_model, depth_value);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Reproject a pixel of the RGB into 3D.
 * Version reading the depth value from the depth image.
 * \param rgb_to_depth_scale_x, rgb_to_depth_scale_y
 *   Enables converting the RGB pixel into a depth pixel.
 */
inline cv::Point3d pixel2world_rgb(const cv::Point2d & pt_rgb,
                                   const image_geometry::PinholeCameraModel & depth_cam_model,
                                   const cv::Mat & depth_img,
                                   const double rgb_to_depth_scale_x = 1,
                                   const double rgb_to_depth_scale_y = 1) {
  cv::Point2d depth_pt(rgb_to_depth_scale_x * pt_rgb.x,
                       rgb_to_depth_scale_y * pt_rgb.y);
  if (depth_pt.x < 0 || depth_pt.x >= depth_img.cols
      || depth_pt.y < 0 || depth_pt.y >= depth_img.rows) {
    printf("Depth point (%g,%g) is outside the depth image (%ix%i)\n",
             depth_pt.x, depth_pt.y,
             depth_img.cols, depth_img.rows);
    return cv::Point3d(0, 0, 0);
  }
  IplImage depth_img_ipl = depth_img;
  double depth_value =
      //depth_img.at<float>(depth_pt.y, depth_pt.x);
      CV_IMAGE_ELEM(&depth_img_ipl, float,
                    (int) depth_pt.y, (int) depth_pt.x);
  return pixel2world_depth<cv::Point3d >(depth_pt, depth_cam_model, depth_value);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief pixel2world_rgb_color255
 * \param bgr_img
 *   The color image
 * \param depth_img
 *   The depth image
 * \param depth_cam_model
 *   The camera model of the depth image
 * \param depth_reprojected
 *   [OUT] the 3D points
 * \param colors
 *   [OUT] a vector, of, for instance, cv::Scalar or cv::Vec3b
 * \param data_step
 *   The increment in columns and rows for reprojecting the pixels.
 *   Use 1 to reproject all pixels, 2 to reproject 50% of them, etc.
 * \param mask_depth
 *   An optional mask to indicate what points should be kept
 *  (0=> discared pixel, anything else => reproject)
 * \param remove_nans
 *    If true, do not insert into the cloud points that correspond to
 *    an undefined depth (shiny surfaces, etc.)
 * \return
 *    true if success
 */
template<class Pt3, class Color255>
inline bool pixel2world_rgb_color255
(const cv::Mat & bgr_img,
 const cv::Mat & depth_img,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 std::vector<Pt3> & depth_reprojected,
 std::vector<Color255> & colors,
 const unsigned int data_step = 1,
 const cv::Mat1b & mask_depth = cv::Mat(),
 bool remove_nans = false)
{
  // clear answer
  int cols = depth_img.cols, rows = depth_img.rows;
  int n_pts = rows * cols / (data_step * data_step);
  depth_reprojected.clear();
  depth_reprojected.reserve(n_pts);
  colors.clear();
  colors.reserve(n_pts);

  // dimensions check
  if (depth_img.size() != bgr_img.size()) {
    printf("pixel2world_rgb(): depth size (%i, %i) != bgr_img size (%i, %i)\n",
           depth_img.cols, depth_img.rows, bgr_img.cols, bgr_img.rows);
    return false;
  }
  bool use_mask = (!mask_depth.empty());
  if (use_mask && depth_img.size() != mask_depth.size()) {
    printf("pixel2world_rgb(): depth size (%i, %i) != mask_depth size (%i, %i) -> "
           "no using mask\n", depth_img.cols, depth_img.rows,
           mask_depth.cols, mask_depth.rows);
    use_mask = false;
  }
  if (!depth_cam_model.initialized()) {
    printf("pixel2world_rgb(): depth_cam_model not initialized\n");
    return false;
  }
  if (depth_img.empty()) {
    printf("pixel2world_rgb(): empty images\n");
    return true;
  }

  // locates matrix header within a parent matrix
  // cf http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  cv::Size size;
  cv::Point offset;
  bgr_img.locateROI(size, offset);

  const uchar* mask_ptr;
  for (int row = 0; row < rows; row += data_step) {
    // get the address of row
    const float* depth_ptr = depth_img.ptr<float>(row);
    const cv::Vec3b* bgr_data = bgr_img.ptr<cv::Vec3b>(row);
    if (use_mask)
      mask_ptr = mask_depth.ptr<uchar>(row);
    for (int col = 0; col < cols; col += data_step) {
      if (use_mask && mask_ptr[col] == 0)
        continue;
      if (remove_nans && is_nan_depth(depth_ptr[col]))
        continue;
      depth_reprojected.push_back
          ( pixel2world_depth<Pt3>
            (cv::Point2d(col + offset.x, row + offset.y),
             depth_cam_model, depth_ptr[col]) );
      //colors.push_back(uchar_bgr_color_to_ros_rgba_color(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]));
      colors.push_back(Color255(bgr_data[col][0], bgr_data[col][1], bgr_data[col][2]));
    } // end loop col
  } // end loop row
  return true;
} // end pixel2world_rgb_color255()

////////////////////////////////////////////////////////////////////////////////

/*!
 * A version of pixel2world_rgb() using std_msgs::ColorRGBA
 * instead of cv::Scalar.
 * It calls the cv::Scalar version of pixel2world_rgb_color255(),
 * then converts the obtained colors.
 *
 * For all params, cf the cv::Scalar version of pixel2world_rgb_color255().
 */
template<class Pt3>
inline bool pixel2world_rgb_color_rgba
(const cv::Mat & bgr_img,
 const cv::Mat & depth_img,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 std::vector<Pt3> & depth_reprojected,
 std::vector<std_msgs::ColorRGBA> & colors,
 const unsigned int data_step = 1,
 const cv::Mat1b & mask_depth = cv::Mat(),
 bool remove_nans = false)
{
  // call pixel2world_rgb() with cv::Scalar.
  std::vector<cv::Scalar> colors_scalars;
  bool ret = pixel2world_rgb_color255
             (bgr_img, depth_img, depth_cam_model,
              depth_reprojected, colors_scalars, data_step, mask_depth, remove_nans);
  // convert cv::Scalar -> std_msgs::ColorRGBA
  colors.clear();
  colors.reserve(colors_scalars.size());
  for (unsigned int color_idx = 0; color_idx < colors_scalars.size(); ++color_idx)
    colors.push_back(uchar_bgr_color_to_ros_rgba_color
                     (colors_scalars[color_idx][0],
                     colors_scalars[color_idx][1],
        colors_scalars[color_idx][2]));
  return ret;
} // end pixel2world_rgb_color_rgba()

////////////////////////////////////////////////////////////////////////////////
//cut:compute_rgb_to_depth_scale

inline void compute_rgb_to_depth_scale(const cv::Size & rgb_size,
                                       const cv::Size & depth_size,
                                       double & rgb_to_depth_scale_x,
                                       double & rgb_to_depth_scale_y) {
  rgb_to_depth_scale_x = 1.f * depth_size.width / rgb_size.width;
  // 1.f * bridge_depth_img_ptr->image.cols / bridge_rgb_img_ptr->image.cols;
  rgb_to_depth_scale_y = 1.f * depth_size.height / rgb_size.height;
  // 1.f * bridge_depth_img_ptr->image.rows / bridge_rgb_img_ptr->image.rows;
}

////////////////////////////////////////////////////////////////////////////////
//cut:world2pixel

template<class Pt2>
inline Pt2 world2pixel
(const cv::Point3d & pt,
 const image_geometry::PinholeCameraModel & depth_or_rgb_cam_model)
{
  cv::Point2d ans_cv = world2pixel<cv::Point2d>(pt, depth_or_rgb_cam_model);
  Pt2 ans;
  ans.x = ans_cv.x;
  ans.y = ans_cv.y;
  return ans;
} // end world2pixel()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Converts a 2D point into a pixel.
 * The coordinates of this pixel will be in the RGB image
 * if the RGB cam model is given (same thing for depth).
 * \param pt
 *  The 3D point, with the following frame
 *  (/<robot>_camera_rgb_optical_frame = /<robot>_camera_depth_optical_frame).
 *  They correspond to the frame_id that comes with the 2D images topics.
 *
 *  ( Careful, the frames /<robot>_camera_rgb_frame = /<robot>_camera_depth_frame )
 *  ( are different. Needs a TF conversion for these.                             )

<scene here>

   ^ z
    \
     +---> x
     |
     |
   y V

 * \param depth_or_rgb_cam_model
 *    The camera model corresponding to the pixel coordinates we want
 * \return rgb the point in the 2D RGB image, classic image frame
     +--------> x
     |
     |
   y V

 */
template<>
inline cv::Point2d world2pixel(const cv::Point3d & pt,
                               const image_geometry::PinholeCameraModel & depth_or_rgb_cam_model) {
  return depth_or_rgb_cam_model.project3dToPixel(pt);
}

////////////////////////////////////////////////////////////////////////////////
//cut:dist3

inline double dist3(const cv::Point3d & p1, const cv::Point3d & p2) {
  return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

//cut:compute_pixel2meters_factor

/*!
 * \return the factor to convert a distance in pixels into a distance in meters.
 * It is in meters / pixels.
 * Return "nan" if depth_value is nan. The returned value can be determined with isnan().
 * \param depth_size
 * \param depth_cam_model
 * \param depth_value
 */
inline double compute_pixel2meters_factor
(const cv::Size & depth_size,
 const image_geometry::PinholeCameraModel & depth_cam_model,
 const double & depth_value)
{
  int seg_size = depth_size.height / 2;
  cv::Point2d p1(depth_size.width / 2, depth_size.height / 2 - seg_size / 2);
  cv::Point2d p2(depth_size.width / 2, depth_size.height / 2 + seg_size / 2);
  cv::Point3d p1_world = pixel2world_depth<cv::Point3d>(p1, depth_cam_model, depth_value);
  cv::Point3d p2_world = pixel2world_depth<cv::Point3d>(p2, depth_cam_model, depth_value);
  return dist3(p1_world, p2_world) / seg_size;
}

////////////////////////////////////////////////////////////////////////////////

//! a short version for the other compute_pixel2meters_factor()
inline double compute_pixel2meters_factor
(const cv::Mat1f & depth_img,
 const image_geometry::PinholeCameraModel & depth_camera_model,
 const cv::Point & depth_pt)
{
  if (depth_pt.x < 0 || depth_pt.x >= depth_img.cols
      || depth_pt.y < 0 || depth_pt.y >= depth_img.rows) {
    printf("Depth point (%i,%i) is outside the depth image (%ix%i).\n",
             depth_pt.x, depth_pt.y,
             depth_img.cols, depth_img.rows);
    return NAN_DOUBLE;
  }
  IplImage depth_img_ipl = depth_img;
  const double depth_value = CV_IMAGE_ELEM(&depth_img_ipl, float,
                                           (int) depth_pt.y, (int) depth_pt.x);
  return compute_pixel2meters_factor
      (depth_img.size(), depth_camera_model, depth_value);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \param depth_img
 *    The depth image coming from a kinect.
 * \param kinect_serial_number
 *    Needed for loading our kinect camera model.
 * \param depth_pt
 *    The point where we want to evaluate the pixel2meters_factor.
 * \return
 *    pixel2meters_factor (in meters/pixels),
 *    or NAN_DOUBLE if error.
 */
inline double compute_pixel2meters_factor(const cv::Mat1f & depth_img,
                                          const std::string & kinect_serial_number,
                                          const cv::Point & depth_pt) {
  // get camera model
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  bool ok = read_camera_model_files
            (kinect_serial_number, depth_camera_model, rgb_camera_model);
  if (!ok)
    return NAN_DOUBLE;
  // get pixel2meters_factor
  return compute_pixel2meters_factor
      (depth_img, depth_camera_model, depth_pt);
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline double compute_average_pixel2meters_factor
(const cv::Mat1f & depth_img,
 const _T & depth_camera_model,
 const cv::Mat1b & mask)
{
  if (mask.empty() || depth_img.size() != mask.size()) {
    printf("compute_average_pixel2meters_factor: dimensions of depth_img(%i, %i), "
           "mask(%i, %i) dont match!\n",
           depth_img.cols, depth_img.rows, mask.cols, mask.rows);
    return NAN_DOUBLE;
  }
  // compute median
  unsigned int nvalues = 50;
  std::vector<double> factors;
  factors.reserve(nvalues);
  unsigned int ntries = 0, maxtries = 100 * nvalues;
  while(factors.size() < nvalues && ++ntries < maxtries) {
    // take random point
    cv::Point depth_pt(rand() % depth_img.cols, rand() % depth_img.rows);
    // check it is in mask
    if (!mask(depth_pt))
      continue;
    // get pixel2meters_factor
    double curr_factor = compute_pixel2meters_factor
                         (depth_img, depth_camera_model, depth_pt);
    if (isnan(curr_factor))
      continue;
    factors.push_back(curr_factor);
  }
  if (ntries >= maxtries) {
    printf("compute_average_pixel2meters_factor(): reached the max number of tries, "
           "most probably the given depth image is empty\n");
    return NAN_DOUBLE;
  }

  // find median,
  // http://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation?lq=1
  size_t n = factors.size() / 2;
  std::nth_element(factors.begin(), factors.begin()+n, factors.end());
  return factors[n];
} // end compute_average_pixel2meters_factor()

////////////////////////////////////////////////////////////////////////////////
//cut

} // end namespace vision_utils

#endif // KINECT_OPENNI_UTILS_H
