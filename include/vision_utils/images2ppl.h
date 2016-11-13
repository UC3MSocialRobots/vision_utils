/*!
  \file        images2ppl.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/21

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

\class Images2PPL
A useful class for converting a set of (rgb, depth, user)
frames to a PPL message.

The list of users is obtained thanks to the unique values of the user mask.

The whole cv_bridge conversion is hidden from the end user,
making life easier.
 */

#ifndef IMAGES2PPL_H
#define IMAGES2PPL_H

// msg
#include <people_msgs/People.h>
// AD
#include "vision_utils/boundingBox.h"
#include "vision_utils/copy3.h"
#include "vision_utils/get_all_non_null_values_and_com.h"
#include "vision_utils/kinect_serials.h"
#include "vision_utils/mask.h"
#include "vision_utils/pixel2world_depth.h"
#include "vision_utils/ppl_attributes.h"
#include "vision_utils/read_camera_model_files.h"
#include "vision_utils/read_rgb_depth_user_image_from_image_file.h"
// ROS
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace vision_utils {

class Images2PP {
public:
  /*!
   * Fill the image fields of a Person with some OpenCV images.
   *  \warning the header of pp should be set BEFORE calling this function
   *    as they are copied to the Person image headers.
   * \param pp
   * \param rgb, depth, user
   *    The optional image fields. Pass NULL if not available.
   *    Here, we suppose user is a binary mask,
   *    what matters is only if user>0 or user==0.
   *    For a similar function with multi-masks, cf \a Images2PPL.
   * \param need_crop_with_userbbox
   *    True if you want to compute the bbox of rgb,
   *    then crop rgb, depth, user to this bbox
   *    (if depth or user is NULL, it will be skip of course).
   * \return
   *    true if success
   */
  bool convert(people_msgs::Person & pp_out,
               const cv::Mat3b* rgb = NULL,
               const cv::Mat1f* depth = NULL,
               const cv::Mat1b* user = NULL,
               bool need_crop_with_userbbox = false) {
    bool has_rgb = (rgb != NULL),
        has_depth = (depth != NULL),
        has_user = (user != NULL);
    if (!has_rgb && !has_depth && !has_user) {
      printf("Images2PP:all images NULL!\n");
      return false;
    }

    // sizes checks
    if ((has_rgb && has_depth && rgb->size() != depth->size())
        || (has_rgb && has_user && rgb->size() != user->size())
        || (has_depth && has_user && depth->size() != user->size())) {
      printf("Images2PP:rgb(%i, %i), depth(%i, %i), user(%i, %i): size mismatch!\n",
             rgb->cols, rgb->rows, depth->cols, depth->rows,
             user->cols, user->rows);
      return false;
    }

    if (need_crop_with_userbbox) { // find the bbox
      if (!has_user) {
        printf("Images2PP:need_crop_with_userbbox true but user image NULL!\n");
        return false;
      }
      cv::Rect bbox_user = boundingBox(*user);
      if (bbox_user.width < 0) {
        printf("Images2PP:need_crop_with_userbbox true but user image black!\n");
        return false;
      }
      // crop the images
      if (has_rgb)
        rgb_bridge.image = (*rgb)(bbox_user);
      if (has_depth)
        depth_bridge.image = (*depth)(bbox_user);
      if (has_user)
        user_bridge.image = (*user)(bbox_user);
      set_tag(pp_out, "images_offsetx", bbox_user.x);
      set_tag(pp_out, "images_offsety", bbox_user.y);
    }
    else { // it is already a crop
      // locates matrix header within a parent matrix
      // cf http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
      cv::Size size;
      cv::Point rgb_offset, depth_offset, user_offset;
      // check all offsets equal
      if (has_rgb)
        rgb->locateROI(size, rgb_offset);
      if (has_depth)
        depth->locateROI(size, depth_offset);
      if (has_user)
        user->locateROI(size, user_offset);
      if ((has_rgb && has_depth && rgb_offset != depth_offset)
          || (has_rgb && has_user && rgb_offset != user_offset)
          || (has_depth && has_user && depth_offset != user_offset)) {
        printf("Images2PP:Offsets rgb(%i, %i), depth(%i, %i), user(%i, %i): offset mismatch!\n",
               rgb_offset.x, rgb_offset.y, depth_offset.x, depth_offset.y,
               user_offset.x, user_offset.y);
        return false;
      }
      if (has_rgb)
        rgb_bridge.image = (*rgb);
      if (has_depth)
        depth_bridge.image = (*depth);
      if (has_user)
        user_bridge.image = (*user);
      set_tag(pp_out, "images_offsetx", rgb_offset.x);
      set_tag(pp_out, "images_offsety", rgb_offset.y);
    } // end if (!need_crop_with_userbbox)

#if 0 // TODO deal better with the image
    rgb_bridge.header = pp_out.header;
    rgb_bridge.encoding = sensor_msgs::image_encodings::BGR8;
    pp_out.rgb = (has_rgb ? *(rgb_bridge.toImageMsg()) : sensor_msgs::Image());

    depth_bridge.header = pp_out.header;
    depth_bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    pp_out.depth = (has_depth ? *(depth_bridge.toImageMsg()) : sensor_msgs::Image());

    user_bridge.header = pp_out.header;
    user_bridge.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    pp_out.user = (has_user ? *(user_bridge.toImageMsg()) : sensor_msgs::Image());
#endif

    return true; // success
  } // end convert()

  //////////////////////////////////////////////////////////////////////////////

  inline bool rgb2user_and_convert(people_msgs::Person & pp,
                                   const cv::Mat3b* rgb = NULL,
                                   const cv::Mat1f* depth = NULL,
                                   bool need_crop_with_userbbox = false) {
    if (rgb == NULL) {
      printf("Images2PP:rgb needed for mask generation, but NULL!\n");
      return false;
    }
    mask(*rgb, _rgb2user, is_zero_vec3b);
    //  imwrite_debug("rgb.png", *rgb, COLORS256);
    //  imwrite_debug("rgb2user.png", _rgb2user, MONOCHROME);
    return convert(pp, rgb, depth, &_rgb2user, need_crop_with_userbbox);
  } // end rgb2user_and_convert()

protected:
  cv::Mat1b _rgb2user;
  cv_bridge::CvImage rgb_bridge, depth_bridge, user_bridge;
}; // end class Images2PP

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class Images2PPL {
public:
  typedef people_msgs::People PPL;

  Images2PPL() {
    // get camera model
    image_geometry::PinholeCameraModel rgb_camera_model;
    read_camera_model_files
        (DEFAULT_KINECT_SERIAL(), _default_depth_cam_model, rgb_camera_model);
    _default_frame_id = "camera_frame";
  }

  //////////////////////////////////////////////////////////////////////////////

  bool convert(const cv::Mat3b & bgr,
               const cv::Mat1f & depth,
               const cv::Mat1b & user,
               const image_geometry::PinholeCameraModel* depth_cam_model = NULL,
               const std_msgs::Header* header = NULL) {
    const image_geometry::PinholeCameraModel* depth_cam_model_safe =
        (depth_cam_model != NULL ? depth_cam_model : &_default_depth_cam_model);

    if (header != NULL)
      _ppl.header = *header;
    else {
      _ppl.header.frame_id = _default_frame_id;
      _ppl.header.stamp = ros::Time::now();
    }

    // generate PPL
    if (!get_all_non_null_values_and_com_fast(user, _coms, true, true))
      return false;
    _ppl.people.resize(_coms.size());
    // convert center of masses to 3D positions
    std::map<uchar, cv::Point>::const_iterator coms_it = _coms.begin();
    int user_idx = 0;
    while (coms_it != _coms.end()) {
      cv::Point3d com3D =
          // pixel2world_depth(coms_it->second);
          pixel2world_depth<cv::Point3d>
          (coms_it->second, *depth_cam_model_safe, depth);
      // shape a Person
      people_msgs::Person* pp = &(_ppl.people[user_idx]);
      pp->name = cast_to_string((int) coms_it->first);
      // keep NiTE name in "user_multimap_name"
      set_method(*pp, "Images2PPL");
      set_tag(*pp, "user_multimap_name", pp->name);
      // pose
      copy3(com3D, pp->position);
      //pp->position.orientation = tf::createQuaternionMsgFromYaw(0);
      pp->reliability = 1;
      set_tag(*pp, "std_dev", .1);
      // images
      cv::Mat1b curr_mask = (user == coms_it->first);
      _images2pp.convert(*pp, &bgr, &depth, &curr_mask, true);
      // iterate
      ++coms_it;
      ++user_idx;
    } // end loop coms_it
    return true;
  } // end convert()

  //////////////////////////////////////////////////////////////////////////////

  bool convert(const std::string & filename_prefix,
               const image_geometry::PinholeCameraModel* depth_cam_model = NULL,
               const std_msgs::Header* header = NULL) {
    cv::Mat3b rgb;
    cv::Mat1f depth;
    cv::Mat1b user_mask;
    if (!read_rgb_depth_user_image_from_image_file
        (filename_prefix, &rgb, &depth, &user_mask))
      return false;
    return convert(rgb, depth, user_mask, depth_cam_model, header);
  } // end convert()

  //////////////////////////////////////////////////////////////////////////////

  inline const PPL & get_ppl() const { return _ppl; }
  inline       PPL & get_ppl()       { return _ppl; }
  inline unsigned int nusers() const { return _ppl.people.size(); }

  //////////////////////////////////////////////////////////////////////////////

  inline const std::map<uchar, cv::Point> & get_coms() { return _coms; }

  //////////////////////////////////////////////////////////////////////////////

private:
  image_geometry::PinholeCameraModel _default_depth_cam_model;
  PPL _ppl;
  std::string _default_frame_id;
  Images2PP _images2pp;
  std::map<uchar, cv::Point> _coms;
}; // end Images2PPL

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*! Convert a PP into a (rgb, depth, user_mask) triplet.
   * \param curr_pose
   * \param rgb, depth, user_mask
   *    pointers to the output data.
   *    Leave the ones that you are not interested into as NULL.
   * \param discarded_unnammed_poses
   *   true to discard PP that have no name data, or NO_RECOGNITION_MADE, or RECOGNITION_FAILED
   * \return
   */
bool convert(const people_msgs::Person* curr_pose,
             cv::Mat3b* rgb = NULL,
             cv::Mat1f* depth = NULL,
             cv::Mat1b* user_mask = NULL,
             bool discarded_unnammed_poses = false) {
  if (discarded_unnammed_poses
      & (curr_pose->name == ""
         || curr_pose->name == "NOREC"
         || curr_pose->name == "RECFAIL") ){
    //USUARIO NO RECONOCIDO... OOOPS!
    printf("PP2Images: user has no name!\n");
    return false;
  }
  //Try to convert ros_image to We make a copy of the image because it will be modified afterwards
#if 0 // TODO
  cv_bridge::CvImageConstPtr rgb_bridge, depth_bridge, user_mask_bridge;
  const boost::shared_ptr<void const> tracked_object;
  try {
    if (rgb)
      rgb_bridge = cv_bridge::toCvShare(curr_pose->rgb,
                                        tracked_object,
                                        sensor_msgs::image_encodings::TYPE_8UC3);
    if (depth)
      depth_bridge = cv_bridge::toCvShare(curr_pose->depth,
                                          tracked_object,
                                          sensor_msgs::image_encodings::TYPE_32FC1);
    if (user_mask)
      user_mask_bridge = cv_bridge::toCvShare(curr_pose->user,
                                              tracked_object,
                                              sensor_msgs::image_encodings::TYPE_8UC1);
  }
  catch (cv_bridge::Exception& e) {
    printf("PP2Images: cv_bridge exception: %s\n", e.what());
    return false;
  }
  if (rgb)
    rgb_bridge->image.copyTo(*rgb);
  if (depth)
    depth_bridge->image.copyTo(*depth);
  if (user_mask)
    user_mask_bridge->image.copyTo(*user_mask);
#endif
  return true;
} // end convert()

////////////////////////////////////////////////////////////////////////////////

/*! Convert a PPL into a vector of (rgb, depth, user_mask) triplet.
   *  All PPs that can not succesfully be converted are discarded.
   *  To know the original index of the PP that resulted in images #i,
   *  access pp_indices[i].
   * \param ppl
   * \param discarded_unnammed_poses
   *   true to discard PP that have no name data, or NO_RECOGNITION_MADE, or RECOGNITION_FAILED
   * \return true if success
   */
bool convert(const people_msgs::People& ppl,
             std::vector<cv::Mat3b>* rgbs = NULL,
             std::vector<cv::Mat1f>* depths = NULL,
             std::vector<cv::Mat1b>* user_masks = NULL,
             std::vector<cv::Point>* masks_offsets = NULL,
             std::vector<unsigned int>* pp_indices = NULL,
             bool discarded_unnammed_poses = false) {
  unsigned int nposes = ppl.people.size();
  if (rgbs)
    rgbs->clear();
  if (depths)
    depths->clear();
  if (user_masks)
    user_masks->clear();
  if (masks_offsets)
    masks_offsets->clear();
  if (pp_indices)
    pp_indices->clear();
  cv::Mat3b rgb;
  cv::Mat1f depth;
  cv::Mat1b user_mask;
  for (unsigned int idx=0; idx< nposes; idx++) {
    bool success = convert(&(ppl.people[idx]),
                                      (rgbs ? &rgb : NULL),
                                      (depths ? &depth : NULL),
                                      (user_masks ? &user_mask : NULL),
                                      discarded_unnammed_poses);
    if (!success)
      return false;
    if (rgbs)
      rgbs->push_back(rgb);
    if (depths)
      depths->push_back(depth);
    if (user_masks)
      user_masks->push_back(user_mask);
    if (masks_offsets) {
      int x = 0, y = 0;
      get_tag(ppl.people[idx], "images_offsetx", x);
      get_tag(ppl.people[idx], "images_offsety", y);
      masks_offsets->push_back(cv::Point(x, y));
    }
    if (pp_indices)
      pp_indices->push_back(idx);
  } // end for idx
  return true;
} //end convert()

//////////////////////////////////////////////////////////////////////////////

//! retrieve names from the PPL using the indices given back by convert()
inline bool indices2names(const people_msgs::People& ppl,
                          const std::vector<unsigned int>& pp_indices,
                          std::vector<std::string>& names) {
  unsigned int npps = pp_indices.size();
  names.clear();
  names.reserve(npps);
  for (unsigned int i = 0; i < npps; ++i)
    names.push_back(ppl.people[pp_indices[i]].name);
  return true;
} // end indices2names()

} // end namespace vision_utils
#endif // IMAGES2PPL_H
