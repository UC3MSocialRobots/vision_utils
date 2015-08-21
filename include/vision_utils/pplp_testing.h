/*!
  \file        pplp_testing.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/22

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

Some useful functions for testing PPLPublisherTemplate-based classes.
 */

#ifndef PPLP_TESTING_H
#define PPLP_TESTING_H

// Bring in gtest
#include <gtest/gtest.h>
// people_msgs
#include "vision_utils/pplp_template.h"
#include "vision_utils/images2ppl.h"
#include "vision_utils/kinect_openni_utils.h"
#include "vision_utils/io.h"
#include "vision_utils/content_processing.h"
#include "vision_utils/drawing_utils.h"
#include "vision_utils/utils/pt_utils.h"
// utils
#include "vision_utils/utils/timer.h"
// long_term_memory
#include <vision_utils/img_path.h>

#define ASSERT_TRUE_TIMEOUT(cond, timeout) { Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)

namespace pplp_testing {

void check_nb_pub(PPLPublisherTemplate & skill, unsigned int exp_nb_pub) {
  ASSERT_TRUE(skill.get_ppl_published_nb() == exp_nb_pub)
      << "PPL published:" << skill.get_ppl_published_nb() << ", expected:" << exp_nb_pub;
}

////////////////////////////////////////////////////////////////////////////////

void start_stop(PPLPublisherTemplate & skill,
                bool publish_user = false) {
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh_public;
  ros::Publisher rgb_pub = nh_public.advertise<sensor_msgs::Image>("rgb", 1),
      depth_pub = nh_public.advertise<sensor_msgs::Image>("depth", 1),
      user_pub = nh_public.advertise<sensor_msgs::Image>("user", 1),
      start_pub = nh_public.advertise<std_msgs::Int16>
                  (skill.get_start_stopic(), 1),
      stop_pub = nh_public.advertise<std_msgs::Int16>
                 (skill.get_stop_stopic(), 1);
  // create skill
  ASSERT_FALSE(skill.is_running());
  check_nb_pub(skill, 0);

  for (unsigned int iter = 1; iter <= 2; ++iter) {
    printf("start_stop(): iter:%i\n", iter);
    // start skill
    start_pub.publish(std_msgs::Int16());
    sleep(1); // time for publishers to be connected
    ASSERT_TRUE(skill.is_running());
    check_nb_pub(skill, iter-1);

    // trigger a PPL publishing
    // read images
    cv::Mat rgb, depth; cv::Mat1b user;
    std::string filename_prefix = IMG_DIR "depth/alberto1";
    ASSERT_TRUE(image_utils::read_rgb_depth_user_image_from_image_file
        (filename_prefix, &rgb, &depth, (publish_user? &user : NULL)));
    if (iter == 2){  // change the size to check we really get the new image
      cv::resize(rgb, rgb, cv::Size(), .5, .5);
      cv::resize(depth, depth, cv::Size(), .5, .5);
      if (!user.empty())
        cv::resize(user, user, cv::Size(), .5, .5);
    }
    // publish them
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "";
    cv_bridge::CvImage rgb_msg(header, sensor_msgs::image_encodings::BGR8, rgb),
        depth_msg(header, sensor_msgs::image_encodings::TYPE_32FC1, depth);
    if (publish_user) {
      cv_bridge::CvImage user_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, user);
      user_pub.publish(user_msg.toImageMsg());
    }
    rgb_pub.publish(rgb_msg.toImageMsg());
    depth_pub.publish(depth_msg.toImageMsg());
    usleep(500 * 1000); // sleep(1);

    // check a new PPL was published
    ASSERT_TRUE(skill.is_running());
    check_nb_pub(skill, iter);

    stop_pub.publish(std_msgs::Int16());
    ASSERT_TRUE_TIMEOUT(!skill.is_running(), 1);
  } // end for iter
} // end start_stop();

////////////////////////////////////////////////////////////////////////////////

void speed_test(PPLPublisherTemplate & skill,
                bool publish_user = false,
                int emit_rate = 30,
                double min_rate_published2received = .8) {
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh_public;
  ros::Publisher rgb_pub = nh_public.advertise<sensor_msgs::Image>("rgb", 1),
      depth_pub = nh_public.advertise<sensor_msgs::Image>("depth", 1),
      user_pub = nh_public.advertise<sensor_msgs::Image>("user", 1),
      start_pub = nh_public.advertise<std_msgs::Int16>
                  (skill.get_start_stopic(), 1),
      stop_pub = nh_public.advertise<std_msgs::Int16>
                 (skill.get_stop_stopic(), 1);
  // create skill
  start_pub.publish(std_msgs::Int16());
  sleep(1); // time for publishers to be connected
  ASSERT_TRUE(skill.is_running());

  // read images
  cv::Mat rgb, depth; cv::Mat1b user;
  std::string filename_prefix = IMG_DIR "depth/juggling1";
  image_utils::read_rgb_depth_user_image_from_image_file
      (filename_prefix, &rgb, &depth, (publish_user? &user : NULL));
  std_msgs::Header header;
  header.frame_id = "";
  cv_bridge::CvImage rgb_msg(header, sensor_msgs::image_encodings::BGR8, rgb),
      depth_msg(header, sensor_msgs::image_encodings::TYPE_32FC1, depth),
      user_msg;
  if (publish_user)
    user_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, user);

  ros::Rate rate(emit_rate);
  unsigned int images_published = 0;
  Timer timer;
  while(timer.getTimeSeconds() < 1) {
    // publish images
    user_msg.header.stamp = depth_msg.header.stamp = rgb_msg.header.stamp = ros::Time::now();
    rgb_pub.publish(rgb_msg.toImageMsg());
    depth_pub.publish(depth_msg.toImageMsg());
    if (publish_user)
      user_pub.publish(user_msg.toImageMsg());
    ++images_published;
    rate.sleep();
  }

  unsigned int ppl_published = skill.get_ppl_published_nb();
  double rate_published2received = 1. * ppl_published / images_published;
  printf("rate_published2received:%g\n", rate_published2received);
  ASSERT_TRUE(rate_published2received >= min_rate_published2received)
      << "rate_published2received:" << rate_published2received
      <<  ", min_rate_published2received:" << min_rate_published2received;

  stop_pub.publish(std_msgs::Int16());
  ASSERT_TRUE_TIMEOUT(!skill.is_running(), 1);
} // end speed_test

////////////////////////////////////////////////////////////////////////////////

template<class MyPPLPublisherTemplate>
void ppl_vs_user_benchmark(MyPPLPublisherTemplate & skill,
                           const std::vector<std::string> & filename_prefixes,
                           bool publish_user = false,
                           bool accept_false_positives = true,
                           bool resume_on_fail = false,
                           const double center_thres_m = .5,// meters
                           const double mask_diff_thres = .1, // percents of the mask
                           const double max_time_alloted = 3 // seconds
                           ) {
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh_public;
  ros::Publisher rgb_pub = nh_public.advertise<sensor_msgs::Image>("rgb", 1),
      depth_pub = nh_public.advertise<sensor_msgs::Image>("depth", 1),
      user_pub;
  if (publish_user)
    user_pub = nh_public.advertise<sensor_msgs::Image>("user", 1);
  skill.start();
  sleep(1); // time for publishers to be connected
  ASSERT_TRUE(skill.is_running());


  // read images
  cv::Mat rgb, depth;
  cv::Mat1b exp_masks;
  unsigned int nfiles = filename_prefixes.size(), nb_success = 0;
  Timer timer;

  for (unsigned int file_idx = 0; file_idx < nfiles; ++file_idx) {
    const std::string filename_prefix = filename_prefixes[file_idx];
    printf("\nppl_vs_user_benchmark('%s')\n", filename_prefix.c_str());

    // read images
    ASSERT_TRUE(image_utils::read_rgb_depth_user_image_from_image_file
                (filename_prefix, &rgb, &depth, &exp_masks));

    // prepair messages and publish images
    unsigned int n_ppl_before = skill.get_ppl_published_nb();
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "";
    cv_bridge::CvImage rgb_msg(header, sensor_msgs::image_encodings::BGR8, rgb),
        depth_msg(header, sensor_msgs::image_encodings::TYPE_32FC1, depth),
        user_msg;
    if (publish_user) {
      user_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_8UC1, exp_masks);
      user_pub.publish(user_msg.toImageMsg());
    }
    rgb_pub.publish(rgb_msg.toImageMsg());
    depth_pub.publish(depth_msg.toImageMsg());

    // wait a given time to get a new PPL

    // check the new PPL has the same header
    ASSERT_TRUE_TIMEOUT(skill.get_ppl_published_nb() == n_ppl_before+1, max_time_alloted);
    const people_msgs::PeoplePoseList* ppl = &(skill.get_last_PPL());
    ASSERT_TRUE(header.stamp == ppl->header.stamp && header.frame_id == ppl->header.frame_id)
        << "header:" << header << ", ppl->header:" << ppl->header;

    // get the real users from the user mask image
    std::map<uchar, cv::Point> coms;
    image_utils::get_all_non_null_values_and_com_fast(exp_masks, coms, true, true);
    unsigned int exp_nusers = coms.size(), comp_nusers = ppl->poses.size();
    bool is_success = (comp_nusers >= exp_nusers);
    ASSERT_TRUE(resume_on_fail || comp_nusers >= exp_nusers)
        <<  "exp_nusers:" << exp_nusers << ", comp_nusers:" << comp_nusers;
    if (comp_nusers == 0 && exp_nusers > 0 && resume_on_fail) {
      printf("No users found, expected %i users!\n", exp_nusers);
#ifdef DISPLAY
      skill.display(rgb, depth);
      cv::imshow("exp_mask", exp_masks > 0);
      cv::waitKey(0);
#endif // DISPLAY
      continue;
    }
    if (!accept_false_positives)
      ASSERT_TRUE(resume_on_fail || comp_nusers == exp_nusers)
          <<  "exp_nusers:" << exp_nusers << ", comp_nusers:" << comp_nusers;

    // compare expected user masks to computed user masks
    std::vector<cv::Mat1b> comp_masks;
    ppl_utils::PPL2Images::convert(*ppl, NULL, NULL, &comp_masks, NULL, NULL, false);
    //ppl_utils::ppl2user_masks(*ppl, exp_masks.size(), comp_masks);
    ASSERT_TRUE(comp_masks.size() == comp_nusers);
    cv::Mat1b comp_masks_collage;
    image_utils::paste_images(comp_masks, comp_masks_collage, true, 100, 100);

    image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
    kinect_openni_utils::read_camera_model_files(DEFAULT_KINECT_SERIAL(), depth_camera_model, rgb_camera_model);
    std::map<uchar, cv::Point>::const_iterator exp_user_it = coms.begin();
    while (exp_user_it != coms.end()) {
      // compute current user mask, center2D and 3D
      uchar exp_user = exp_user_it->first;
      cv::Mat3b exp_mask = (exp_masks == exp_user);
      cv::Point2d exp_center2d = exp_user_it->second;
      cv::Point3f exp_center3d = kinect_openni_utils::pixel2world_depth<cv::Point3f>
                                 (exp_center2d, depth_camera_model, depth);
      ++exp_user_it;

      // compute distance with closest mask
      unsigned int match_user_idx = comp_nusers;
      double min_diff_ratio = 1;
      for (unsigned int comp_user_idx = 0; comp_user_idx < comp_nusers; ++comp_user_idx) {
        double diff_ratio = 1. * cv::countNonZero(comp_masks[comp_user_idx] != exp_mask)
                            / (exp_mask.cols * exp_mask.rows);
        if (min_diff_ratio > diff_ratio) {
          min_diff_ratio = diff_ratio;
          match_user_idx = comp_user_idx;
        }
      } // end for comp_user_idx

      // check geometric distance between centers (only using xz, dismissing height y)
      ASSERT_TRUE(match_user_idx < comp_nusers);
      cv::Point3f comp_center3d;
      pt_utils::copy3(ppl->poses[match_user_idx].head_pose.position, comp_center3d);
      double dist_centers = hypot(exp_center3d.x - comp_center3d.x,
                                  exp_center3d.z - comp_center3d.z);
      printf("match exp %i-> comp %i (of %i): "
             "min_diff_ratio:%g%%, exp_center3d:%s, comp_center3d:%s, dist_centers:%g m\n",
             (int) exp_user, match_user_idx, comp_nusers,
             100. * min_diff_ratio,
             geometry_utils::printP(exp_center3d).c_str(),
             geometry_utils::printP(comp_center3d).c_str(), dist_centers);

      // check distances
      bool distance_success = (dist_centers <= center_thres_m);
      bool mask_success = (min_diff_ratio <= mask_diff_thres);

      if (distance_success && mask_success) {
        continue;
      }
      // failure handling:
      if (!distance_success)
        printf("exp_user %i is far away! dist_centers:%g, center_thres_m:%g\n",
               (int) exp_user, dist_centers, center_thres_m);
      if (!mask_success)
        printf("exp_user %i has no close mask!\n", (int) exp_user);

      is_success = false;
#ifdef DISPLAY
      skill.display(rgb, depth);
      cv::imshow("exp_mask", exp_mask);
      cv::imshow("comp_masks", comp_masks_collage);
      cv::waitKey(0);
#endif // DISPLAY
      ASSERT_TRUE(resume_on_fail);
    } // end for exp_user_idx

    if (is_success) {
      printf("Aaaaand another success! :) \n");
      ++nb_success;
#ifdef DISPLAY
      skill.display(rgb, depth);
      cv::waitKey(0);
#endif // DISPLAY
    } else
      printf("So sad, that image is not a success :/ \n");
  } // end for file_idx

  skill.stop();
  ASSERT_FALSE(skill.is_running());
  printf("\n\n *** success: %g%%, time: %g ms\n",
         100. * nb_success / nfiles, timer.getTimeMilliseconds());
} // end ppl_vs_user_benchmark();

////////////////////////////////////////////////////////////////////////////////

//! version for only one file
template<class MyPPLPublisherTemplate>
void ppl_vs_user_benchmark(MyPPLPublisherTemplate & skill,
                           const std::string & filename_prefix = IMG_DIR "depth/juggling1",
                           bool publish_user = false,
                           bool accept_false_positives = true,
                           bool resume_on_fail = false,
                           const double center_thres_m = .5,// meters
                           const double mask_diff_thres = .1, // percents of the mask
                           const double max_time_alloted = 3 // seconds
                                                           ) {
  std::vector<std::string> files (1, filename_prefix);
  return ppl_vs_user_benchmark(skill, files, publish_user,
                               accept_false_positives, resume_on_fail,
                               center_thres_m, mask_diff_thres, max_time_alloted);
}

} // end namespace pplp_testing

#endif // PPLP_TESTING_H
