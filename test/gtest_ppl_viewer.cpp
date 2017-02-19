/*!
  \file        gtest_ppl_viewer.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/2

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

Some tests for PPLViewer

 */
bool display = false;

// Bring in gtest
#include <gtest/gtest.h>
#include "vision_utils/iterable_to_string.h"
#include "vision_utils/ppl_viewer.h"
#include "vision_utils/images2ppl.h"
#include "vision_utils/timer.h"
#include <vision_utils/img_path.h>
#include <vision_utils/rosmaster_alive.h>
#include <vision_utils/ppl_attributes.h>

#define MYTIMEOUT 5 // seconds
#define WAIT_WHILE(cond, timeout) { vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && (cond)) usleep(50 * 1000); }
#define ASSERT_TRUE_TIMEOUT(cond, timeout) { vision_utils::Timer timer; while (timer.getTimeSeconds() < timeout && !(cond)) usleep(50 * 1000); } ASSERT_TRUE(cond)
#define DEG2RAD     0.01745329251994329577  //!< to convert degrees to radians

void imshow_if_needed() {
  if (display) {
    printf("\nPress a key..\n");
    cv::waitKey(0);
  } // end if display
}

TEST(TestSuite, ctor) {
  if (!vision_utils::rosmaster_alive()) return;
  vision_utils::PPLViewer viewer;
  ASSERT_TRUE(viewer.get_nb_received_methods() == 0);
  ASSERT_TRUE(viewer.get_nb_total_received_tracks() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_ppl) {
  if (!vision_utils::rosmaster_alive()) return;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  // create PPL publisher
  ros::NodeHandle nh_public, nh_private("~");
  ros::Publisher ppl_pub = nh_public.advertise<vision_utils::PPLViewer::PPL>("ppl", 1);
  // create viewer
  nh_private.setParam(nh_private.resolveName("ppl_topics"), ppl_pub.getTopic());
  vision_utils::PPLViewer viewer;
  viewer.set_display(display);
  // check viewer and PPL publihser are connected
  ASSERT_TRUE_TIMEOUT(viewer.get_nb_PPL_publishers() == 1, MYTIMEOUT);

  // publish empty PPL
  vision_utils::PPLViewer::PPL ppl;
  ppl.header.stamp = ros::Time::now();
  vision_utils::set_method(ppl, "testmethod"); // the PPL is empty so we won't set anything
  ppl_pub.publish(ppl);

  // check the PPLViewer got the emitted PPL
  ASSERT_TRUE_TIMEOUT(viewer.get_nb_received_methods() == 0, MYTIMEOUT)
      << "methods:" << vision_utils::iterable_to_string(viewer.get_all_received_methods());
  ASSERT_TRUE(viewer.get_nb_total_received_tracks() == 0);
  imshow_if_needed();
}

////////////////////////////////////////////////////////////////////////////////

void test_publish(std::string filename_prefix = vision_utils::IMG_DIR() + "depth/juggling1",
                  unsigned int nusers = 1,
                  bool use_2_ppl_methods = false,
                  bool generate_path = false) {
  if (!vision_utils::rosmaster_alive()) return;
  printf("test_publish('%s', %i users, use_2_ppl_methods:%i, generate_path:%i)\n",
         filename_prefix.c_str(), nusers, use_2_ppl_methods, generate_path);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh_public, nh_private("~");

  // spawn a ppl generator
  vision_utils::Images2PPL ppl_conv;
  std::string ppl_topic = "ppl";
  ros::Publisher ppl_pub = nh_public.advertise<vision_utils::PPLViewer::PPL>(ppl_topic, 1);

  // create viewer
  nh_private.setParam(nh_private.resolveName("ppl_topics"), ppl_pub.getTopic());
  vision_utils::PPLViewer viewer;
  viewer.set_display(display);
  ASSERT_TRUE(viewer.get_nb_received_methods() == 0);
  ASSERT_TRUE(viewer.get_nb_total_received_tracks() == 0);
  // check viewer and PPL publihser are connected
  ASSERT_TRUE_TIMEOUT(viewer.get_nb_PPL_publishers() == 1, MYTIMEOUT);

  // read images
  cv::Mat rgb, depth;
  cv::Mat1b exp_masks;
  ASSERT_TRUE(vision_utils::read_rgb_depth_user_image_from_image_file
              (filename_prefix, &rgb, &depth, &exp_masks));
  ASSERT_TRUE(ppl_conv.convert(rgb, depth, exp_masks));
  // ppl_pub.publish(ppl_conv.get_ppl());

  // publish some more
  unsigned int nexp_methods = (nusers == 0 ? 0 : use_2_ppl_methods ? 2 : 1);
  people_msgs::People ppl = ppl_conv.get_ppl();
  std::string method1 = vision_utils::get_method(ppl),
      method2 = "new_method";
  unsigned int nppls_per_method = (generate_path ? 12 : 1),
      theta_incr = 360/nppls_per_method;
  printf("nexp_methods: %i, nppls_per_method:%i, theta_incr:%i\n",
         nexp_methods, nppls_per_method, theta_incr);
  for (unsigned int theta = 0; theta < 360; theta += theta_incr) {
    printf("theta:%i\n", theta);
    for (unsigned int methodidx = 1; methodidx <= nexp_methods; ++methodidx) {
      std::string curr_method = (methodidx == 1 ? method1 : method2);
      vision_utils::set_method(ppl, curr_method);
      printf("methodidx:%i: setting method '%s'\n", methodidx, curr_method.c_str());
      for (unsigned int pp_idx = 0; pp_idx < ppl.people.size(); ++pp_idx) {
        ppl.people[pp_idx].position.x = pp_idx + cos(theta*DEG2RAD);
        ppl.people[pp_idx].position.z = methodidx + sin(theta*DEG2RAD);
      }
      ppl_pub.publish(ppl);
      usleep(100*1000);
    } // end for method
  } // end for theta

  // check the PPLViewer got the emitted PPL
  ASSERT_TRUE_TIMEOUT(viewer.get_nb_received_methods() == nexp_methods, MYTIMEOUT)
      << "methods:" << vision_utils::iterable_to_string(viewer.get_all_received_methods());
  ASSERT_TRUE(viewer.get_nb_total_received_tracks() == nexp_methods * nusers);
  std::vector<vision_utils::PPLViewer::MethodName> methods = viewer.get_all_received_methods();
  ASSERT_TRUE(methods.size() == nexp_methods);
  if (nexp_methods == 1)
    ASSERT_TRUE(methods.front() == vision_utils::get_method(ppl_conv.get_ppl()));
  else if (nexp_methods ==  2)
    ASSERT_TRUE((methods.front() == method1 && methods.back() == method2)
                || (methods.front() == method2 && methods.back() == method1));

  imshow_if_needed();
}

////////////////////////////////////////////////////////////////////////////////

void full_test_publish(std::string filename_prefix = vision_utils::IMG_DIR() + "depth/juggling1",
                       unsigned int nusers = 1) {
  test_publish(filename_prefix, nusers);
  test_publish(filename_prefix, nusers, true);
  test_publish(filename_prefix, nusers, false, true);
  test_publish(filename_prefix, nusers, true, true);
}

TEST(TestSuite, empty_lab)    { full_test_publish(vision_utils::IMG_DIR() + "depth/empty_lab", 0); }
TEST(TestSuite, juggling1)    { full_test_publish(vision_utils::IMG_DIR() + "depth/juggling1", 1); }
TEST(TestSuite, david_arnaud1){ full_test_publish(vision_utils::IMG_DIR() + "depth/david_arnaud1", 2); }

////////////////////////////////////////////////////////////////////////////////

void test_image_no_image(std::string filename_prefix = vision_utils::IMG_DIR() + "depth/juggling1",
                         unsigned int nusers = 1) {
  if (!vision_utils::rosmaster_alive()) return;
  printf("test_image_no_image('%s', %i users)\n", filename_prefix.c_str(), nusers);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh_public, nh_private("~");

  // spawn a ppl generator
  vision_utils::Images2PPL ppl_conv;
  std::string ppl_topic = "ppl";
  ros::Publisher ppl_pub = nh_public.advertise<vision_utils::PPLViewer::PPL>(ppl_topic, 1);

  // create viewer
  nh_private.setParam(nh_private.resolveName("ppl_topics"), ppl_pub.getTopic());
  vision_utils::PPLViewer viewer;
  viewer.set_display(display);
  ASSERT_TRUE(viewer.get_nb_received_methods() == 0);
  ASSERT_TRUE(viewer.get_nb_total_received_tracks() == 0);
  // check viewer and PPL publihser are connected
  ASSERT_TRUE_TIMEOUT(viewer.get_nb_PPL_publishers() == 1, MYTIMEOUT);

  // read images
  cv::Mat rgb, depth;
  cv::Mat1b exp_masks;
  ASSERT_TRUE(vision_utils::read_rgb_depth_user_image_from_image_file
              (filename_prefix, &rgb, &depth, &exp_masks));
  for (unsigned int time_idx = 0; time_idx < 5; ++time_idx) {
    printf("test_image_no_image(): time_idx:%i\n", time_idx);
    ASSERT_TRUE(ppl_conv.convert(rgb, depth, exp_masks));
    people_msgs::People ppl = ppl_conv.get_ppl();
    ppl_pub.publish(ppl);

    // check the PPLViewer got the emitted PPL
    unsigned int nexp_methods = (nusers == 0 ? 0 : 1);
    ASSERT_TRUE_TIMEOUT(viewer.get_nb_received_methods() == nexp_methods, MYTIMEOUT)
        << "methods:" << vision_utils::iterable_to_string(viewer.get_all_received_methods());
    ASSERT_TRUE(viewer.get_nb_total_received_tracks() == nusers);
    std::vector<vision_utils::PPLViewer::MethodName> methods = viewer.get_all_received_methods();
    ASSERT_TRUE(methods.size() == nexp_methods);
    ASSERT_TRUE(methods.front() == vision_utils::get_method(ppl_conv.get_ppl()));
    imshow_if_needed();

    // now re-emit without image
    for (unsigned int user_idx = 0; user_idx < nusers; ++user_idx) {
      people_msgs::Person* pp = &(ppl.people[user_idx]);
#if 0
      pp->rgb.height = pp->rgb.width = 0;
      pp->depth.height = pp->depth.width = 0;
      pp->user.height = pp->user.width = 0;
#endif
      pp->position.x += 1;
    } // end loop user_idx
    ppl_pub.publish(ppl);
    sleep(1);
    EXPECT_NO_FATAL_FAILURE();

    imshow_if_needed();
  } // end loop time_idx
} // end test_image_no_image();

TEST(TestSuite, image_no_image_empty_lab)    { test_image_no_image(vision_utils::IMG_DIR() + "depth/empty_lab", 0); }
TEST(TestSuite, image_no_image_juggling1)    { test_image_no_image(vision_utils::IMG_DIR() + "depth/juggling1", 1); }
TEST(TestSuite, image_no_image_david_arnaud1){ test_image_no_image(vision_utils::IMG_DIR() + "depth/david_arnaud1", 2); }


////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
