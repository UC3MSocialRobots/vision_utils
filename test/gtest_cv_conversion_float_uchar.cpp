/*!
 * compressed_rounded_image_transport is an image_transport plugin for float images.
 * \author Arnaud Ramey ( arnaud.a.ramey@gmail.com )
            -- Robotics Lab, University Carlos III of Madrid
 * \date Jan. 2012
  */

#include <gtest/gtest.h>
#include <stdio.h>
#include "time/timer.h"
#include "vision_utils/cv_conversion_float_uchar.h"
#include "vision_utils/print_3f_img.h"
#include "vision_utils/float_image_generator.h"

////////////////////////////////////////////////////////////////////////////////

void test_depth_image_to_vizualisation_color_image
(const std::string & rgb_depth_filename_prefix)
{
  printf("\ntest_depth_image_to_vizualisation_color_image()\n");
  Timer timer;
  // read depth and rgb files
  cv::Mat rgb_img, depth_img;
  image_utils::read_rgb_and_depth_image_from_image_file
      (rgb_depth_filename_prefix, &rgb_img, &depth_img);
  timer.printTime("read_rgb_and_depth_image_from_image_file()");

  cv::Mat3b depth_img_illus_grey, depth_img_illus_red, depth_img_illus_full_rgb;
  cv::Mat3b depth_img_illus_grey_scaled, depth_img_illus_red_scaled, depth_img_illus_full_rgb_scaled;
  unsigned int n_times = 10;

  timer.reset();
  for (unsigned int i = 0; i < n_times; ++i)
    image_utils::depth_image_to_vizualisation_color_image
        (depth_img, depth_img_illus_grey, image_utils::GREYSCALE_STRETCHED);
  timer.printTime_factor("depth_image_to_vizualisation_color_image(GREYSCALE_STRETCHED):", n_times);

  timer.reset();
  for (unsigned int i = 0; i < n_times; ++i)
    image_utils::depth_image_to_vizualisation_color_image
        (depth_img, depth_img_illus_red, image_utils::REDSCALE_STRETCHED);
  timer.printTime_factor("depth_image_to_vizualisation_color_image(REDSCALE_STRETCHED):", n_times);

  timer.reset();
  for (unsigned int i = 0; i < n_times; ++i)
    image_utils::depth_image_to_vizualisation_color_image
        (depth_img, depth_img_illus_full_rgb, image_utils::FULL_RGB_STRETCHED);
  timer.printTime_factor("depth_image_to_vizualisation_color_image(FULL_RGB_STRETCHED):", n_times);

  timer.reset();
  for (unsigned int i = 0; i < n_times; ++i)
    image_utils::depth_image_to_vizualisation_color_image
        (depth_img, depth_img_illus_grey_scaled, image_utils::GREYSCALE_SCALED);
  timer.printTime_factor("depth_image_to_vizualisation_color_image(GREYSCALE_SCALED):", n_times);

  timer.reset();
  for (unsigned int i = 0; i < n_times; ++i)
    image_utils::depth_image_to_vizualisation_color_image
        (depth_img, depth_img_illus_red_scaled, image_utils::REDSCALE_SCALED);
  timer.printTime_factor("depth_image_to_vizualisation_color_image(REDSCALE_SCALED):", n_times);

  timer.reset();
  for (unsigned int i = 0; i < n_times; ++i)
    image_utils::depth_image_to_vizualisation_color_image
        (depth_img, depth_img_illus_full_rgb_scaled, image_utils::FULL_RGB_SCALED);
  timer.printTime_factor("depth_image_to_vizualisation_color_image(FULL_RGB_SCALED):", n_times);

  cv::imshow("rgb_img", rgb_img);
  cv::imshow("depth_img_illus_grey", depth_img_illus_grey);
  cv::imshow("depth_img_illus_red", depth_img_illus_red);
  cv::imshow("depth_img_illus_full_rgb", depth_img_illus_full_rgb);
  cv::imshow("depth_img_illus_grey_scaled", depth_img_illus_grey_scaled);
  cv::imshow("depth_img_illus_red_scaled", depth_img_illus_red_scaled);
  cv::imshow("depth_img_illus_full_rgb_scaled", depth_img_illus_full_rgb_scaled);
  cv::imwrite("depth_img_illus_full_rgb.png", depth_img_illus_full_rgb);
  cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

void test_io() {
  printf("\ntest_io()\n");
  int w = 100, h = 100;
  // only make for a depth image
  cv::Mat src_depth, src_depth_copy;
  generate_sum_row_col<float>(src_depth, w, h);
  image_utils::write_rgb_and_depth_image_to_image_file
      ("/tmp/foo", NULL,  &src_depth);
  image_utils::read_rgb_and_depth_image_from_image_file
      ("/tmp/foo", NULL,  &src_depth_copy);
  print_cv_img_info(src_depth, "src_depth");
  print_cv_img_info(src_depth_copy, "src_depth_copy");
  compare_two_images(src_depth, src_depth_copy);

  // also make it for a RGB
  cv::Mat src_rgb, src_rgb_copy;
  generate_sum_row_col<cv::Vec3b>(src_rgb, w, h);
  image_utils::write_rgb_and_depth_image_to_image_file
      ("/tmp/foo", &src_rgb,  &src_depth);
  image_utils::read_rgb_and_depth_image_from_image_file
      ("/tmp/foo", &src_rgb_copy,  &src_depth_copy);
  print_cv_img_info(src_rgb, "src_rgb");
  print_cv_img_info(src_rgb_copy, "src_rgb_copy");
  compare_two_images(src_rgb, src_rgb_copy);
}

////////////////////////////////////////////////////////////////////////////////

#include <ros/package.h>

int main(int argc, char** argv) {
  //ros::init(argc, argv, "test_cv_conversion_float_uchar");
  std::string IMG_DIR = ros::package::getPath("vision_utils") + std::string("/data/images/");
  ros::Time::init();
  srand(time(NULL));
  int idx = 1;
  if (argc < 2) {
    printf("%i: test_depth_image_to_vizualisation_color_image(IMG_DIR depth/juggling1)\n", idx++);
    printf("%i: test_depth_image_to_vizualisation_color_image(argv[1])\n", idx++);
    printf("%i: test_io()\n", idx++);
    return -1;
  }

  int choice = 0;
  choice = atoi(argv[1]);

  idx = 1;
  if (choice == idx++)
    test_depth_image_to_vizualisation_color_image(IMG_DIR + "depth/juggling1");
  else if (choice == idx++ && argc == 3)
    test_depth_image_to_vizualisation_color_image(argv[2]);
  else if (choice == idx++)
    test_io();
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
