/***************************************************************************//**
 * \file test_image_utils.cpp
 *
 * \brief Some tests for the library image_utils
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date April 2009
 *******************************************************************************/
bool display = false;
// std
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
// AD
#include "vision_utils/compute_pixel2meters_factor.h"
#include <vision_utils/img_path.h>
#include "vision_utils/timer.h"
#include "vision_utils/connected_comp_interface.h"
#include "vision_utils/image_comparer.h"
#include "vision_utils/value_remover.h"
#include "vision_utils/user_image_to_rgb.h"
#include "vision_utils/rectangle_to_string.h"
#include "vision_utils/region_growth.h"
#include "vision_utils/drawListOfPoints.h"
#include "vision_utils/to_string.h"
#include "vision_utils/from_string.h"
#include "vision_utils/infosimage.h"
#include "vision_utils/kinect_serials.h"
#include "vision_utils/read_rgb_depth_user_image_from_image_file.h"
#include "vision_utils/propagative_floodfill.h"

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, bbox) {
  cv::Mat1b test = cv::imread(vision_utils::IMG_DIR() + "star.png",
                              CV_LOAD_IMAGE_GRAYSCALE);

  vision_utils::Timer timer;
  cv::Rect bbox = vision_utils::boundingBox(test);
  timer.printTime("boundingBox()\n");
  printf("bbox:'%s'", vision_utils::rectangle_to_string(bbox).c_str());

  std::vector<cv::Point> pts;
  vision_utils::nonNulPoints(test, pts);
  timer.printTime("nonNulPoints()\n");
  printf("size of pts:%i", (int) pts.size());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, region_growth) {
  std::vector<cv::Point2i> ans;

  cv::Mat1b image = cv::imread(vision_utils::IMG_DIR() + "region_growth.png", 0);
  vision_utils::region_growth_no_seen_points(
        image, cv::Point2i(47, 70), 50, 150, ans);

  // cv::Mat1b image = cv::imread(vision_utils::IMG_DIR() + "rectangles.png", 0);
  // vision_utils::region_growth(image, cv::Point2i(1, 8), 0, 0, ans);

  vision_utils::drawListOfPoints(image, ans, (uchar) 127);
  printf("nb of points:%i", (int) ans.size());
  cv::imwrite("region_growth.png", image);
}

////////////////////////////////////////////////////////////////////////////////

inline void test_string(const std::string & img_filename, int nb_times) {
  /*
     * to string
     */
  cv::Mat image = cv::imread(img_filename);
  vision_utils::Timer timer;
  std::string ans;
  for (int var = 0; var < nb_times; ++var)
    vision_utils::to_string(image, ans);
  printf("time after to_string():%f ms", ((float) timer.time()) / nb_times);
  printf("image:'%s'", vision_utils::infosImage(image).c_str());
  printf("vision_utils::to_string(image):size:%i, '%s...'",
         (int) ans.size(), ans.substr(0, 300).c_str());
  /* display */
  // for (size_t var = 0; var < ans.size(); ++var)
  // ROS_INFO("to_string:var %i=%i", var, ans.at(var));

  /*
     * from string
     */
  cv::Mat image_cp;
  timer.reset();
  for (int var = 0; var < nb_times; ++var) {
    image_cp.release();
    vision_utils::from_string(image_cp, ans);
  }
  printf("time after from_string():%f ms", ((float) timer.time()) / nb_times);
  printf("image_cp:'%s'", vision_utils::infosImage(image_cp).c_str());
  /* display */
  // uchar* mat_data_ptr = image_cp.data;
  // for (size_t var = 0; var < image_cp.cols * image_cp.rows
  // * image_cp.elemSize(); ++var)
  // ROS_INFO("from_string:var %i=%i", var, *mat_data_ptr++);
if (display) {
  cv::imshow("image_cp", image_cp);
  cv::waitKey(5000);
    } // end if display

  /*
     * check with the diff
     */
  cv::Mat image_diff;
  cv::absdiff(image, image_cp, image_diff);
  double diff_norm = cv::norm(image_diff);
  printf("diff_norm:'%f'", diff_norm);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, string_all_images) {
  test_string(vision_utils::IMG_DIR() + "balloon.png", 200);
  test_string(vision_utils::IMG_DIR() + "paleo.png", 200);
  test_string(vision_utils::IMG_DIR() + "rectangles.png", 200);
  test_string(vision_utils::IMG_DIR() + "depth/alberto1_rgb.png", 200);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, redim_content) {
  cv::Mat1b test = cv::imread(vision_utils::IMG_DIR() + "balloonBW.png" , CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat1b test_roi;
  test(cv::Rect(50, 50, 150, 150)).copyTo(test_roi);

  // cv::Mat test0 = cv::imread(vision_utils::IMG_DIR() + "rectangles.png" , CV_LOAD_IMAGE_GRAYSCALE);
  // cv::Mat test_roi ( test0, cv::Rect(2, 2, 8, 8));

  /* std::vector version */
  std::vector<cv::Point> comp, comp_resized;

  vision_utils::Timer timer;
  vision_utils::biggestComponent_vector2(test_roi, comp);
  timer.printTime("biggestComponent_vector2()\n");

  timer.reset();
  vision_utils::redimContent_vector_without_repetition
      (comp, comp_resized, cv::Rect(150, 150, 50, 50));
  timer.printTime("redimContent_vector_without_repetition()\n");
  // for (int i = 0; i < comp.size(); ++i)
  // printf("(" << comp.at(i).x << ", " << comp.at(i).y << ") ";
  // for (int i = 0; i < comp_resized.size(); ++i)
  // printf("(" << comp_resized.at(i).x << ", " << comp_resized.at(i).y << ") ";

  cv::Mat3b out (300, 300);
  out.setTo(0);
  vision_utils::drawListOfPoints(out, comp_resized, cv::Vec3b(255, 0 ,0));
  vision_utils::drawListOfPoints(out, comp, cv::Vec3b(0, 255 , 0));
if (display) {
  cv::imshow("out", out);
  cv::waitKey(0);
    } // end if display
  cv::imwrite("/tmp/out.png", out);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, redim_content2) {
  cv::Mat1b test(300, 300);
  test.setTo(0);
  int circle_diam = 75;
  cv::circle(test, cv::Point(150, 150), circle_diam, cv::Scalar(255), -1);

  // get the circle
  std::vector<cv::Point> comp;
  vision_utils::biggestComponent_vector2(test, comp);

  vision_utils::Timer timer;
  std::vector<cv::Point> comp_resized;
  int occur_max = 8;
  for (int occur = 1; occur <= occur_max; ++occur) {
    int new_width = circle_diam * occur / occur_max;
    int new_height = new_width / 2;
    int x = test.cols * occur / occur_max - new_width / 2;
    timer.reset();
    vision_utils::redimContent_vector_without_repetition
        (comp, comp_resized, cv::Rect(x, 50, new_width, new_height));
    timer.printTime("redimContent_vector_without_repetition");
    vision_utils::drawListOfPoints(test, comp_resized,
                                  (uchar) (255 * occur / occur_max));
  }
if (display) {
  cv::imshow("test", test);
  cv::waitKey(0);
  cv::imwrite("/tmp/test.png", test);
    } // end if display
}

////////////////////////////////////////////////////////////////////////////////

inline void test_redim_content3(cv::Mat1b & img_out) {
  std::vector<cv::Point> comp;
  vision_utils::biggestComponent_vector2(img_out, comp);

  // resize
  std::vector<cv::Point> comp_resized;
  for (int var = 0; var < 3; ++var) {
    cv::Rect bbox;
    if (var == 0)
      bbox = cv::Rect(0, 150, 101,  101);
    else if (var == 1)
      bbox = cv::Rect(0, 0, 101,  51);
    else if (var == 2)
      bbox = cv::Rect(350, 50, 26, 101);
    vision_utils::Timer timer;
    vision_utils::redimContent_vector_without_repetition
        (comp, comp_resized, bbox, (var >= 1));
    timer.printTime("redimContent_vector_without_repetition()\n");
    cv::rectangle(img_out, bbox, CV_RGB(100, 100, 100), 5);
    vision_utils::drawListOfPoints(img_out, comp_resized, (uchar) 255);
if (display) {
    cv::imshow("img_out", img_out);
    } // end if display
  } // end loop var
if (display) {
  cv::waitKey();
    } // end if display

  for(std::vector<cv::Point>::const_iterator pt = comp_resized.begin();
      pt != comp_resized.end() ; ++pt) {
    //printf("pt:'%s'", vision_utils::printP2(*pt).c_str() );
  } // end loop pt
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, redim_content3) {
  cv::Mat1b img_out(500, 500);
  // make a nice rectangle
  img_out.setTo(0);
  cv::rectangle(img_out, cv::Rect(150, 150, 101, 201), CV_RGB(200, 200, 200));
  test_redim_content3(img_out);

  // make a nice circle
  img_out.setTo(0);
  cv::circle(img_out, cv::Point(200, 200), 150, CV_RGB(200, 200, 200), 2);
  test_redim_content3(img_out);
}

////////////////////////////////////////////////////////////////////////////////

void test_remove_value(const std::string image_filename) {
  cv::Mat rgb, depth;
  vision_utils::read_rgb_depth_user_image_from_image_file
      (image_filename, &rgb, &depth, NULL);
  cv::Mat1b uchar_greyscale_buffer;
  cv::Mat src_float_clean_buffer;
  cv::Mat3b depth_illus;
  vision_utils::depth_image_to_vizualisation_color_image
      (depth, depth_illus, vision_utils::FULL_RGB_STRETCHED);
if (display) {
  cv::imshow("rgb", rgb);
  cv::imshow("depth_illus", depth_illus);
  cv::waitKey(0); cv::destroyAllWindows();
    } // end if display

  // remove nans
  cv::Mat1b img_uchar_with_no_nan_arr[4];
  cv::Mat1b inpaint_mask;
  double alpha_trans, beta_trans;
  vision_utils::convert_float_to_uchar
      (depth, uchar_greyscale_buffer, src_float_clean_buffer, alpha_trans, beta_trans);
  vision_utils::Timer timer;
  vision_utils::remove_value
      (uchar_greyscale_buffer, img_uchar_with_no_nan_arr[0], (uchar) 0,
      inpaint_mask, vision_utils::VALUE_REMOVAL_METHOD_DO_NOTHING);
  timer.printTime("VALUE_REMOVAL_METHOD_DO_NOTHING"); timer.reset();

  vision_utils::remove_value
      (uchar_greyscale_buffer, img_uchar_with_no_nan_arr[1], (uchar) 0,
      inpaint_mask, vision_utils::VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION);
  timer.printTime("VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION"); timer.reset();

  vision_utils::remove_value
      (uchar_greyscale_buffer, img_uchar_with_no_nan_arr[2], (uchar) 0,
      inpaint_mask, vision_utils::VALUE_REMOVAL_METHOD_INPAINT);
  timer.printTime("VALUE_REMOVAL_METHOD_INPAINT"); timer.reset();

  vision_utils::remove_value
      (uchar_greyscale_buffer, img_uchar_with_no_nan_arr[3], (uchar) 0,
      inpaint_mask, vision_utils::VALUE_REMOVAL_METHOD_AVERAGE_BORDER);
  timer.printTime("VALUE_REMOVAL_METHOD_AVERAGE_BORDER"); timer.reset();

if (display) {
  cv::imshow("VALUE_REMOVAL_METHOD_DO_NOTHING", img_uchar_with_no_nan_arr[0]);
  cv::imshow("VALUE_REMOVAL_METHOD_DIRECTIONAL_VALUE_PROPAGATION", img_uchar_with_no_nan_arr[1]);
  cv::imshow("VALUE_REMOVAL_METHOD_INPAINT", img_uchar_with_no_nan_arr[2]);
  cv::imshow("VALUE_REMOVAL_METHOD_AVERAGE_BORDER", img_uchar_with_no_nan_arr[3]);
  cv::waitKey(0); cv::destroyAllWindows();
    } // end if display
} // end test_remove_value();

TEST(TestSuite, value_remover) {
  test_remove_value(vision_utils::IMG_DIR() + "depth/inside1");
}

////////////////////////////////////////////////////////////////////////////////

void test_floodfill_edge_closer(const std::string filename,
                                const cv::Point & seed,
                                bool display_buffer = true) {
  cv::Mat1b img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE),
      img_closed = img.clone();
  vision_utils::CMatrix<bool> floodfill_buffer;
  vision_utils::Timer timer;
  vision_utils::FloodFillEdgeCloser closer;
  closer.close(img_closed, seed, (uchar) 0, 1.5f, .8f);
  timer.printTime("floodfill_edge_closer");
  if (display_buffer)
    printf("buffer:'%s'\n", floodfill_buffer.to_string(2).c_str());
  cv::Mat1b img_closed_scale;
  cv::resize(img_closed, img_closed_scale, cv::Size(), 15, 15, CV_INTER_NN);
  // cv::imwrite("img_closed.png", img_closed);
if (display) {
  cv::imshow("img", img);
  cv::imshow("img_closed", img_closed);
  cv::imshow("img_closed_scale", img_closed_scale);
  cv::waitKey(0);
    } // end if display
}

////////////////////////////////////////////////////////////////////////////////

void test_find_top_point_centered(const cv::Mat1b & img) {
  vision_utils::Timer timer;
  cv::Rect search_window;
  cv::Point top_point;
  for (unsigned int i = 0; i < 100; ++i)
    top_point = vision_utils::find_top_point_centered(img, true, .4, (uchar) 0, &search_window);
  timer.printTime_factor("find_top_point_centered", 100);

  cv::Mat3b img_illus;
  cv::cvtColor(img, img_illus, CV_GRAY2BGR);
  cv::rectangle(img_illus, search_window, CV_RGB(255, 0, 0));
  cv::circle(img_illus, top_point, 3, CV_RGB(0, 0, 255), 2);

if (display) {
  cv::imshow("img", img);
  cv::imshow("img_illus", img_illus);
  cv::waitKey(0); cv::destroyAllWindows();
    } // end if display
} // end test_find_top_point_centered();

////////////////////////////////////////////////////////////////////////////////

void test_find_top_point_centered(const std::string filename) {
  cv::Mat1b img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  test_find_top_point_centered(img);
}

////////////////////////////////////////////////////////////////////////////////

void test_propagative_floodfill(const std::string filename,
                                const cv::Point & seed) {
  cv::Mat1b img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  SEEN_BUFFER_TYPE seen_buffer_short;
  vision_utils::Timer timer;
  cv::Point seed_copy = seed;
  unsigned int ntimes = 100;
  for (unsigned int i = 0; i < ntimes; ++i)
    vision_utils::propagative_floodfill(img, seed_copy, seen_buffer_short);
  timer.printTime_factor("propagative_floodfill", ntimes);

  // paint seen_buffer
  cv::Mat1f seen_buffer_float_buffer;
  cv::Mat3b seen_buffer_illus;
  vision_utils::propagative_floodfill_seen_buffer_to_viz_image
      (seen_buffer_short, seen_buffer_float_buffer, seen_buffer_illus);
  cv::circle(seen_buffer_illus, seed_copy, 4, CV_RGB(0, 0, 255), 2);

if (display) {
  cv::imshow("img", img);
  cv::imshow("seen_buffer_illus", seen_buffer_illus);
  cv::waitKey(0); cv::destroyAllWindows();
    } // end if display
} // end test_propagative_floodfill();

////////////////////////////////////////////////////////////////////////////////

inline float my_lookup_function(const int buffer_val, const int, const int,
                                void* /*cookie*/) {
  return (int) (buffer_val / 20);
}

void test_propagative_floodfill_custom_lookup
(const std::string filename, const cv::Point & seed) {
  cv::Mat1b img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat1f lookup_result;
  cv::Point seed_copy = seed;
  SEEN_BUFFER_TYPE seen_buffer_short;
  vision_utils::Timer timer;
  for (unsigned int i = 0; i < 100; ++i)
    vision_utils::propagative_floodfill(img, seed_copy, seen_buffer_short, true,
                                       &my_lookup_function, &lookup_result, NULL);
  timer.printTime_factor("propagative_floodfill", 100);
  // paint seen_buffer
  cv::Mat1f seen_buffer_float_buffer;
  cv::Mat3b seen_buffer_illus;
  vision_utils::propagative_floodfill_seen_buffer_to_viz_image
      (seen_buffer_short, seen_buffer_float_buffer, seen_buffer_illus);
  cv::circle(seen_buffer_illus, seed, 4, CV_RGB(0, 0, 255), 1);
  cv::circle(seen_buffer_illus, seed_copy, 4, CV_RGB(0, 0, 255), 2);

  // paint lookup_result
  cv::Mat1b lookup_result_uchar;
  cv::Mat3b lookup_result_illus;
  lookup_result.convertTo(lookup_result_uchar, CV_8U);
  vision_utils::user_image_to_rgb(lookup_result_uchar, lookup_result_illus, 8);

if (display) {
  cv::imshow("img", img);
  cv::imshow("seen_buffer_illus", seen_buffer_illus);
  cv::imshow("lookup_result_illus", lookup_result_illus);
  cv::waitKey(0); cv::destroyAllWindows();
    } // end if display
}

////////////////////////////////////////////////////////////////////////////////

/**
 the data passed to the callback test_compute_pixel2meters_factor_mouse_cb()
 from the function  test_compute_pixel2meters_factor()
 */
struct pixel2metersData {
  image_geometry::PinholeCameraModel depth_camera_model, rgb_camera_model;
  cv::Mat rgb_img, depth_img;
  cv::Mat3b illus_img;
  std::string window_name;
}; // end struct pixel2metersData

void test_compute_pixel2meters_factor_mouse_cb
(int /*event*/, int x, int y, int /*flags*/, void* data_ptr)
{
  // printf("test_compute_pixel2meters_factor_mouse_cb(%i, %i)\n", x, y);
  pixel2metersData* data = (pixel2metersData*) data_ptr;
  // init output
  data->rgb_img.copyTo(data->illus_img);

  // get pixel2meters_factor
  double pixel2meters_factor = vision_utils::compute_pixel2meters_factor
                               (data->depth_img, data->depth_camera_model, cv::Point(x, y));

  // convert a distance of 50 cm to pixel (1 meter circle)
  int fifty_cm_radius = .5f / pixel2meters_factor;

  // show result
  printf("(%i, %i): pixel2meters_factor:%g, fifty_cm_radius:%i\n",
         x, y, pixel2meters_factor, fifty_cm_radius);
  if (isnan(pixel2meters_factor))
    return;
  cv::circle(data->illus_img, cv::Point(x, y), 2, CV_RGB(255, 0, 0), -1);
  cv::circle(data->illus_img, cv::Point(x, y), fifty_cm_radius, CV_RGB(255, 0, 0), 2);
if (display) {
  cv::imshow(data->window_name, data->illus_img);
    } // end if display
}

void test_compute_pixel2meters_factor(const std::string & rgb_depth_filename_prefix,
                                      const std::string & kinect_serial_number) {
  pixel2metersData data;

  // get camera model
  vision_utils::read_camera_model_files
      (kinect_serial_number, data.depth_camera_model, data.rgb_camera_model);

  // read depth and rgb files
  vision_utils::read_rgb_and_depth_image_from_image_file
      (rgb_depth_filename_prefix, &data.rgb_img, &data.depth_img);
  data.rgb_img.copyTo(data.illus_img);

  data.window_name = "test_compute_pixel2meters_factor";
if (display) {
  cv::namedWindow(data.window_name);
  cv::setMouseCallback(data.window_name, test_compute_pixel2meters_factor_mouse_cb, &data);
  cv::imshow("rgb_img", data.rgb_img);
  cv::imshow("depth_img_illus",
             vision_utils::depth_image_to_vizualisation_color_image(data.depth_img));
  while(true) {
    cv::imshow(data.window_name, data.illus_img);
    char c = cv::waitKey(50);
    if ((int) c == 27)
      break;
  }
    } // end if display
} // end test_compute_pixel2meters_factor();

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, floodfill_edge_closer) {
  cv::Point juggling1_pt = cv::Point(370, 70),
      //juggling2_pt = cv::Point(400, 70),
      juggling3_pt = cv::Point(360, 90);
  test_floodfill_edge_closer(vision_utils::IMG_DIR() + "broken_edge.png", cv::Point(11, 19));
  test_floodfill_edge_closer(vision_utils::IMG_DIR() + "broken_edge.png", cv::Point(9, 8));
  test_floodfill_edge_closer(vision_utils::IMG_DIR() + "depth/juggling1_broken_edge.png", juggling1_pt, false);
  test_floodfill_edge_closer(vision_utils::IMG_DIR() + "depth/juggling3_broken_edge.png", juggling3_pt, false);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, find_top_point_centered) {
  cv::Point juggling1_pt = cv::Point(370, 70),
      juggling2_pt = cv::Point(400, 70);
      //juggling3_pt = cv::Point(360, 90);
  test_find_top_point_centered(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png");
  test_find_top_point_centered(vision_utils::IMG_DIR() + "depth/juggling2_user_mask.png");
  test_propagative_floodfill(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png", juggling1_pt);
  test_propagative_floodfill(vision_utils::IMG_DIR() + "depth/juggling2_user_mask.png", juggling2_pt);
  test_propagative_floodfill_custom_lookup(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png", juggling1_pt);
  test_propagative_floodfill_custom_lookup(vision_utils::IMG_DIR() + "depth/juggling2_user_mask.png", juggling2_pt);
  test_compute_pixel2meters_factor(vision_utils::IMG_DIR() + "depth/juggling1", vision_utils::KINECT_SERIAL_LAB());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, propagative_floodfill) {
  cv::Point juggling1_pt = cv::Point(370, 70),
      juggling2_pt = cv::Point(400, 70);
  test_propagative_floodfill(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png", juggling1_pt);
  test_propagative_floodfill(vision_utils::IMG_DIR() + "depth/juggling2_user_mask.png", juggling2_pt);
  test_propagative_floodfill_custom_lookup(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png", juggling1_pt);
  test_propagative_floodfill_custom_lookup(vision_utils::IMG_DIR() + "depth/juggling2_user_mask.png", juggling2_pt);
  test_compute_pixel2meters_factor(vision_utils::IMG_DIR() + "depth/juggling1", vision_utils::KINECT_SERIAL_LAB());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_compute_pixel2meters_factor) {
  test_compute_pixel2meters_factor(vision_utils::IMG_DIR() + "depth/juggling1", vision_utils::KINECT_SERIAL_LAB());
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  //ros::Time::init();
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
