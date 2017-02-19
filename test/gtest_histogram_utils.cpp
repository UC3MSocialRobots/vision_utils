/*!
  \file        gtest_histogram_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/6

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

Tests for the histogram_utils namespace.
 */
bool display = false;
// Bring in gtest
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
// AD
#include "vision_utils/analyse_image.h"
#include <vision_utils/cast_to_string.h>
#include <vision_utils/colormaps.h>
#include <vision_utils/distance_hists.h>
#include "vision_utils/get_histogram.h"
#include "vision_utils/get_hue_histogram.h"
#include "vision_utils/get_vector_of_histograms.h"
#include <vision_utils/histogram2gnuplot.h>
#include <vision_utils/histogram_to_image.h>
#include "vision_utils/hist_to_string.h"
#include "vision_utils/hue_to_rgb_mat.h"
#include <vision_utils/img_path.h>
#include "vision_utils/matrix_testing.h"
#include <vision_utils/mean_std_dev_modulo.h>
#include "vision_utils/read_histogram_from_file.h"
#include <vision_utils/rectangle_intersection.h>
#include <vision_utils/rgb2hue.h>
#include "vision_utils/rgb_file2hue.h"
#include "vision_utils/timer.h"
#include "vision_utils/user_image_to_rgb.h"
#include "vision_utils/vector_of_histograms_to_image.h"
#include "vision_utils/write_histogram_to_file.h"


////////////////////////////////////////////////////////////////////////////////

namespace VU = vision_utils;

void test_io(const std::string filename) {
  cv::Mat1b img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  // compute histogram
  int nbins = 25, max_value = 255; // grey till 255
  VU::Histogram hist1 = VU::get_histogram(img, nbins, max_value);
  // save it
  std::string output_filename = "/tmp/foo.yaml";
  VU::write_histogram_to_file(hist1, output_filename);
  // now load it
  VU::Histogram hist2 = VU::read_histogram_from_file
      (output_filename);
  double dist = VU::distance_hists(hist1, hist2);
  ASSERT_TRUE(dist < 1E-2) << "dist:" << dist;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, io_balloon) { test_io(vision_utils::IMG_DIR() + "balloon.png"); }
TEST(TestSuite, io_paleo) { test_io(vision_utils::IMG_DIR() + "paleo.png"); }

////////////////////////////////////////////////////////////////////////////////

void test_mean_std_dev(const std::string & filename,
                       double exp_hue_mean, double exp_std_dev) {
  cv::Mat1b rgb = cv::imread(filename, CV_LOAD_IMAGE_COLOR), hue;
  cv::Mat3b hsv, h_illus;
  VU::Histogram h = VU::get_hue_histogram(rgb, hsv, hue, 15, 180);
  VU::histogram_to_image(h, h_illus, 400, 400, vision_utils::ratio2hue);
  double mean, std_dev;
  VU::mean_std_dev_modulo(h, 180, mean, std_dev);
  ASSERT_NEAR(mean, exp_hue_mean, 1);
  ASSERT_NEAR(std_dev, exp_std_dev, 1);

  // display
  if (display) {
    cv::imshow("h_illus", h_illus); cv::waitKey(0);
    cv::destroyAllWindows();
    printf("mean:%g, std_dev:%g\n", mean, std_dev);
    //gaussian_pdf2gnuplot(mean, std_dev, false, -50, 230);
    //VU::histogram2gnuplot(h, 180, -50, 230, mean, std_dev);
    VU::histogram2gnuplot(h, 180, 0, 180, mean, std_dev);
  } // end if display
}

//TEST(TestSuite, mean_std_dev_red_grad) { test_mean_std_dev(vision_utils::IMG_DIR() + "red_grad.png", 0.5, 13); }
TEST(TestSuite, mean_std_dev_notebook) { test_mean_std_dev(vision_utils::IMG_DIR() + "notebook.png", 13, 6); }
TEST(TestSuite, mean_std_dev_paleo) { test_mean_std_dev(vision_utils::IMG_DIR() + "paleo.png", 58, 26); }

////////////////////////////////////////////////////////////////////////////////

void test_hue_roi_mask(const cv::Mat3b & bgr) {
  cv::Mat3b hsv = bgr.clone();
  cv::Mat hue = vision_utils::rgb2hue(hsv);
  // compute histogram
  int nbins = 25, max_value = 180; // hue till 180
  vision_utils::Timer timer;
  VU::Histogram histo_full;
  for (unsigned int i = 0; i < 100; ++i)
    histo_full = VU::get_histogram(hue, nbins, max_value);
  timer.printTime_factor("get_histogram()", 100);
  ASSERT_TRUE(histo_full.rows == nbins);

  // just keep a ROI
  unsigned int cols = hue.cols, rows = hue.rows;
  cv::Rect ROI(cv::Point(cols/4, rows/4), cv::Point(cols/2, rows/2));
  VU::Histogram histo_roi = VU::get_histogram(hue(ROI), nbins, max_value, cv::Mat(), false);
  cv::Mat mask(hue.size(), CV_8UC1, cv::Scalar::all(0));
  mask(ROI).setTo(255);
  if (display) {
    cv::imshow("img", bgr);
    cv::imshow("hue", hue);
    cv::imshow("hue(ROI)", hue(ROI));
    cv::imshow("mask", mask);
    cv::waitKey(0);
  }
  VU::Histogram histo_mask = VU::get_histogram(hue, nbins, max_value, mask, false);
  ASSERT_TRUE(hue(ROI).cols == ROI.width);
  ASSERT_TRUE(hue(ROI).rows == ROI.height);
  ASSERT_TRUE(vision_utils::matrices_equal(histo_mask, histo_roi))
      << "histo_mask:" << histo_mask.t()
      << ", sum:" << cv::sum(histo_mask) << std::endl
      << "histo_roi:" << histo_roi.t()
      << ", sum:" << cv::sum(histo_roi) << std::endl
      << "histo_full:" << histo_full.t()
      << ", sum:" << cv::sum(histo_full) << std::endl;
}

void test_hue_roi_mask(const std::string & filename) {
  test_hue_roi_mask(cv::imread(vision_utils::IMG_DIR() + filename, CV_LOAD_IMAGE_COLOR));
}

TEST(TestSuite, hue_roi_mask_red) {
  cv::Mat3b red(640, 480);
  red.setTo(cv::Vec3b(0, 0, 255));
  test_hue_roi_mask(red);
}
TEST(TestSuite, hue_roi_mask_green) {
  cv::Mat3b green(640, 480);
  green.setTo(cv::Vec3b(0, 255, 0));
  test_hue_roi_mask(green);
}
TEST(TestSuite, hue_roi_mask_blue) {
  cv::Mat3b blue(640, 480);
  blue.setTo(cv::Vec3b(255, 0, 0));
  test_hue_roi_mask(blue);
}
TEST(TestSuite, hue_roi_mask_greenblue) {
  cv::Mat3b greenblue(100, 100, cv::Vec3b(255, 0, 0));
  greenblue(cv::Rect(20, 30, 40, 50)).setTo(cv::Vec3b(0, 255, 0));
  test_hue_roi_mask(greenblue);
}
//TEST(TestSuite, hue_roi_mask_red_grad) { test_hue_roi_mask("red_grad.png"); }
//TEST(TestSuite, hue_roi_mask_notebook) { test_hue_roi_mask("notebook.png"); }
//TEST(TestSuite, hue_roi_mask_paleo)    { test_hue_roi_mask("paleo.png"); }

////////////////////////////////////////////////////////////////////////////////

void test_histo_bw(const std::string filename) {
  cv::Mat1b img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  // compute histogram
  int nbins = 25, max_value = 255; // grey till 255
  VU::Histogram histo = VU::get_histogram(img, nbins, max_value);
  // draw it
  cv::Mat3b histo_illus;
  VU::histogram_to_image(histo, histo_illus, 800, 600);
  if (display) {
    cv::imwrite("histo_illus.png", histo_illus); printf("'histo_illus.png' saved.\n");
    cv::imshow("img", img);
    cv::imshow("histo_illus", histo_illus);
    cv::waitKey(0);
    cv::destroyAllWindows();
  } // end if display
}

TEST(TestSuite, histo_bw) {
  test_histo_bw(vision_utils::IMG_DIR() + "balloon.png");
}

////////////////////////////////////////////////////////////////////////////////

void test_histo_hue_mask(const std::string filename,
                         const std::string mask_filename = "") {
  cv::Mat3b img = cv::imread(filename, CV_LOAD_IMAGE_COLOR), img_hsv = img.clone();
  cv::Mat1b hue = vision_utils::rgb2hue(img_hsv);
  // compute histogram
  int nbins = 25, max_value = 180; // hue till 180
  VU::Histogram histo_full = VU::get_histogram(hue, nbins, max_value);

  // draw it
  cv::Mat3b histo_full_illus, histo_mask_illus;
  int illus_w = 320, illus_h = 240;
  VU::histogram_to_image
      (histo_full, histo_full_illus, illus_w, illus_h, vision_utils::ratio2hue);

  if (display) {
    cv::imshow("img", img);
    cv::imwrite("hue.png", vision_utils::hue2rgb(hue)); printf("'hue.png' saved.\n");
    cv::imshow("hue", vision_utils::hue2rgb(hue));
    cv::imshow("histo_full_illus", histo_full_illus);
    cv::imwrite("histo_full_illus.png", histo_full_illus); printf("'histo_full_illus.png' saved.\n");
  } // end if display

  if (mask_filename.size() > 0) {
    cv::Mat1b mask = cv::imread(mask_filename, CV_LOAD_IMAGE_GRAYSCALE);
    assert(mask.size() == img.size());
    VU::Histogram histo_mask = VU::get_histogram(hue, nbins, max_value, mask);
    VU::histogram_to_image
        (histo_mask, histo_mask_illus, illus_w, illus_h, vision_utils::ratio2hue);
    if (display) {
      cv::imshow("histo_mask_illus", histo_mask_illus);
      cv::imwrite("histo_mask_illus.png", histo_mask_illus); printf("'histo_mask_illus.png' saved.\n");
    } // end if display
  }
  if (display) {
    cv::waitKey(0);
    cv::destroyAllWindows();
  } // end if display
}

TEST(TestSuite, histo_hue_mask) {
  test_histo_hue_mask(vision_utils::IMG_DIR() + "balloon.png");
  test_histo_hue_mask(vision_utils::IMG_DIR() + "balloon.png", vision_utils::IMG_DIR() + "balloon_mask1.png");
}

////////////////////////////////////////////////////////////////////////////////

int roi_w = 80, roi_h = 50;
cv::Rect ROI2(0, 0, roi_w, roi_h);
bool recompute_hist = true;

void test_dist_roi_moving_mouse_mouse_cb(int /*event*/, int x, int y, int /*flags*/, void* /*param*/) {
  ROI2 = cv::Rect(x - roi_w / 2, y - roi_h / 2, roi_w, roi_h);
  recompute_hist = true;
}

void test_dist_roi_moving_mouse(const std::string filename) {
  cv::Mat3b img = cv::imread(filename, CV_LOAD_IMAGE_COLOR), img_hsv = img.clone();
  cv::Mat hue = vision_utils::rgb2hue(img_hsv);
  cv::Rect img_bbox(0, 0, img.cols, img.rows);
  // compute histogram - just keep a ROI
  int nbins = 25, max_value = 180; // hue till 180
  cv::Rect ROI(rand() % img.cols, rand() % img.rows,
               rand() % (2 * roi_w), rand() % (2 * roi_h));
  ROI = vision_utils::rectangle_intersection(ROI, img_bbox);
  std::vector<VU::Histogram> hists(2);
  hists[0] = VU::get_histogram(hue(ROI), nbins, max_value);
  printf("histograms[0]:'%s'\n", VU::hist_to_string(hists[0]).c_str());

  std::string window_name = "test_dist_roi_moving_mouse";
  cv::namedWindow(window_name);
  cv::setMouseCallback(window_name, test_dist_roi_moving_mouse_mouse_cb, &img);

  // draw it
  int hist_w = 320, hist_h = 240;
  cv::Mat3b hist_img(2 * hist_h + 60, hist_w);
  cv::Rect text_area_roi(cv::Point(0, 2 * hist_h), cv::Point(hist_img.cols, hist_img.rows));
  cv::Mat3b text_area = hist_img(text_area_roi);
  std::vector<bool> refresh_mask(2, true);
  VU::vector_of_histograms_to_image
      (hists, hist_img, hist_w, hist_h, vision_utils::ratio2hue, &refresh_mask);
  refresh_mask[0] = false; // no need to refresh this histogram anymore

  if (display) {
    cv::Mat3b img_illus;
    while (true) {
      // compute and draw moving histogram
      if (recompute_hist) {
        recompute_hist = false;
        // check the ROI does not get out of image
        ROI2 = vision_utils::rectangle_intersection(ROI2, img_bbox);
        // recompute histogram and draw it
        hists[1] = VU::get_histogram(hue(ROI2), nbins, max_value);
        // printf("\nhistograms[1]:'%s'\n", VU::hist_to_string(hists[1]).c_str());
        VU::vector_of_histograms_to_image
            (hists, hist_img, hist_w, hist_h, vision_utils::ratio2hue, &refresh_mask);

        // for each possible dist method, get distance and print it in text area of hist_img
        text_area =  cv::Vec3b(230, 230, 230);
        unsigned int n_methods = 4;
        int methods[] = {CV_COMP_INTERSECT, CV_COMP_CHISQR, CV_COMP_BHATTACHARYYA, CV_COMP_CORREL};
        const char* method_str[] = {"INTERSECT", "CHISQR", "BHATTACHARYYA", "CORREL"};
        for (unsigned int method_idx = 0; method_idx < n_methods; ++method_idx) {
          double dist = VU::distance_hists(hists[0], hists[1], methods[method_idx]);
          double dist_raw = VU::distance_hists(hists[0], hists[1], methods[method_idx], false);
          std::ostringstream text;
          text << method_str[method_idx] << ":" << std::setprecision(3) << dist
               << " (" << std::setprecision(3) << dist_raw << ")";
          //cv::Scalar bg_color = CV_RGB(255, 100, 100)
          cv::Scalar bg_color = vision_utils::ratio2red_green(dist);
          cv::rectangle(text_area,
                        cv::Rect(0, method_idx * text_area_roi.height / 4,
                                 text_area_roi.width * dist, text_area_roi.height / 4),
                        bg_color, -1);
          cv::putText(text_area, text.str(),
                      cv::Point(5, -3 + (method_idx+1) * text_area_roi.height / n_methods),
                      CV_FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(0));
        } // end loop method_idx

        // draw rectangles
        img.copyTo(img_illus);
        cv::rectangle(img_illus, ROI, CV_RGB(255, 0, 0), 2);
        cv::rectangle(img_illus, ROI2, CV_RGB(0, 0, 255), 2);
        cv::imshow(window_name, img_illus);
      } // end if (recompute_hist)

      cv::imshow("hist_img", hist_img);
      char c = cv::waitKey(50);
      if ((int) c == 27)
        break;
    } // end while (true)
    cv::destroyAllWindows();
  } // end if display
}

TEST(TestSuite, dist_roi_moving_mouse) {
  test_dist_roi_moving_mouse(vision_utils::IMG_DIR() + "balloon.png");
}

////////////////////////////////////////////////////////////////////////////////

void test_get_vector_of_histograms_illus
(const std::vector<cv::Mat> & hues,
 const std::vector<cv::Mat> & masks,
 const std::vector<VU::Histogram> & hists,
 const int /*nbins*/)
{
  // draw the histograms
  cv::Mat3b hists_img;
  VU::vector_of_histograms_to_image(hists, hists_img, 300, 200,
                                    vision_utils::ratio2hue);
  if (display) {
    // show everything
    for (unsigned int i = 0; i < hues.size(); ++i)
      cv::imshow(std::string("hue #") + vision_utils::cast_to_string(i),
                 vision_utils::hue2rgb(hues[i]));
    for (unsigned int i = 0; i < masks.size(); ++i)
      cv::imshow(std::string("mask #") + vision_utils::cast_to_string(i), masks[i]);
    cv::imwrite("hists_img.png", hists_img); printf("'hists_img.png' saved.\n");
    cv::imshow("hists_img", hists_img);
    cv::waitKey(0);
    cv::destroyAllWindows();
  } // end if display
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, get_vector_of_histograms_2_images_2_masks) {
  // read files
  std::vector<cv::Mat> hues;
  hues.push_back(vision_utils::rgb_file2hue(vision_utils::IMG_DIR() + "depth/juggling1_rgb.png"));
  hues.push_back(vision_utils::rgb_file2hue(vision_utils::IMG_DIR() + "depth/juggling2_rgb.png"));
  std::vector<cv::Mat> masks;
  masks.push_back(cv::imread(vision_utils::IMG_DIR() + "depth/juggling1_user_mask.png", CV_LOAD_IMAGE_GRAYSCALE));
  masks.push_back(cv::imread(vision_utils::IMG_DIR() + "depth/juggling2_user_mask.png", CV_LOAD_IMAGE_GRAYSCALE));
  // compute histograms
  int nbins = 25, max_value = 180; // hue till 180
  std::vector<VU::Histogram> hists;
  VU::get_vector_of_histograms(hues, hists, nbins, max_value, masks);
  test_get_vector_of_histograms_illus(hues, masks, hists, nbins);
} // end test_get_vector_of_histograms_2_images_2_masks();

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, get_vector_of_histograms_1_image_3_masks) {
  // read files
  cv::Mat hue = vision_utils::rgb_file2hue(vision_utils::IMG_DIR() + "balloon.png");
  std::vector<cv::Mat> masks;
  masks.push_back(cv::imread(vision_utils::IMG_DIR() + "balloon_mask1.png", CV_LOAD_IMAGE_GRAYSCALE));
  masks.push_back(cv::imread(vision_utils::IMG_DIR() + "balloon_mask2.png", CV_LOAD_IMAGE_GRAYSCALE));
  masks.push_back(cv::imread(vision_utils::IMG_DIR() + "balloon_mask3.png", CV_LOAD_IMAGE_GRAYSCALE));
  // compute histograms
  int nbins = 25, max_value = 180; // hue till 180
  std::vector<VU::Histogram> hists;
  VU::get_vector_of_histograms(hue, hists, nbins, max_value, masks);
  test_get_vector_of_histograms_illus
      (std::vector<cv::Mat>(1, hue), masks, hists, nbins);
} // end test_get_vector_of_histograms_1_image_3_masks();

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, get_vector_of_histograms_1_image_1_multimask) {
  // read files
  cv::Mat hue = vision_utils::rgb_file2hue(vision_utils::IMG_DIR() + "balloon.png");
  cv::Mat multimask = cv::imread(vision_utils::IMG_DIR() + "balloon_masks.png", CV_LOAD_IMAGE_GRAYSCALE);
  unsigned int n_masks = 3;
  // compute histograms
  int nbins = 25, max_value = 180; // hue till 180
  std::vector<VU::Histogram> hists;
  VU::get_vector_of_histograms(hue, hists, nbins, max_value, multimask, n_masks);
  // paint multimask
  cv::Mat3b multimask_illus;
  vision_utils::user_image_to_rgb(multimask, multimask_illus);
  cv::imwrite("multimask_illus.png", multimask_illus); printf("'multimask_illus.png' saved.\n");
  cv::imshow("multimask_illus", multimask_illus);
  // paint each mask separately
  std::vector<cv::Mat> masks;
  for (unsigned int mask_idx = 1; mask_idx <= n_masks; ++mask_idx)
    masks.push_back(multimask == mask_idx);
  test_get_vector_of_histograms_illus
      (std::vector<cv::Mat>(1, hue), masks, hists, nbins);
} // end test_get_vector_of_histograms_1_image_1_multimask();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  ros::init(argc, argv, "gtest");
  ros::NodeHandle nh_public;
  nh_public.param("display", display, display);
  printf("display:%i\n", display);
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
