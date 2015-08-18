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
//#define DISPLAY
// Bring in gtest
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
// AD
#include <visu_utils/histogram_utils.h>
#include <time/timer.h>
#include <test/matrix_testing.h>
#include <vision_utils/img_path.h>
// kinect
#include "kinect/user_image_to_rgb.h"


////////////////////////////////////////////////////////////////////////////////

namespace HU = histogram_utils;

void test_io(const std::string filename) {
  cv::Mat1b img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  // compute histogram
  int nbins = 25, max_value = 255; // grey till 255
  HU::Histogram hist1 = HU::get_histogram(img, nbins, max_value);
  // save it
  std::string output_filename = "/tmp/foo.yaml";
  HU::write_histogram_to_file(hist1, output_filename);
  // now load it
  HU::Histogram hist2 = HU::read_histogram_from_file
      (output_filename);
  double dist = HU::distance_hists(hist1, hist2);
  ASSERT_TRUE(dist < 1E-2) << "dist:" << dist;
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, io_balloon) { test_io(IMG_DIR "balloon.png"); }
TEST(TestSuite, io_paleo) { test_io(IMG_DIR "paleo.png"); }

////////////////////////////////////////////////////////////////////////////////

void test_mean_std_dev(const std::string & filename,
                       double exp_hue_mean, double exp_std_dev) {
  cv::Mat1b rgb = cv::imread(filename, CV_LOAD_IMAGE_COLOR), hue;
  cv::Mat3b hsv, h_illus;
  HU::Histogram h = HU::get_hue_histogram(rgb, hsv, hue, 15, 180);
  HU::histogram_to_image(h, h_illus, 400, 400, colormaps::ratio2hue);
  double mean, std_dev;
  HU::mean_std_dev_modulo(h, 180, mean, std_dev);
  ASSERT_NEAR(mean, exp_hue_mean, 1);
  ASSERT_NEAR(std_dev, exp_std_dev, 1);

  // display
#ifdef DISPLAY
  cv::imshow("h_illus", h_illus); cv::waitKey(0);
  cv::destroyAllWindows();
  printf("mean:%g, std_dev:%g\n", mean, std_dev);
  //gaussian_pdf2gnuplot(mean, std_dev, false, -50, 230);
  //HU::histogram2gnuplot(h, 180, -50, 230, mean, std_dev);
  HU::histogram2gnuplot(h, 180, 0, 180, mean, std_dev);
#endif // DISPLAY
}

//TEST(TestSuite, test_mean_std_dev_red_grad) { test_mean_std_dev(IMG_DIR "red_grad.png", 0.5, 13); }
TEST(TestSuite, test_mean_std_dev_notebook) { test_mean_std_dev(IMG_DIR "notebook.png", 13, 6); }
TEST(TestSuite, test_mean_std_dev_paleo) { test_mean_std_dev(IMG_DIR "paleo.png", 58, 26); }

////////////////////////////////////////////////////////////////////////////////

void test_hue_roi_mask(const std::string filename) {
  cv::Mat3b img = cv::imread(filename, CV_LOAD_IMAGE_COLOR), img_hsv = img.clone();
  cv::Mat hue = color_utils::rgb2hue(img_hsv);
  // compute histogram
  int nbins = 25, max_value = 180; // hue till 180
  Timer timer;
  HU::Histogram histo_full;
  for (unsigned int i = 0; i < 100; ++i)
    histo_full = HU::get_histogram(hue, nbins, max_value);
  timer.printTime_factor("get_histogram()", 100);
  ASSERT_TRUE(histo_full.rows == nbins);

  // just keep a ROI
  unsigned int cols = hue.cols, rows = hue.rows;
  cv::Rect ROI(cv::Point(cols/4, rows/4), cv::Point(cols/2, rows/2));
  HU::Histogram histo_roi = HU::get_histogram(hue(ROI), nbins, max_value);
  cv::Mat1b mask(img.size(), 0);
  mask(ROI) = 255;
  HU::Histogram histo_mask = HU::get_histogram(hue, nbins, max_value, mask);

  ASSERT_TRUE(matrix_testing::matrices_equal(histo_mask, histo_roi));
}

TEST(TestSuite, test_hue_roi_mask_red_grad) { test_hue_roi_mask(IMG_DIR "red_grad.png"); }
TEST(TestSuite, test_hue_roi_mask_notebook) { test_hue_roi_mask(IMG_DIR "notebook.png"); }
TEST(TestSuite, test_hue_roi_mask_paleo) { test_hue_roi_mask(IMG_DIR "paleo.png"); }

////////////////////////////////////////////////////////////////////////////////

void test_histo_bw(const std::string filename) {
  cv::Mat1b img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  // compute histogram
  int nbins = 25, max_value = 255; // grey till 255
  HU::Histogram histo = HU::get_histogram(img, nbins, max_value);
  // draw it
  cv::Mat3b histo_illus;
  HU::histogram_to_image(histo, histo_illus, 800, 600);
#ifdef DISPLAY
  cv::imwrite("histo_illus.png", histo_illus); printf("'histo_illus.png' saved.\n");
  cv::imshow("img", img);
  cv::imshow("histo_illus", histo_illus);
  cv::waitKey(0);
  cv::destroyAllWindows();
#endif // DISPLAY
}

TEST(TestSuite, histo_bw) {
  test_histo_bw(IMG_DIR "balloon.png");
}

////////////////////////////////////////////////////////////////////////////////

void test_histo_hue_mask(const std::string filename,
                         const std::string mask_filename = "") {
  cv::Mat3b img = cv::imread(filename, CV_LOAD_IMAGE_COLOR), img_hsv = img.clone();
  cv::Mat1b hue = color_utils::rgb2hue(img_hsv);
  // compute histogram
  int nbins = 25, max_value = 180; // hue till 180
  HU::Histogram histo_full = HU::get_histogram(hue, nbins, max_value);

  // draw it
  cv::Mat3b histo_full_illus, histo_mask_illus;
  int illus_w = 320, illus_h = 240;
  HU::histogram_to_image
      (histo_full, histo_full_illus, illus_w, illus_h, colormaps::ratio2hue);

#ifdef DISPLAY
  cv::imshow("img", img);
  cv::imwrite("hue.png", color_utils::hue2rgb(hue)); printf("'hue.png' saved.\n");
  cv::imshow("hue", color_utils::hue2rgb(hue));
  cv::imshow("histo_full_illus", histo_full_illus);
  cv::imwrite("histo_full_illus.png", histo_full_illus); printf("'histo_full_illus.png' saved.\n");
#endif // DISPLAY

  if (mask_filename.size() > 0) {
    cv::Mat1b mask = cv::imread(mask_filename, CV_LOAD_IMAGE_GRAYSCALE);
    assert(mask.size() == img.size());
    HU::Histogram histo_mask = HU::get_histogram(hue, nbins, max_value, mask);
    HU::histogram_to_image
        (histo_mask, histo_mask_illus, illus_w, illus_h, colormaps::ratio2hue);
#ifdef DISPLAY
    cv::imshow("histo_mask_illus", histo_mask_illus);
    cv::imwrite("histo_mask_illus.png", histo_mask_illus); printf("'histo_mask_illus.png' saved.\n");
#endif // DISPLAY
  }
#ifdef DISPLAY
  cv::waitKey(0);
  cv::destroyAllWindows();
#endif // DISPLAY
}

TEST(TestSuite, histo_hue_mask) {
  test_histo_hue_mask(IMG_DIR "balloon.png");
  test_histo_hue_mask(IMG_DIR "balloon.png", IMG_DIR "balloon_mask1.png");
}

////////////////////////////////////////////////////////////////////////////////

int roi_w = 80, roi_h = 50;
cv::Rect ROI2(0, 0, roi_w, roi_h);
bool recompute_hist = true;

void test_dist_roi_moving_mouse_mouse_cb(int event, int x, int y, int flags, void* param) {
  ROI2 = cv::Rect(x - roi_w / 2, y - roi_h / 2, roi_w, roi_h);
  recompute_hist = true;
}

void test_dist_roi_moving_mouse(const std::string filename) {
  cv::Mat3b img = cv::imread(filename, CV_LOAD_IMAGE_COLOR), img_hsv = img.clone();
  cv::Mat hue = color_utils::rgb2hue(img_hsv);
  cv::Rect img_bbox(0, 0, img.cols, img.rows);
  // compute histogram - just keep a ROI
  int nbins = 25, max_value = 180; // hue till 180
  cv::Rect ROI(rand() % img.cols, rand() % img.rows,
               rand() % (2 * roi_w), rand() % (2 * roi_h));
  ROI = geometry_utils::rectangle_intersection(ROI, img_bbox);
  std::vector<HU::Histogram> hists(2);
  hists[0] = HU::get_histogram(hue(ROI), nbins, max_value);
  printf("histograms[0]:'%s'\n", HU::hist_to_string(hists[0]).c_str());

  std::string window_name = "test_dist_roi_moving_mouse";
  cv::namedWindow(window_name);
  cv::setMouseCallback(window_name, test_dist_roi_moving_mouse_mouse_cb, &img);

  // draw it
  int hist_w = 320, hist_h = 240;
  cv::Mat3b hist_img(2 * hist_h + 60, hist_w);
  cv::Rect text_area_roi(cv::Point(0, 2 * hist_h), cv::Point(hist_img.cols, hist_img.rows));
  cv::Mat3b text_area = hist_img(text_area_roi);
  std::vector<bool> refresh_mask(2, true);
  HU::vector_of_histograms_to_image
      (hists, hist_img, hist_w, hist_h, colormaps::ratio2hue, &refresh_mask);
  refresh_mask[0] = false; // no need to refresh this histogram anymore

#ifdef DISPLAY
  cv::Mat3b img_illus;
  while (true) {
    // compute and draw moving histogram
    if (recompute_hist) {
      recompute_hist = false;
      // check the ROI does not get out of image
      ROI2 = geometry_utils::rectangle_intersection(ROI2, img_bbox);
      // recompute histogram and draw it
      hists[1] = HU::get_histogram(hue(ROI2), nbins, max_value);
      // printf("\nhistograms[1]:'%s'\n", HU::hist_to_string(hists[1]).c_str());
      HU::vector_of_histograms_to_image
          (hists, hist_img, hist_w, hist_h, colormaps::ratio2hue, &refresh_mask);

      // for each possible dist method, get distance and print it in text area of hist_img
      text_area =  cv::Vec3b(230, 230, 230);
      unsigned int n_methods = 4;
      int methods[] = {CV_COMP_INTERSECT, CV_COMP_CHISQR, CV_COMP_BHATTACHARYYA, CV_COMP_CORREL};
      const char* method_str[] = {"INTERSECT", "CHISQR", "BHATTACHARYYA", "CORREL"};
      for (unsigned int method_idx = 0; method_idx < n_methods; ++method_idx) {
        double dist = HU::distance_hists(hists[0], hists[1], methods[method_idx]);
        double dist_raw = HU::distance_hists(hists[0], hists[1], methods[method_idx], false);
        std::ostringstream text;
        text << method_str[method_idx] << ":" << std::setprecision(3) << dist
             << " (" << std::setprecision(3) << dist_raw << ")";
        //cv::Scalar bg_color = CV_RGB(255, 100, 100)
        cv::Scalar bg_color = colormaps::ratio2red_green(dist);
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
#endif // DISPLAY
}

TEST(TestSuite, dist_roi_moving_mouse) {
  test_dist_roi_moving_mouse(IMG_DIR "balloon.png");
}

////////////////////////////////////////////////////////////////////////////////

void test_get_vector_of_histograms_illus
(const std::vector<cv::Mat> & hues,
 const std::vector<cv::Mat> & masks,
 const std::vector<HU::Histogram> & hists,
 const int nbins)
{
  // draw the histograms
  cv::Mat3b hists_img;
  HU::vector_of_histograms_to_image(hists, hists_img, 300, 200,
                                    colormaps::ratio2hue);
#ifdef DISPLAY
  // show everything
  for (unsigned int i = 0; i < hues.size(); ++i)
    cv::imshow(std::string("hue #") + StringUtils::cast_to_string(i),
               color_utils::hue2rgb(hues[i]));
  for (unsigned int i = 0; i < masks.size(); ++i)
    cv::imshow(std::string("mask #") + StringUtils::cast_to_string(i), masks[i]);
  cv::imwrite("hists_img.png", hists_img); printf("'hists_img.png' saved.\n");
  cv::imshow("hists_img", hists_img);
  cv::waitKey(0);
  cv::destroyAllWindows();
#endif // DISPLAY
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, get_vector_of_histograms_2_images_2_masks) {
  // read files
  std::vector<cv::Mat> hues;
  hues.push_back(color_utils::rgb_file2hue(IMG_DIR "depth/juggling1_rgb.png"));
  hues.push_back(color_utils::rgb_file2hue(IMG_DIR "depth/juggling2_rgb.png"));
  std::vector<cv::Mat> masks;
  masks.push_back(cv::imread(IMG_DIR "depth/juggling1_user_mask.png", CV_LOAD_IMAGE_GRAYSCALE));
  masks.push_back(cv::imread(IMG_DIR "depth/juggling2_user_mask.png", CV_LOAD_IMAGE_GRAYSCALE));
  // compute histograms
  int nbins = 25, max_value = 180; // hue till 180
  std::vector<HU::Histogram> hists;
  HU::get_vector_of_histograms(hues, hists, nbins, max_value, masks);
  test_get_vector_of_histograms_illus(hues, masks, hists, nbins);
} // end test_get_vector_of_histograms_2_images_2_masks();

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, get_vector_of_histograms_1_image_3_masks) {
  // read files
  cv::Mat hue = color_utils::rgb_file2hue(IMG_DIR "balloon.png");
  std::vector<cv::Mat> masks;
  masks.push_back(cv::imread(IMG_DIR "balloon_mask1.png", CV_LOAD_IMAGE_GRAYSCALE));
  masks.push_back(cv::imread(IMG_DIR "balloon_mask2.png", CV_LOAD_IMAGE_GRAYSCALE));
  masks.push_back(cv::imread(IMG_DIR "balloon_mask3.png", CV_LOAD_IMAGE_GRAYSCALE));
  // compute histograms
  int nbins = 25, max_value = 180; // hue till 180
  std::vector<HU::Histogram> hists;
  HU::get_vector_of_histograms(hue, hists, nbins, max_value, masks);
  test_get_vector_of_histograms_illus
      (std::vector<cv::Mat>(1, hue), masks, hists, nbins);
} // end test_get_vector_of_histograms_1_image_3_masks();

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, get_vector_of_histograms_1_image_1_multimask) {
  // read files
  cv::Mat hue = color_utils::rgb_file2hue(IMG_DIR "balloon.png");
  cv::Mat multimask = cv::imread(IMG_DIR "balloon_masks.png", CV_LOAD_IMAGE_GRAYSCALE);
  unsigned int n_masks = 3;
  // compute histograms
  int nbins = 25, max_value = 180; // hue till 180
  std::vector<HU::Histogram> hists;
  HU::get_vector_of_histograms(hue, hists, nbins, max_value, multimask, n_masks);
  // paint multimask
  cv::Mat3b multimask_illus;
  user_image_to_rgb(multimask, multimask_illus);
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
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
