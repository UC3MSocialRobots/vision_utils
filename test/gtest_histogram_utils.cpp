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

\todo Description of the file

 */
// Bring in gtest
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
// AD
#include <visu_utils/histogram_utils.h>
#include <time/timer.h>
#include <test/matrix_testing.h>
#include <vision_utils/img_path.h>

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
#if 0
  cv::imshow("h_illus", h_illus); cv::waitKey(0);
  printf("mean:%g, std_dev:%g\n", mean, std_dev);
  //gaussian_pdf2gnuplot(mean, std_dev, false, -50, 230);
  //HU::histogram2gnuplot(h, 180, -50, 230, mean, std_dev);
  HU::histogram2gnuplot(h, 180, 0, 180, mean, std_dev);
#endif
}

TEST(TestSuite, test_mean_std_dev_red_grad) { test_mean_std_dev(IMG_DIR "red_grad.png", 0.5, 13); }
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

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
