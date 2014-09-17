/*!
  \file        gtest_drawing_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/12/6

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
#include <opencv2/highgui/highgui.hpp>
// Bring in gtest
#include <gtest/gtest.h>
// AD
#include <vision_utils/img_path.h>
#include "test/matrix_testing.h"
#include "image_utils/drawing_utils.h"

// #define DISPLAY

TEST(TestSuite, paste_images_gallery_empty) {
  std::vector<cv::Mat1b> ims;
  cv::Mat1b out;
  image_utils::paste_images_gallery(ims, out, 2, (uchar) 0, true, CV_RGB(255, 255, 255));
  ASSERT_TRUE(out.empty());
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
void check_imgs(const std::vector<cv::Mat_<T> > & ims,
                unsigned int gallerycols,
                T background_color) {
  cv::Mat_<T> out;
  image_utils::paste_images_gallery(ims, out, gallerycols, background_color, false); // no border
#ifdef DISPLAY
  cv::imshow("out", out); cv::waitKey(0);
#endif // DISPLAY
  int cols1 = ims.front().cols, rows1 = ims.front().rows;
  unsigned int neededgalrows = std::ceil(1. * ims.size() / gallerycols);
  ASSERT_TRUE(out.cols == (int) gallerycols * cols1);
  ASSERT_TRUE(out.rows == (int) neededgalrows * rows1);
  for (unsigned int i = 0; i < ims.size(); ++i) {
    cv::Rect roi((i%gallerycols) * cols1, (i/gallerycols) * rows1, cols1, rows1);
    ASSERT_TRUE(matrix_testing::matrices_equal(out(roi), ims[i]));
    // printf("out:%ix%i, roi %i:'%s'\n", out.cols, out.rows, i, geometry_utils::print_rect(roi).c_str());
  }
}

TEST(TestSuite, paste_images_gallery_arnaud) {
  std::vector<cv::Mat1b> ims;
  ims.push_back(cv::imread(IMG_DIR "faces/people_lab/arnaud_20.png", CV_LOAD_IMAGE_GRAYSCALE));
  ims.push_back(cv::imread(IMG_DIR "faces/people_lab/arnaud_21.png", CV_LOAD_IMAGE_GRAYSCALE));
  ims.push_back(cv::imread(IMG_DIR "faces/people_lab/arnaud_22.png", CV_LOAD_IMAGE_GRAYSCALE));
  ims.push_back(cv::imread(IMG_DIR "faces/people_lab/arnaud_23.png", CV_LOAD_IMAGE_GRAYSCALE));
  for (unsigned int i = 1; i <= 6; ++i)
    check_imgs(ims, i, (uchar) 0);
}

TEST(TestSuite, paste_images_gallery_color) {
  std::vector<cv::Mat3b> ims;
  ims.push_back(cv::imread(IMG_DIR "depth/alberto1_rgb.png", CV_LOAD_IMAGE_COLOR));
  ims.push_back(cv::imread(IMG_DIR "depth/alberto2_rgb.png", CV_LOAD_IMAGE_COLOR));
  ims.push_back(cv::imread(IMG_DIR "depth/alvaro1_rgb.png", CV_LOAD_IMAGE_COLOR));
  ims.push_back(cv::imread(IMG_DIR "depth/alvaro2_rgb.png", CV_LOAD_IMAGE_COLOR));
  for (unsigned int i = 1; i <= 6; ++i)
    check_imgs(ims, i, cv::Vec3b(0,0,0));
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


