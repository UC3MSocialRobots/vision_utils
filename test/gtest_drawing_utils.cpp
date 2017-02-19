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
#include <ros/ros.h>
// AD
#include "vision_utils/draw_text_rotated.h"
#include <vision_utils/img_path.h>
#include "vision_utils/matrix_testing.h"
#include "vision_utils/paste_images_gallery.h"
#include "vision_utils/paste_images.h"
#include <vision_utils/resize_constrain_proportions.h>
#include "vision_utils/timer.h"
#include "vision_utils/titlemaps.h"

bool display = false;

TEST(TestSuite, paste_images_gallery_empty) {
  std::vector<cv::Mat1b> ims;
  cv::Mat1b out;
  vision_utils::paste_images_gallery(ims, out, 2, (uchar) 0, true, CV_RGB(255, 255, 255));
  ASSERT_TRUE(out.empty());
}

////////////////////////////////////////////////////////////////////////////////

template<class T>
void check_imgs(const std::vector<cv::Mat_<T> > & ims,
                unsigned int gallerycols,
                T background_color) {
  cv::Mat_<T> out;
  vision_utils::paste_images_gallery(ims, out, gallerycols, background_color, false); // no border
if (display) {
  cv::imshow("out", out);
  cv::waitKey(0);
  cv::destroyAllWindows();
    } // end if display
  int cols1 = ims.front().cols, rows1 = ims.front().rows;
  unsigned int neededgalrows = std::ceil(1. * ims.size() / gallerycols);
  ASSERT_TRUE(out.cols == (int) gallerycols * cols1);
  ASSERT_TRUE(out.rows == (int) neededgalrows * rows1);
  for (unsigned int i = 0; i < ims.size(); ++i) {
    cv::Rect roi((i%gallerycols) * cols1, (i/gallerycols) * rows1, cols1, rows1);
    ASSERT_TRUE(vision_utils::matrices_equal(out(roi), ims[i]));
    // printf("out:%ix%i, roi %i:'%s'\n", out.cols, out.rows, i, vision_utils::print_rect(roi).c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, paste_images_gallery_arnaud) {
  std::vector<cv::Mat1b> ims;
  ims.push_back(cv::imread(vision_utils::IMG_DIR() + "faces/people_lab/arnaud_20.png", CV_LOAD_IMAGE_GRAYSCALE));
  ims.push_back(cv::imread(vision_utils::IMG_DIR() + "faces/people_lab/arnaud_21.png", CV_LOAD_IMAGE_GRAYSCALE));
  ims.push_back(cv::imread(vision_utils::IMG_DIR() + "faces/people_lab/arnaud_22.png", CV_LOAD_IMAGE_GRAYSCALE));
  ims.push_back(cv::imread(vision_utils::IMG_DIR() + "faces/people_lab/arnaud_23.png", CV_LOAD_IMAGE_GRAYSCALE));
  for (unsigned int i = 1; i <= 6; ++i)
    check_imgs(ims, i, (uchar) 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, paste_images_gallery_color) {
  std::vector<cv::Mat3b> ims;
  ims.push_back(cv::imread(vision_utils::IMG_DIR() + "depth/alberto1_rgb.png", CV_LOAD_IMAGE_COLOR));
  ims.push_back(cv::imread(vision_utils::IMG_DIR() + "depth/alberto2_rgb.png", CV_LOAD_IMAGE_COLOR));
  ims.push_back(cv::imread(vision_utils::IMG_DIR() + "depth/alvaro1_rgb.png", CV_LOAD_IMAGE_COLOR));
  ims.push_back(cv::imread(vision_utils::IMG_DIR() + "depth/alvaro2_rgb.png", CV_LOAD_IMAGE_COLOR));
  for (unsigned int i = 1; i <= 6; ++i)
    check_imgs(ims, i, cv::Vec3b(0,0,0));
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, text_rotated) {
  cv::Mat3b img = cv::imread(vision_utils::IMG_DIR() + "balloon.png");
  cv::Mat1b buffer1, buffer2;
  for (unsigned int i = 0; i <= 10; ++i) {
    std::ostringstream text; text << "text" << i;
    vision_utils::draw_text_rotated(img, buffer1, buffer2, text.str(),
                                   cv::Point(50 * i, 100), 0.2 * i,
                                   CV_FONT_HERSHEY_DUPLEX, 1, CV_RGB(255, 0, 0));
if (display) {
    cv::imshow("buffer1", buffer1);
    // cv::imshow("buffer2", buffer2);
    cv::imshow("img", img);
    cv::waitKey(5000);
    } // end if display
  } // end loop i
if (display) {
  cv::destroyAllWindows();
    } // end if display
} // end test_text_rotated();

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, text_rotated2) {
  cv::Mat3b img = cv::imread(vision_utils::IMG_DIR() + "balloon.png");
  cv::Mat1b buffer1, buffer2;
  vision_utils::Timer timer;
  unsigned int n_times = 1000;
  for (unsigned int i = 0; i <= n_times; ++i) {
    std::ostringstream text; text << "t" << i;
    vision_utils::draw_text_rotated(img, buffer1, buffer2, text.str(),
                                   cv::Point(rand()%img.cols, rand()%img.rows), drand48() * 2 * M_PI,
                                   CV_FONT_HERSHEY_DUPLEX, 1, CV_RGB(255, 0, 0));
    //  cv::putText(img, text.str(), cv::Point(rand()%img.cols, rand()%img.rows),
    //              CV_FONT_HERSHEY_DUPLEX, 1, CV_RGB(255, 0, 0));
    // cv::imshow("img", img); cv::waitKey(1);
  } // end loop i
  timer.printTime_factor("draw_text_rotated()", n_times);
if (display) {
  cv::imshow("img", img);
  cv::waitKey(0);
  cv::destroyAllWindows();
    } // end if display
} // end test_text_rotated2();

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, resize_constrain_proportions) {
  cv::Mat3b img = cv::imread(vision_utils::IMG_DIR() + "balloon.png");
  cv::Mat3b img_resize_if_bigger, img_resize_if_bigger2, img_resize_constrain_proportions;
  vision_utils::resize_if_bigger(img, img_resize_if_bigger, 100, 200);
  vision_utils::resize_if_bigger(img, img_resize_if_bigger2, 200, 100);
  vision_utils::resize_constrain_proportions(img, img_resize_constrain_proportions, 100, 200);

if (display) {
  cv::imshow("img_resize_if_bigger", img_resize_if_bigger);
  cv::imshow("img_resize_if_bigger2", img_resize_if_bigger2);
  cv::imshow("img_resize_constrain_proportions", img_resize_constrain_proportions);
  cv::waitKey(0);
  cv::destroyAllWindows();
    } // end if display
}

////////////////////////////////////////////////////////////////////////////////

inline void test_paste_image(const cv::Mat & bg, const cv::Mat & fg) {
  printf("test_paste_image() - use keys to move foreground image\n");
if (display) {
  cv::Mat pasted;
  int fg_x = (bg.cols - fg.cols) / 2, fg_y = (bg.rows - fg.rows) / 2;
  while (true) {
    bg.copyTo(pasted);
    vision_utils::paste_img(fg, pasted, fg_x, fg_y);
    cv::imshow("test_paste_image", pasted);
    char c = cv::waitKey(25);
    if ((int) c == 27)
      break;
    else if ((int) c == 82) // up
      fg_y -= 10;
    else if ((int) c == 84) // down
      fg_y += 10;
    else if ((int) c == 81) // left
      fg_x -= 10;
    else if ((int) c == 83) // right
      fg_x += 10;
  } // end while
  cv::destroyAllWindows();
    } // end if display
}

TEST(TestSuite, paste_image) {
  test_paste_image(cv::imread(vision_utils::IMG_DIR() + "arnaud001.png"),
                   cv::imread(vision_utils::IMG_DIR() + "paleo.png"));
  test_paste_image(cv::imread(vision_utils::IMG_DIR() + "paleo.png"),
                   cv::imread(vision_utils::IMG_DIR() + "arnaud001.png"));
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline void test_paste_images(const std::vector<cv::Mat_<_T> > & imgs) {
  //printf("test_paste_images(%i images)\n", imgs.size());
  const int width1 = 100;
  const int height1 = 100;
  for (unsigned int horiz_idx = 0; horiz_idx < 2; ++horiz_idx) {
    bool horiz = (horiz_idx == 0);
    cv::Mat_<_T> out1;
    vision_utils::paste_images(imgs, out1, horiz, width1, height1, 5, false,
                              &vision_utils::int_to_number, std::vector<cv::Mat1b>(), false);
    cv::Mat_<_T> out2;
    vision_utils::paste_images(imgs, out2, horiz, width1, height1, 5, true,
                              &vision_utils::int_to_number, std::vector<cv::Mat1b>(), false);
    cv::Mat_<_T> out3;
    vision_utils::paste_images(imgs, out3, horiz, width1, height1, 5, false,
                              &vision_utils::int_to_number, std::vector<cv::Mat1b>(), true);
    cv::Mat_<_T> out4;
    vision_utils::paste_images(imgs, out4, horiz, width1, height1, 5, true,
                              &vision_utils::int_to_number, std::vector<cv::Mat1b>(), true);
if (display) {
    cv::imshow("no headers, constrained width", out1);
    cv::imshow("headers, constrained width", out2);
    cv::imshow("no headers, no constrained width", out3);
    cv::imshow("headers, no constrained width", out4);
    cv::waitKey(0);
    cv::destroyAllWindows();
    } // end if display
  } // end loop horiz
}

TEST(TestSuite, paste_images) {
  std::vector<cv::Mat3b> imgs;
  test_paste_images(imgs);

  imgs.push_back(cv::imread(vision_utils::IMG_DIR() + "maggie.png"));
  test_paste_images(imgs);

  imgs.push_back(cv::Mat());
  test_paste_images(imgs);

  imgs.push_back(cv::imread(vision_utils::IMG_DIR() + "frenadol.png"));
  test_paste_images(imgs);

  imgs.push_back(cv::imread(vision_utils::IMG_DIR() + "paleo.png"));
  test_paste_images(imgs);
}

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


