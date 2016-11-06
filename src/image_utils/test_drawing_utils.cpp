/*!
  \file        test_drawing_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/27

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
// AD
#include <vision_utils/draw_text_rotated.h>
#include <vision_utils/img_path.h>
#include "vision_utils/paste_images.h"
#include <vision_utils/resize_constrain_proportions.h>
#include "vision_utils/timer.h"
#include <iostream>

////////////////////////////////////////////////////////////////////////////////

inline void test_text_rotated() {
  printf("test_text_rotated()\n");
  cv::Mat3b img = cv::imread(vision_utils::IMG_DIR() + "balloon.png");
  cv::Mat1b buffer1, buffer2;
  for (unsigned int i = 0; i <= 10; ++i) {
    std::ostringstream text; text << "text" << i;
    vision_utils::draw_text_rotated(img, buffer1, buffer2, text.str(),
                                   cv::Point(50 * i, 100), 0.2 * i,
                                   CV_FONT_HERSHEY_DUPLEX, 1, CV_RGB(255, 0, 0));
    cv::imshow("buffer1", buffer1);
    // cv::imshow("buffer2", buffer2);
    cv::imshow("img", img);
    cv::waitKey(5000);
  } // end loop i
} // end test_text_rotated();

////////////////////////////////////////////////////////////////////////////////

inline void test_text_rotated2() {
  printf("test_text_rotated2()\n");
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
  cv::imshow("img", img); cv::waitKey(5000);
} // end test_text_rotated2();

////////////////////////////////////////////////////////////////////////////////

inline void test_resize_constrain_proportions() {
  printf("test_resize_constrain_proportions()\n");
  cv::Mat3b img = cv::imread(vision_utils::IMG_DIR() + "balloon.png");
  cv::Mat3b img_resize_if_bigger, img_resize_if_bigger2, img_resize_constrain_proportions;
  vision_utils::resize_if_bigger(img, img_resize_if_bigger, 100, 200);
  vision_utils::resize_if_bigger(img, img_resize_if_bigger2, 200, 100);
  vision_utils::resize_constrain_proportions(img, img_resize_constrain_proportions, 100, 200);

  cv::imshow("img_resize_if_bigger", img_resize_if_bigger);
  cv::imshow("img_resize_if_bigger2", img_resize_if_bigger2);
  cv::imshow("img_resize_constrain_proportions", img_resize_constrain_proportions);
  cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

inline void test_paste_image(const cv::Mat & bg, const cv::Mat & fg) {
  printf("test_paste_image() - use keys to move foreground image\n");
  int fg_x = (bg.cols - fg.cols) / 2, fg_y = (bg.rows - fg.rows) / 2;
  cv::Mat pasted;
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
}

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline void test_paste_images(const std::vector<cv::Mat_<_T> > & imgs) {
  printf("test_paste_images(%li images)\n", imgs.size());
  const int width1 = 100;
  const int height1 = 100;
  for (unsigned int horiz_idx = 0; horiz_idx < 2; ++horiz_idx) {
    bool horiz = (horiz_idx == 0);
    cv::Mat_<_T> out1;
    vision_utils::paste_images(imgs, out1, horiz, width1, height1, 5, false,
                              &vision_utils::int_to_number, std::vector<cv::Mat1b>(), false);
    cv::imshow("no headers, constrained width", out1);

    cv::Mat_<_T> out2;
    vision_utils::paste_images(imgs, out2, horiz, width1, height1, 5, true,
                              &vision_utils::int_to_number, std::vector<cv::Mat1b>(), false);
    cv::imshow("headers, constrained width", out2);

    cv::Mat_<_T> out3;
    vision_utils::paste_images(imgs, out3, horiz, width1, height1, 5, false,
                              &vision_utils::int_to_number, std::vector<cv::Mat1b>(), true);
    cv::imshow("no headers, no constrained width", out3);

    cv::Mat_<_T> out4;
    vision_utils::paste_images(imgs, out4, horiz, width1, height1, 5, true,
                              &vision_utils::int_to_number, std::vector<cv::Mat1b>(), true);
    cv::imshow("headers, no constrained width", out4);

    cv::waitKey(0);
  } // end loop horiz
}


inline void test_paste_images() {
  printf("test_paste_images()\n");
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

int main() {
#if 0
  cv::Mat foo = cv::Mat();
  cv::imshow("foo", foo);
  printf("ok!\n");
  return 0;
#endif

  int idx = 1;
  printf("%i: test_text_rotated()\n", idx++);
  printf("%i: test_text_rotated2()\n", idx++);
  printf("%i: test_resize_constrain_proportions()\n", idx++);
  printf("%i: test_paste_image(arnaud001, paleo)\n", idx++);
  printf("%i: test_paste_image(paleo, arnaud001)\n", idx++);
  printf("%i: test_paste_images()\n", idx++);

  printf("choice?\n");
  int choice = 6;
  std::cin >> choice;

  idx = 1;
  if (choice == idx++)
    test_text_rotated();
  else if (choice == idx++)
    test_text_rotated2();
  else if (choice == idx++)
    test_resize_constrain_proportions();
  else if (choice == idx++)
    test_paste_image(cv::imread(vision_utils::IMG_DIR() + "arnaud001.png"), cv::imread(vision_utils::IMG_DIR() + "paleo.png"));
  else if (choice == idx++)
    test_paste_image(cv::imread(vision_utils::IMG_DIR() + "paleo.png"), cv::imread(vision_utils::IMG_DIR() + "arnaud001.png"));
  else if (choice == idx++)
    test_paste_images();
}
