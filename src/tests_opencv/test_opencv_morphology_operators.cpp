/*!
  \file        test_opencv_morphology_operators.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/21

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

Some tests for the morphology functions of OpenCV.
http://docs.opencv.org/modules/imgproc/doc/filtering.html?highlight=dilate#cv2.dilate

 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void draw_rectangles(cv::Mat1b & dst, const std::vector<cv::Rect> & recs,
                     const int thickness = 1, const int color = 180) {
  for (unsigned int rect_idx = 1; rect_idx < recs.size(); ++rect_idx)
    cv::rectangle(dst, recs[rect_idx], cv::Scalar::all(color), thickness);
}

////////////////////////////////////////////////////////////////////////////////

int main() {
  srand(time(NULL));

  unsigned int w = 400, h = 400;
  cv::Mat1b in(h, w), out;
  // generate morph kernel
  cv::Mat kernel(10, 10, CV_8U, 255);

  while (true) {
    // generate random rectangles
    std::vector<cv::Rect> rectangles;
    for (unsigned int rect_idx = 1; rect_idx < 10; ++rect_idx)
      rectangles.push_back(cv::Rect
                           (rand() % w,
                            rand() % h,
                            rand() % 50,
                            rand() % 50));

    // draw input image
    in = 0;
    draw_rectangles(in, rectangles, -1, 255);
    cv::imshow("in", in);

    cv::erode(in, out, kernel);
    draw_rectangles(out, rectangles, 1);
    cv::imshow("erode", out);

    cv::dilate(in, out, kernel);
    draw_rectangles(out, rectangles, 1);
    cv::imshow("dilate", out);

    cv::morphologyEx(in, out, cv::MORPH_OPEN, kernel);
    draw_rectangles(out, rectangles, 1);
    cv::imshow("open", out);

    cv::morphologyEx(in, out, cv::MORPH_CLOSE, kernel);
    draw_rectangles(out, rectangles, 1);
    cv::imshow("close", out);

    cv::morphologyEx(in, out, cv::MORPH_GRADIENT, kernel);
    draw_rectangles(out, rectangles, 1);
    cv::imshow("gradient", out);

    cv::morphologyEx(in, out, cv::MORPH_TOPHAT, kernel);
    draw_rectangles(out, rectangles, 1);
    cv::imshow("tophat", out);

    cv::morphologyEx(in, out, cv::MORPH_BLACKHAT, kernel);
    draw_rectangles(out, rectangles, 1);
    cv::imshow("blackhat", out);

    char c = cv::waitKey(0);
    if ((int) c == 27)
      break;
  } // end while (true)
  return 0;
}


