/*!
  \file        test_opencv_text_fonts.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/23

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

A basic rendering example for all fonts of OpenCV.

 */

#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION > 2
#include "opencv2/imgproc/imgproc.hpp" // for versions 2.4 and +
#endif // CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >= 4

int main() {
  cv::Mat3b img(600, 800);
  img.setTo(255);
  int h = 50, i = 1;
  ++i; cv::putText(img, "CV_FONT_HERSHEY_PLAIN", cv::Point(10, h * i),
                   CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));
  ++i; cv::putText(img, "CV_FONT_HERSHEY_SIMPLEX", cv::Point(10, h * i),
                   CV_FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 0, 0));
  ++i; cv::putText(img, "CV_FONT_HERSHEY_DUPLEX", cv::Point(10, h * i),
                   CV_FONT_HERSHEY_DUPLEX, 1, CV_RGB(0, 0, 0));
  ++i; cv::putText(img, "CV_FONT_HERSHEY_COMPLEX", cv::Point(10, h * i),
                   CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0, 0));
  ++i; cv::putText(img, "CV_FONT_HERSHEY_TRIPLEX", cv::Point(10, h * i),
                   CV_FONT_HERSHEY_TRIPLEX, 1, CV_RGB(0, 0, 0));
  ++i; cv::putText(img, "CV_FONT_HERSHEY_COMPLEX_SMALL", cv::Point(10, h * i),
                   CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0, 0, 0));
  ++i; cv::putText(img, "CV_FONT_HERSHEY_SCRIPT_SIMPLEX", cv::Point(10, h * i),
                   CV_FONT_HERSHEY_SCRIPT_SIMPLEX, 1, CV_RGB(0, 0, 0));
  ++i; cv::putText(img, "CV_FONT_HERSHEY_SCRIPT_COMPLEX", cv::Point(10, h * i),
                   CV_FONT_HERSHEY_SCRIPT_COMPLEX, 1, CV_RGB(0, 0, 0));
  cv::imshow("img", img);
  cv::waitKey();
  return 0;
}
