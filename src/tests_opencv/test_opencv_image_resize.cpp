/*!
  \file        test_opencv_image_resize.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/10/16

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

\section Parameters
  - \b "foo"
        [string] (default: "bar")
        Description of the parameter.

\section Subscriptions
  - \b "/foo"
        [xxx]
        Descrption of the subscription

\section Publications
  - \b "~foo"
        [xxx]
        Descrption of the publication

 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "vision_utils/timer.h"
#include <vision_utils/img_path.h>

#define NTIMES 100

////////////////////////////////////////////////////////////////////////////////

void process_image(cv::Mat3b & image, double scale) {
  printf("process_image()");
  cv::Mat3b image_resized;
  vision_utils::Timer timer;

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    cv::resize(image, image_resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
  printf("scale:%g, INTER_NEAREST (%i times) : %g ms.",
              scale, NTIMES, timer.getTimeMilliseconds() / NTIMES);
  cv::imshow("INTER_NEAREST", image_resized);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    cv::resize(image, image_resized, cv::Size(), scale, scale, cv::INTER_LINEAR);
  printf("scale:%g, INTER_LINEAR (%i times) : %g ms.",
              scale, NTIMES, timer.getTimeMilliseconds() / NTIMES);
  cv::imshow("INTER_LINEAR", image_resized);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    cv::resize(image, image_resized, cv::Size(), scale, scale, cv::INTER_CUBIC);
  printf("scale:%g, INTER_CUBIC (%i times) : %g ms.",
              scale, NTIMES, timer.getTimeMilliseconds() / NTIMES);
  cv::imshow("INTER_CUBIC", image_resized);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    cv::resize(image, image_resized, cv::Size(), scale, scale, cv::INTER_AREA);
  printf("scale:%g, INTER_AREA (%i times) : %g ms.",
              scale, NTIMES, timer.getTimeMilliseconds() / NTIMES);
  cv::imshow("INTER_AREA", image_resized);

  timer.reset();
  for (int time = 0; time < NTIMES; ++time)
    cv::resize(image, image_resized, cv::Size(), scale, scale, cv::INTER_LANCZOS4);
  printf("scale:%g, INTER_LANCZOS4 (%i times) : %g ms.",
              scale, NTIMES, timer.getTimeMilliseconds() / NTIMES);
  cv::imshow("INTER_LANCZOS4", image_resized);

  cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

int main() {
    cv::Mat3b image = cv::imread(vision_utils::IMG_DIR() + "pz/pz01.jpg", CV_LOAD_IMAGE_COLOR);
    process_image(image, 0.6);
    printf("\n");
    process_image(image, 1.6);
}

