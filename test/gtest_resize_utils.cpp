/*!
  \file        gtest_resize_utils.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/28

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

#include "vision_utils/timer.h"
#include <vision_utils/img_path.h>
#include <vision_utils/scale_img_forward_backward.h>
#include <vision_utils/scale_img_warp.h>

////////////////////////////////////////////////////////////////////////////////

void benchmark(const std::string & imgfile,
               const float & xscale = 1, const int & xoffset = 0,
               const float & yscale = 1, const int & yoffset = 0,
               unsigned int ntimes = 100) {
  printf("scale(%g, %g), offset(%i, %i)\n", xscale, yscale, xoffset, yoffset);
  cv::Mat3b img = cv::imread(imgfile, CV_LOAD_IMAGE_COLOR), img_out(img.size());
  vision_utils::Timer timer;
  for (unsigned int time = 0; time < ntimes; ++time)
    vision_utils::scale_img_forward_backward(img, img_out, xscale, xoffset, yscale, yoffset, true);
  timer.printTime_factor("scale_img_forward()", ntimes);

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    vision_utils::scale_img_forward_backward(img, img_out, xscale, xoffset, yscale, yoffset, false);
  timer.printTime_factor("scale_img_backward()", ntimes);

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    vision_utils::scale_img_warp(img, img_out, xscale, xoffset, yscale, yoffset);
  timer.printTime_factor("scale_img_warp()", ntimes);

}

TEST(TestSuite, benchmark) {
  unsigned int ntimes = 20;
  benchmark(vision_utils::IMG_DIR() + "depth/alberto1_rgb.png", 1,   50, 1,   30, ntimes);
  benchmark(vision_utils::IMG_DIR() + "depth/alberto1_rgb.png", .5,  50, .8,  30, ntimes);
  benchmark(vision_utils::IMG_DIR() + "depth/alberto1_rgb.png", 1.5, 50, 1.2, 30, ntimes);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
