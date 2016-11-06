/*!
  \file        test_opencv_image_io_speed.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/8/1

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

Some speed tests for OpenCV imread, imwrite functions
 */

#include <opencv2/highgui/highgui.hpp>
#include <vision_utils/exec_system_get_output.h>
#include "vision_utils/float_image_generator.h"
#include <vision_utils/img_path.h>
#include "vision_utils/timer.h"


inline std::string file_size(const std::string & filename) {
  std::ostringstream command;
  command << "du -h " << filename << " | cut -f 1";
  return vision_utils::exec_system_get_output(command.str().c_str());
}

////////////////////////////////////////////////////////////////////////////////

void test_color_image(const cv::Mat & rgb) {
  printf("test_color_image(%i x %i)\n", rgb.cols, rgb.rows);
  unsigned int ntimes = 10;
  std::vector<int> compression_params;
  vision_utils::Timer timer;

  compression_params.clear();
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);
  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    cv::imwrite("/tmp/foo.png", rgb, compression_params);
  printf("PNG writing, compression 0: \t\t%g ms, %s", timer.time() / ntimes,
              file_size("/tmp/foo.png").c_str());

  compression_params.clear();
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(3);
  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    cv::imwrite("/tmp/foo.png", rgb, compression_params);
  printf("PNG writing, compression 3 (default): \t%g ms, %s", timer.time() / ntimes,
              file_size("/tmp/foo.png").c_str());

  compression_params.clear();
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    cv::imwrite("/tmp/foo.png", rgb, compression_params);
  printf("PNG writing, compression 9: \t\t%g ms, %s", timer.time() / ntimes,
              file_size("/tmp/foo.png").c_str());

  //  PBM is for bitmaps (black and white, no grays)[3]
  //  PGM is for grayscale
  //  PPM is for "pixmaps" which represent full RGB color.[4]
  for (unsigned int binary = 0; binary <= 1; ++binary) {
    compression_params.clear();
    compression_params.push_back(CV_IMWRITE_PXM_BINARY);
    compression_params.push_back(binary);
    timer.reset();
    for (unsigned int time = 0; time < ntimes; ++time)
      cv::imwrite("/tmp/foo.ppm", rgb, compression_params);
    printf("PPM writing, binary:%i: \t\t\t%g ms, %s", binary, timer.time() / ntimes,
                file_size("/tmp/foo.ppm").c_str());
  } // end loop binary

  for (unsigned int quality = 55; quality <= 100; quality+=15) {
    compression_params.clear();
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(quality);
    timer.reset();
    for (unsigned int time = 0; time < ntimes; ++time)
      cv::imwrite("/tmp/foo.jpg", rgb, compression_params);
    printf("JPG writing, quality %i: \t\t\t%g ms, %s", quality, timer.time() / ntimes,
                file_size("/tmp/foo.jpg").c_str());
  } // end loop quality

  for (unsigned int quality = 55; quality <= 100; quality+=15) {
    compression_params.clear();
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(quality);
    timer.reset();
    for (unsigned int time = 0; time < ntimes; ++time)
      cv::imwrite("/tmp/foo.jp2", rgb, compression_params);
    printf("JPG2000 writing, quality %i: \t\t\t%g ms, %s", quality, timer.time() / ntimes,
                file_size("/tmp/foo.jp2").c_str());
  } // end loop quality

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    cv::imwrite("/tmp/foo.bmp", rgb);
  printf("bmp writing: \t\t\t\t%g ms, %s", timer.time() / ntimes,
              file_size("/tmp/foo.bmp").c_str());

  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    cv::imwrite("/tmp/foo.tiff", rgb);
  printf("tiff writing: \t\t\t\t%g ms, %s", timer.time() / ntimes,
              file_size("/tmp/foo.tiff").c_str());
}

////////////////////////////////////////////////////////////////////////////////

void test_depth_image(const cv::Mat & depth) {
  printf("test_depth_image(%i x %i)\n", depth.cols, depth.rows);
  unsigned int ntimes = 10;
  std::vector<int> compression_params;
  vision_utils::Timer timer;

  compression_params.clear();
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);
  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    cv::imwrite("/tmp/foo.png", depth, compression_params);
  printf("PNG writing, compression 0: \t\t%g ms, %s", timer.time() / ntimes,
              file_size("/tmp/foo.png").c_str());

  cv::Mat depth1000;
  depth.convertTo(depth1000, CV_16U, 1000, 0);
  timer.reset();
  for (unsigned int time = 0; time < ntimes; ++time)
    cv::imwrite("/tmp/foo.png", depth1000, compression_params);
  printf("scaling & PNG writing, compression 0: \t\t%g ms, %s", timer.time() / ntimes,
              file_size("/tmp/foo.png").c_str());
}

////////////////////////////////////////////////////////////////////////////////

int main() {
  //test_color_image(cv::imread(vision_utils::IMG_DIR() + "balloon.png"));
  test_color_image(cv::imread(vision_utils::IMG_DIR() + "arnaud001.png"));

  //cv::Mat depth;
  //generate_full_rand<float>(depth, 640, 480, 1, 10, false);
  //test_depth_image(depth);
  return 0;
}
