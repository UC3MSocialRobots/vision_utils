/*!
  \file        test_google_images_lookup.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/5/17

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

#include "google_images_search.h"

#include <vision_utils/img_path.h>

void test_img(const cv::Mat & in) {
  cv::namedWindow("img_to_recognize");
  cv::imshow("img_to_recognize", in);
  cv::waitKey(50);

  // with the image ROI, call the Google Images web service
  std::string best_guess;
  std::vector<cv::Mat> similar_images;
  image_utils::GoogleImagesLookStatus google_status =
      image_utils::google_images_lookup(in, best_guess, similar_images);

  if (google_status == image_utils::LOOKUP_FAIL) {
    maggiePrint("google_images_lookup() failed.");
    return;
  }

  if (google_status == image_utils::SIMILAR_IMAGES_NO_BEST_GUESS_OK
      || google_status == image_utils::SIMILAR_IMAGES_OK_BEST_GUESS_OK)
    maggiePrint("best_guess:'%s'", best_guess.c_str());

  // show each image
  if (google_status == image_utils::SIMILAR_IMAGES_OK_BEST_GUESS_OK
      || google_status == image_utils::SIMILAR_IMAGES_OK_BEST_GUESS_NO) {
    for (unsigned int similar_idx = 0; similar_idx < similar_images.size();
         ++similar_idx) {
      std::string window_name =
          std::string("similar #") + StringUtils::cast_to_string(similar_idx);
      cv::imshow(window_name, similar_images[similar_idx]);
    } // end loop similar_idx
    cv::waitKey(0);
    cv::destroyAllWindows();
  }
} // end test_img();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  // ros::init(argc, argv, "test_google_images_lookup");
  maggiePrint("test_google_images_lookup");
  test_img(cv::imread(IMG_DIR "frenadol.png"));
  //test_img(cv::imread(IMG_DIR "balloon.png"));
  //test_img(cv::imread(IMG_DIR "maggie.png"));
  //test_img(cv::imread(IMG_DIR "group.jpg"));
  return 0;
} // end main()




