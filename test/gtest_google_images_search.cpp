/*!
  \file        gtest_google_images_search.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/7

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
*/
// Bring in gtest
#include <gtest/gtest.h>
#include "vision_utils/string_split.h"
#include "vision_utils/google_images_search.h"
#include <vision_utils/img_path.h>

bool has_connectivity = false;

TEST(TestSuite, empty_image) {
  if (!has_connectivity)
    return;
  cv::Mat query;
  std::string best_guess;
  std::vector<cv::Mat> similar_images;
  vision_utils::GoogleImagesLookStatus status =
      vision_utils::google_images_lookup(query, best_guess, similar_images, true);
  ASSERT_TRUE(status == vision_utils::SIMILAR_IMAGES_NO_BEST_GUESS_NO) << "status:" << status;
}

////////////////////////////////////////////////////////////////////////////////

void test(const std::string & filename, const std::string & exp_guesses) {
  if (!has_connectivity)
    return;
  cv::Mat query = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  std::string best_guess;
  std::vector<cv::Mat> similar_images;
  vision_utils::GoogleImagesLookStatus status =
      vision_utils::google_images_lookup(query, best_guess, similar_images, true);
  ASSERT_TRUE(status == vision_utils::SIMILAR_IMAGES_OK_BEST_GUESS_OK) << "status:" << status;

  bool guess_match = false;
  std::vector<std::string> exp_guesses_vec;
  vision_utils::StringSplit(exp_guesses, ";", &exp_guesses_vec);
  for (unsigned int word_idx = 0; word_idx < exp_guesses_vec.size(); ++word_idx) {
    if (best_guess.find(exp_guesses[word_idx]) != std::string::npos) {
      guess_match = true;
      break;
    }
  } // end for word_idx

  ASSERT_TRUE(guess_match)
      << "exp_guesses:'" << exp_guesses << "', best_guess:'" << best_guess << "'";
  // check all images make sense
  unsigned int nimages = similar_images.size();
  ASSERT_TRUE(nimages > 0);
  for (unsigned int image_idx = 0; image_idx < nimages; ++image_idx)
    ASSERT_TRUE(similar_images[image_idx].cols > 0 && similar_images[image_idx].rows > 0);
}

TEST(TestSuite, frenadol) { test(vision_utils::IMG_DIR() + "frenadol.png", "frenadol"); }
TEST(TestSuite, maggie) { test(vision_utils::IMG_DIR() + "maggie.png", "social robots;human robot interaction"); }

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
