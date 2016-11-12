/*!
  \file        gtest_image_comparer.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/26

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

Some tests for ImageComparer

 */
// Bring in gtest
#include <gtest/gtest.h>
#include "vision_utils/image_comparer.h"
#include <vision_utils/img_path.h>
#include "vision_utils/timer.h"

#include <opencv2/highgui/highgui.hpp>
#include <queue>
#include <list>

typedef std::vector<cv::Point> Pt2Vec;
typedef std::list<cv::Point>   Pt2List;
typedef std::deque<cv::Point>  Pt2Deque;
typedef std::queue<cv::Point>  Pt2Queue;

TEST(TestSuite, constructor) {
  vision_utils::ImageComparer_<Pt2Vec> comparer;
  ASSERT_TRUE(comparer.get_models_nb() == 0);

  bool ok = comparer.compareFile(cv::Mat1b());
  ASSERT_TRUE(ok == false);

  ok = comparer.compareFile(cv::Mat1b(10, 10, (uchar) 255));
  ASSERT_TRUE(ok == false);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, constructor_empty_files) {
  vision_utils::ImageComparer_<Pt2Vec> comparer;
  comparer.set_models("", cv::Size(32, 32));
  ASSERT_TRUE(comparer.get_models_nb() == 0);

  bool ok = comparer.compareFile(cv::Mat1b());
  ASSERT_TRUE(ok == false);

  ok = comparer.compareFile(cv::Mat1b(10, 10, (uchar) 255));
  ASSERT_TRUE(ok == false);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, constructor_empty_model) {
  vision_utils::ImageComparer_<Pt2Vec> comparer;
  std::vector<cv::Mat> models;
  comparer.set_models(models, cv::Size(32, 32));
  ASSERT_TRUE(comparer.get_models_nb() == 0);

  bool ok = comparer.compareFile(cv::Mat1b());
  ASSERT_TRUE(ok == false);

  ok = comparer.compareFile(cv::Mat1b(10, 10, (uchar) 255));
  ASSERT_TRUE(ok == false);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, constructor_one_file) {
  vision_utils::ImageComparer_<Pt2Vec> comparer;
  std::vector<std::string> models;
  models.push_back(vision_utils::IMG_DIR() + "comparer/1.png");
  comparer.set_models(models, cv::Size(10, 10));
  ASSERT_TRUE(comparer.get_models_nb() == models.size());
  printf("model[0]:'%s'\n", comparer.model_to_string(0).c_str());

  bool ok = comparer.compareFile(cv::Mat1b());
  ASSERT_TRUE(ok == false);

  ok = comparer.compareFile(cv::Mat1b(50, 50, (uchar) 255));
  // printf("unknown_object:'%s'\n", comparer.unknown_to_string().c_str());
  ASSERT_TRUE(ok == true);
  ASSERT_TRUE(comparer.get_best_index() == 0);

  ok = comparer.compareFile(cv::imread(models.front(), CV_LOAD_IMAGE_GRAYSCALE));
  // printf("unknown_object:'%s'\n", comparer.unknown_to_string().c_str());
  ASSERT_TRUE(ok == true);
  ASSERT_TRUE(comparer.get_best_index() == 0);
  ASSERT_TRUE(comparer.getBestResult() == 0)
      << "unknown:" << comparer.unknown_to_string();
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt2Iterable>
void test_four_files() {
  vision_utils::ImageComparer_<Pt2Iterable> comparer;
  std::vector<std::string> models;
  models.push_back(vision_utils::IMG_DIR() + "comparer/1.png");
  models.push_back(vision_utils::IMG_DIR() + "comparer/2.png");
  models.push_back(vision_utils::IMG_DIR() + "comparer/3.png");
  models.push_back(vision_utils::IMG_DIR() + "comparer/4.png");
  int nmodels = models.size();
  comparer.set_models(models, cv::Size(64, 64));
  ASSERT_TRUE(comparer.get_models_nb() == models.size());
  //  for (int model_idx = 0; model_idx < nmodels; ++model_idx)
  //    printf("model[%i]:'%s'\n", model_idx, comparer.model_to_string(model_idx).c_str());

  for (int model_idx = 0; model_idx < nmodels; ++model_idx) {
    bool ok = comparer.compareFile(cv::imread(models.at(model_idx), CV_LOAD_IMAGE_GRAYSCALE));
    ASSERT_TRUE(ok == true);
    ASSERT_TRUE(comparer.get_best_index() == model_idx);
    ASSERT_TRUE(comparer.getBestResult() == 0)
        << "unknown:" << comparer.unknown_to_string();
  }
}

TEST(TestSuite, constructor_four_files) {
  vision_utils::Timer timer;
  test_four_files<Pt2Vec>();
  timer.printTime("test_four_files<Pt2Vec>()");

  //  timer.reset();
  //  test_four_files<Pt2List>();
  //  timer.printTime("test_four_files<Pt2List>()");

  timer.reset();
  test_four_files<Pt2Deque>();
  timer.printTime("test_four_files<Pt2Deque>()");

  //  timer.reset();
  //  test_four_files<Pt2Queue>();
  //  timer.printTime("test_four_files<Pt2Queue>()");
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_comparer) {
  vision_utils::ImageComparer im;
  im.set_models(vision_utils::IMG_DIR() + "paintRecognizer/index.txt", cv::Size(32, 32));

  cv::Mat img = cv::imread(vision_utils::IMG_DIR() + "paintRecognizer/inputs/m.png",
                           CV_LOAD_IMAGE_GRAYSCALE);
  int nbTimes = 5;
  vision_utils::Timer timer;
  for (int i = 0; i < nbTimes; ++i)
    im.compareFile(img);
  timer.printTime_factor("compareFile", nbTimes);
  printf("results:'%s'\n", im.results_to_string().c_str());
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test_comparer2) {
  printf("test_comparer()\n");
  vision_utils::ImageComparer im;
  im.set_models(vision_utils::IMG_DIR() + "comparer/index.txt", cv::Size(32, 32));

  std::vector<std::string> filenames;
  filenames.push_back(vision_utils::IMG_DIR() + "comparer/model.png");
  filenames.push_back(vision_utils::IMG_DIR() + "comparer/1.png");
  filenames.push_back(vision_utils::IMG_DIR() + "comparer/2.png");
  filenames.push_back(vision_utils::IMG_DIR() + "comparer/3.png");
  filenames.push_back(vision_utils::IMG_DIR() + "comparer/4.png");
  for (int i = 0 ; i < 5 ; ++i) {
    im.compareFile(cv::imread(filenames.at(i), CV_LOAD_IMAGE_GRAYSCALE));
    printf("Comparing with '%s', results:'%s'\n",
           filenames.at(i).c_str(), im.results_to_string().c_str());
  }
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
