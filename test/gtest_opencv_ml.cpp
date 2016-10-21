/*!
  \file        gtest_opencv_ml.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/1/24

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

Some tests with OpenCV machine learning algos.

 */
#include <gtest/gtest.h>
#include <opencv2/ml/ml.hpp>
#if 0 // TODO
TEST(TestSuite, SVM_regression) {
  unsigned int nsamples = 50;
  cv::Mat1f training(nsamples, 2), truth_labels(nsamples, 1); // rows, cols
  for (unsigned int sample_idx = 0; sample_idx < nsamples; ++sample_idx) {
    double x = 10 * drand48(), y = 10 * drand48();
    training(sample_idx, 0) = x;
    training(sample_idx, 1) = y;
    truth_labels(sample_idx) = x + y;
  } // end loop sample_idx

  CvSVMParams params;
  //param.svm_type    = CvSVM::EPS_SVR;
  params.p = 0.01;
  params.svm_type    = CvSVM::NU_SVR;
  params.nu = 0.5;
  params.kernel_type = CvSVM::LINEAR;
  //params.kernel_type = CvSVM::RBF;
  params.gamma = 1; // the smaller, the bigger the blobs
  params.C = 1; // the bigger, the bigger the penalty for misclassification
  params.term_crit.type = CV_TERMCRIT_ITER +CV_TERMCRIT_EPS;
  params.term_crit.max_iter = 1000;
  params.term_crit.epsilon = 1e-6;
  cv::SVM svm;
  ASSERT_TRUE(svm.train_auto(training, truth_labels, cv::Mat(), cv::Mat(), params, 3));

  cv::Mat1f predict_labels;
  svm.predict(training, predict_labels);
  std::cout << "truth_labels:" << truth_labels << std::endl;
  std::cout << "predict_labels:" << predict_labels << std::endl;
  double mean_error = cv::mean(cv::abs(predict_labels - truth_labels))[0];
  std::cout << "mean_error:"  << mean_error << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

void test_RandomTrees_regression(unsigned int nsamples,
                                 double max_val) {
  printf("\ntest_RandomTrees_regression(%i samples in [0,%g])\n",
         nsamples, max_val);
  unsigned int nfeatures = 2;
  cv::Mat1f training(nsamples, nfeatures), truth_labels(nsamples, 1); // rows, cols
  for (unsigned int sample_idx = 0; sample_idx < nsamples; ++sample_idx) {
    double x = max_val * drand48(), y = max_val * drand48();
    training(sample_idx, 0) = x; // OK float
    training(sample_idx, 1) = y; // OK float
    truth_labels(sample_idx) = x + y; // OK float with CV_VAR_NUMERICAL
  } // end loop sample_idx

  cv::RandomTrees rt;
  // declare input as numerical
  cv::Mat var_type = cv::Mat(nfeatures+1 , 1, CV_8U, cv::Scalar::all(CV_VAR_NUMERICAL));
  ASSERT_TRUE(rt.train(training, CV_ROW_SAMPLE, truth_labels,
                       cv::Mat(), cv::Mat(), var_type));

  cv::Mat1f predict_labels(nsamples, 1);
  for (unsigned int row_idx = 0; row_idx < nsamples; ++row_idx)
    predict_labels(row_idx) = rt.predict(training.row(row_idx));
  std::cout << "truth_labels:" << truth_labels << std::endl;
  std::cout << "predict_labels:" << predict_labels << std::endl;
  double mean_error = cv::mean(cv::abs(predict_labels - truth_labels))[0];
  std::cout << "mean_error:"  << mean_error << std::endl;
}

TEST(TestSuite, RandomTrees_regression) {
  test_RandomTrees_regression(50, 10);
  test_RandomTrees_regression(50, 1);
  test_RandomTrees_regression(20, 10);
  test_RandomTrees_regression(10, 10);
  test_RandomTrees_regression(5, 10);
  test_RandomTrees_regression(5, 1);
}
#endif

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
