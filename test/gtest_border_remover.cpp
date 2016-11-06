#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>

#include "vision_utils/timer.h"
#include "vision_utils/border_remover.h"
#include <vision_utils/img_path.h>

TEST(TestSuite, simple_test) {
  //ROS_INFO("%sborder.png", IMG_DIR);
  cv::Mat1b src = cv::imread(vision_utils::IMG_DIR() + "border.png", CV_LOAD_IMAGE_GRAYSCALE),
      dst;
  vision_utils::Timer timer;
  vision_utils::remove_border(src, dst);
  timer.printTime("remove_border()");
  //cv::imshow("src", src);
  //cv::imshow("dst", dst);
  //cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
