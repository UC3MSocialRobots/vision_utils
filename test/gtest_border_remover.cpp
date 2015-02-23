#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>

#include "time/timer.h"
#include "image_utils/border_remover.h"
#include <vision_utils/img_path.h>

TEST(TestSuite, simple_test) {
  //maggieDebug2("%sborder.png", IMG_DIR);
  cv::Mat1b src = cv::imread(IMG_DIR "border.png", CV_LOAD_IMAGE_GRAYSCALE),
      dst;
  Timer timer;
  image_utils::remove_border(src, dst);
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
