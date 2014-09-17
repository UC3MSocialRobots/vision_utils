#include "debug/error.h"
#include "time/timer.h"
#include "border_remover.h"
#include <vision_utils/img_path.h>

#include <opencv2/highgui/highgui.hpp>

int main() {
  maggieDebug2("main()");
  //maggieDebug2("%sborder.png", IMG_DIR);
  cv::Mat1b src = cv::imread(IMG_DIR "border.png", CV_LOAD_IMAGE_GRAYSCALE),
      dst;
  Timer timer;
  image_utils::remove_border(src, dst);
  timer.printTime("remove_border()");
  cv::imshow("src", src);
  cv::imshow("dst", dst);
  cv::waitKey(0);
  return 0;
}


