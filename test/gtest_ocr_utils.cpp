/*!
 * \file gtest_ocr_utils.cpp
 *
 * Some tests for the OCR
 *
 * \date Dec 18, 2010
 * \author Arnaud Ramey
 */

// Bring in gtest
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
// AD
#include "vision_utils/OcrUtils.h"
#include "vision_utils/utils/debug_utils.h"
#include <vision_utils/img_path.h>

void ocr_test(const std::string filename, Translator::LanguageId language,
                     const std::string & searched_pattern = "") {
  printf("\n\n*** ocr_test('%s')\n", filename.c_str());
  cv::Mat m = cv::imread(filename);
  ASSERT_FALSE(m.empty());

  std::string ocr_result;
  OcrUtils ocr;
  ASSERT_TRUE(ocr.analyse_image(m, language, ocr_result));
  printf("ocr_result:'%s'\n", ocr_result.c_str());
  ASSERT_TRUE(ocr_result.find(searched_pattern) != std::string::npos)
      << "searched_pattern:'" << searched_pattern <<"'";
//  std::string trad = Translator::translate(ocr_result, language, Translator::LANGUAGE_ENGLISH);
//  printf("trad:'%s'\n", trad.c_str());
}

TEST(TestSuite, ocr1) {
  ocr_test(IMG_DIR "ocr/ocr.png", Translator::LANGUAGE_ENGLISH, "1234");
}
TEST(TestSuite, ocr2) {
  ocr_test(IMG_DIR "ocr/ocr2.png", Translator::LANGUAGE_SPANISH, "escuelas infantiles");
}
TEST(TestSuite, ocr3) {
  ocr_test(IMG_DIR "ocr/ocr3.png", Translator::LANGUAGE_SPANISH, "rendimiento");
}
TEST(TestSuite, ocr4) {
  ocr_test(IMG_DIR "ocr/ocr4.png", Translator::LANGUAGE_SPANISH, "vuelta");
}
TEST(TestSuite, ocr5) {
  ocr_test(IMG_DIR "ocr/ocr5.png", Translator::LANGUAGE_SPANISH, "cita en esta muestra");
}
TEST(TestSuite, ocr6) {
  ocr_test(IMG_DIR "ocr/ocr6.png", Translator::LANGUAGE_SPANISH, "nuestras");
}
TEST(TestSuite, ocr7) {
  ocr_test(IMG_DIR "ocr/ocr7.png", Translator::LANGUAGE_SPANISH, "urgente");
}
TEST(TestSuite, ocr8) {
  ocr_test(IMG_DIR "ocr/ocr8.png", Translator::LANGUAGE_ENGLISH, "caps lock");
}
TEST(TestSuite, ru) {
  ocr_test(IMG_DIR "ocr/ocr-ru-one_language_is_never_enough.png", Translator::LANGUAGE_RUSSIAN);
}
TEST(TestSuite, ja) {
  ocr_test(IMG_DIR "ocr/ocr-ja.png", Translator::LANGUAGE_JAPANESE);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
