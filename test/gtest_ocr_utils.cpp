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
#include <vision_utils/analyse_image.h>
#include <vision_utils/build_languages_map.h>
#include <vision_utils/img_path.h>

void ocr_test(const std::string filename, vision_utils::LanguageId language,
                     const std::string & searched_pattern = "") {
  printf("\n\n*** ocr_test('%s')\n", filename.c_str());
  cv::Mat m = cv::imread(filename);
  ASSERT_FALSE(m.empty());
  std::string ocr_result;
  ASSERT_TRUE(vision_utils::analyse_image(m, language, ocr_result));
  printf("ocr_result:'%s'\n", ocr_result.c_str());
  ASSERT_TRUE(ocr_result.find(searched_pattern) != std::string::npos)
      << "searched_pattern:'" << searched_pattern <<"'";
//  std::string trad = vision_utils::translate(ocr_result, language, vision_utils::LANGUAGE_ENGLISH);
//  printf("trad:'%s'\n", trad.c_str());
}

TEST(TestSuite, ocr1) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr.png", vision_utils::LANGUAGE_ENGLISH, "1234");
}
TEST(TestSuite, ocr2) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr2.png", vision_utils::LANGUAGE_SPANISH, "escuelas infantiles");
}
TEST(TestSuite, ocr3) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr3.png", vision_utils::LANGUAGE_SPANISH, "rendimiento");
}
TEST(TestSuite, ocr4) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr4.png", vision_utils::LANGUAGE_SPANISH, "vuelta");
}
TEST(TestSuite, ocr5) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr5.png", vision_utils::LANGUAGE_SPANISH, "cita en esta muestra");
}
TEST(TestSuite, ocr6) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr6.png", vision_utils::LANGUAGE_SPANISH, "nuestras");
}
TEST(TestSuite, ocr7) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr7.png", vision_utils::LANGUAGE_SPANISH, "urgente");
}
TEST(TestSuite, ocr8) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr8.png", vision_utils::LANGUAGE_ENGLISH, "caps lock");
}
TEST(TestSuite, ru) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr-ru-one_language_is_never_enough.png", vision_utils::LANGUAGE_RUSSIAN);
}
TEST(TestSuite, ja) {
  ocr_test(vision_utils::IMG_DIR() + "ocr/ocr-ja.png", vision_utils::LANGUAGE_JAPANESE);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
