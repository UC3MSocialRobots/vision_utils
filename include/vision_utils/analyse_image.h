/*!
  \file        analyse_image.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/2
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

#ifndef ANALYSE_IMAGE_H
#define ANALYSE_IMAGE_H
// std includes
#include <opencv2/core/core.hpp>
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>
#include <vector>
// vision_utils
#include <vision_utils/build_languages_map.h>
#include <vision_utils/clean_string_from_weird_chars.h>
#include <vision_utils/find_and_replace.h>
#include <vision_utils/infosimage.h>
#include <vision_utils/retrieve_file.h>
#include <vision_utils/retrieve_file_split.h>

namespace vision_utils {

// constants
#define OCR_APP_NAME  "tesseract"
//#define OCR_APP_NAME  "/usr/local/src/tesseract/api/tesseract"

#define ENGLISH   "eng"
#define SPANISH   "spa"
#define FRENCH    "fra"
#define GREEK     "ell"
#define RUSSIAN   "rus"
#define JAPANESE  "jpn"
#define UNKNOWN   "???"

#define TEMP_BASE_FILENAME    "/tmp/ocr_dump"
#define TEMP_TIF_FILENAME     TEMP_BASE_FILENAME  ".tif"
#define TEMP_TXT_FILENAME     TEMP_BASE_FILENAME  ".txt"
#define TEMP_CONFIG_FILENAME  TEMP_BASE_FILENAME  "_config.txt"

//#define USE_API

#ifdef USE_API
// tesseract API : http://tesseract-ocr.repairfaq.org/classTessBaseAPI.html
#include <tesseract/baseapi.h>
#endif

typedef std::string OcrLanguage;

bool analyse_image(const cv::Mat & image,
                   const vision_utils::LanguageId src_language,
                   std::string & answer) {
  printf("analyse_image('%s', language:%i)\n",
           vision_utils::infosImage(image).c_str(),
           src_language);

  OcrLanguage ocr_src_language = UNKNOWN;
  if (src_language == vision_utils::LANGUAGE_ENGLISH)
    ocr_src_language = ENGLISH;
  else if (src_language == vision_utils::LANGUAGE_SPANISH)
    ocr_src_language = SPANISH;
  else if (src_language == vision_utils::LANGUAGE_FRENCH)
    ocr_src_language = FRENCH;
  else if (src_language == vision_utils::LANGUAGE_GREEK)
    ocr_src_language = GREEK;
  else if (src_language == vision_utils::LANGUAGE_JAPANESE)
    ocr_src_language = JAPANESE;
  else if (src_language == vision_utils::LANGUAGE_RUSSIAN)
    ocr_src_language = RUSSIAN;

#ifdef USE_API
  /*
         convert to continuous if needed
         */
  cv::Mat image_cont;
  //    if (image_cont.isContinuous())
  //        image_cont == image;
  //    else
  image.copyTo(image_cont);

  //    cv::cvtColor(image, image_cont, CV_RGB2GRAY);
  //    cv::adaptiveThreshold(image_cont, image_cont, 255,
  //                          CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 51, 0);

  //    cv::cvtColor(image, image_cont, CV_RGB2GRAY);
  //    cv::adaptiveThreshold(image_cont, image_cont, 255,
  //                          CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 91, 0);

  /*
     tesseract convert
     */
  char* text =
      TessBaseAPI2::analyze(image_cont.datastart,
                            image_cont.elemSize(),
                            image_cont.cols * image_cont.elemSize(),
                            0, 0,
                            image_cont.cols, image_cont.rows,
                            ocr_src_language);

  //std::vector<std::string> ocr_result_lines;
  //ocr_result_lines.push_back( std::string(text) );
  std::string ocr_result_dirty(text);
#else
  cv::imwrite(TEMP_TIF_FILENAME, image);
  std::ostringstream instruction;
  instruction << OCR_APP_NAME << " "
              << TEMP_TIF_FILENAME << ' '
              << TEMP_BASE_FILENAME;
  if (ocr_src_language != UNKNOWN)
    instruction << " -l " << ocr_src_language;
  printf("Executing '%s'\n", instruction.str().c_str());
  int retval = system( instruction.str().c_str() );
  if (retval < 0) {
    printf("Executing '%s' returned an error!\n", instruction.str().c_str());
    return false;
  }
  // get the file
  //std::vector<std::string> ocr_result_lines;
  //vision_utils::retrieve_file_split(TEMP_TXT_FILENAME, ocr_result_lines);
  std::string ocr_result_dirty;
  if (!vision_utils::retrieve_file(TEMP_TXT_FILENAME, ocr_result_dirty))
    return false;
#endif

  printf("Result before cleaning :'%s'\n", ocr_result_dirty.c_str());
  answer = ocr_result_dirty;
  if (!clean_string_from_weird_chars(answer, src_language))
    return false;

  // put together words cut in two
  vision_utils::find_and_replace(answer, "-\n", "");
  vision_utils::find_and_replace(answer, "- \n", "");
  return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#ifdef USE_API
class TessBaseAPI2 : TessBaseAPI {
public:
  char* analyze(const unsigned char* imagedata,
                       int bytes_per_pixel,
                       int bytes_per_line,
                       int left, int top, int width, int height, //
                       Language language) {
    InitWithLanguage(NULL, NULL, language.c_str(), NULL,
                     false, 0, NULL);

    //        SetVariable("tessedit_char_whitelist", "abcdefghijklmnopqrstuvwxyz"
    //                    "0123456789"
    //                    "áéíñóúüÁÉÍÑÓÚ"
    //                    //",.;:-¿¡?!"
    //                    );

    //        CopyImageToTesseract(imagedata, bytes_per_pixel, bytes_per_line,
    //                             left, top, width, height);
    DumpPGM("out.pgm");

    return TesseractRect(imagedata, bytes_per_pixel, bytes_per_line,
                         left, top, width, height);

    TessBaseAPI::End();
    // cf http://www.mail-archive.com/tesseract-ocr@googlegroups.com/msg01875.html
  }
};
#endif

} // end namespace vision_utils

#endif // ANALYSE_IMAGE_H
