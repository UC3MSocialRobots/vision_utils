/*!
 * \file OcrUtils.cpp
 *
 * TODO descr
 *
 * \date Dec 18, 2010
 * \author Arnaud Ramey
 */

#include "vision_utils/OcrUtils.h"
#include "vision_utils/io.h"
#include "vision_utils/utils/error.h"
#include "vision_utils/img_path.h"
#include "vision_utils/utils/file_io.h"
#include "vision_utils/utils/find_and_replace.h"
#include "vision_utils/utils/string_split.h"
#include "vision_utils/utils/string_case.h"

//#define USE_API

#ifdef USE_API
// tesseract API : http://tesseract-ocr.repairfaq.org/classTessBaseAPI.html
#include <tesseract/baseapi.h>
#endif

#include <opencv2/highgui/highgui.hpp>

// constants
const std::string OcrUtils::OCR_APP_NAME =
    "tesseract";
//"/usr/local/src/tesseract/api/tesseract";

const OcrUtils::Language OcrUtils::ENGLISH = "eng";
const OcrUtils::Language OcrUtils::SPANISH = "spa";
const OcrUtils::Language OcrUtils::FRENCH = "fra";
const OcrUtils::Language OcrUtils::GREEK = "ell";
const OcrUtils::Language OcrUtils::RUSSIAN = "rus";
const OcrUtils::Language OcrUtils::JAPANESE = "jpn";
const OcrUtils::Language OcrUtils::UNKNOWN = "???";

const std::string OcrUtils::TEMP_BASE_FILENAME = "/tmp/ocr_dump";
const std::string OcrUtils::TEMP_TIF_FILENAME = OcrUtils::TEMP_BASE_FILENAME + ".tif";
const std::string OcrUtils::TEMP_TXT_FILENAME = OcrUtils::TEMP_BASE_FILENAME + ".txt";
const std::string OcrUtils::TEMP_CONFIG_FILENAME = OcrUtils::TEMP_BASE_FILENAME + "_config.txt";

////////////////////////////////////////////////////////////////////////////////

bool OcrUtils::load_word_dict(const std::string & dict_filename, WordDict & dict) {
  maggieDebug2("load_word_dict('%s')", dict_filename.c_str());

  // read the file as a vector of strings
  std::vector<std::string> dict_vector;
  if (!string_utils::retrieve_file_split(dict_filename, dict_vector))
    return false;
  // push all the words in the set
  for (std::vector<std::string>::const_iterator word = dict_vector.begin();
       word != dict_vector.end() ; ++word)
    dict.insert(*word);
  return true;
}

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////

#ifdef USE_API
class TessBaseAPI2 : TessBaseAPI {
public:
  static char* analyze(const unsigned char* imagedata,
                       int bytes_per_pixel,
                       int bytes_per_line,
                       int left, int top, int width, int height, //
                       OcrUtils::Language language) {
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

bool OcrUtils::analyse_image(const cv::Mat & image,
                             const Translator::LanguageId src_language,
                             std::string & answer) {
  maggieDebug2("analyse_image('%s', language:%i)",
               image_utils::infosImage(image).c_str(),
               src_language);

  Language ocr_src_language = OcrUtils::UNKNOWN;
  if (src_language == Translator::LANGUAGE_ENGLISH)
    ocr_src_language = OcrUtils::ENGLISH;
  else if (src_language == Translator::LANGUAGE_SPANISH)
    ocr_src_language = OcrUtils::SPANISH;
  else if (src_language == Translator::LANGUAGE_FRENCH)
    ocr_src_language = OcrUtils::FRENCH;
  else if (src_language == Translator::LANGUAGE_GREEK)
    ocr_src_language = OcrUtils::GREEK;
  else if (src_language == Translator::LANGUAGE_JAPANESE)
    ocr_src_language = OcrUtils::JAPANESE;
  else if (src_language == Translator::LANGUAGE_RUSSIAN)
    ocr_src_language = OcrUtils::RUSSIAN;

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
  maggieDebug2("Executing '%s'", instruction.str().c_str());
  int retval = system( instruction.str().c_str() );
  if (retval < 0) {
    maggiePrint("Executing '%s' returned an error!", instruction.str().c_str());
    return false;
  }
  // get the file
  //std::vector<std::string> ocr_result_lines;
  //string_utils::retrieve_file_split(TEMP_TXT_FILENAME, ocr_result_lines);
  std::string ocr_result_dirty;
  if (!string_utils::retrieve_file(TEMP_TXT_FILENAME, ocr_result_dirty))
    return false;
#endif

  maggieDebug2("Result before cleaning :'%s'", ocr_result_dirty.c_str());
  answer = ocr_result_dirty;
  if (!clean_string_from_weird_chars(answer, src_language))
    return false;

  // put together words cut in two
  string_utils::find_and_replace(answer, "-\n", "");
  string_utils::find_and_replace(answer, "- \n", "");
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool OcrUtils::load_letter_dict(const Translator::LanguageId &src_language,
                                LetterDict & dict) {
  maggieDebug2("Loading the dictionary for the language %i", src_language);
  std::string dict_filename = "";
  if (src_language == Translator::LANGUAGE_ENGLISH)
    dict_filename = IMG_DIR "hangman/english.characters";
  else if (src_language == Translator::LANGUAGE_FRENCH)
    dict_filename = IMG_DIR "hangman/spanish.characters";
  else if (src_language == Translator::LANGUAGE_SPANISH)
    dict_filename = IMG_DIR "hangman/spanish.characters";
  else {
    printf("OcrUtils::t here is no letter list for language %i!\n", src_language);
    return false;
  }

  maggieDebug2("load_letter_dict('%s')", dict_filename.c_str());

  // read the file as a string
  std::string file_str;
  if (!string_utils::retrieve_file(dict_filename, file_str))
    return false;
  // push all the letters in the set
  dict.clear();
  for (unsigned int letter_idx = 0 ; letter_idx < file_str.length() ;
       ++letter_idx)
    dict.insert( file_str.at(letter_idx) );
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool OcrUtils::clean_string_from_weird_chars(std::string & sentence_to_clean,
                                             const Translator::LanguageId src_language) {
  maggieDebug3("clean_string_from_weird_chars(%s)",
               sentence_to_clean.substr(0, std::min(10, (int) sentence_to_clean.size())).c_str());

  // convert to lowercase
  string_utils::to_lowercase(sentence_to_clean);
  maggieDebug3("After to_lowercase:'%s'",
               sentence_to_clean.c_str());

  /*
    *  check if the word contains weird letters
    */
  if (load_letter_dict(src_language, _letters)) {
    std::ostringstream sentence_to_clean_no_weird_words;

    // split to lines
    std::vector<std::string> ocr_lines;
    string_utils::StringSplit(sentence_to_clean, "\n", &ocr_lines);

    // check each line
    for (std::vector<std::string>::const_iterator ocr_line = ocr_lines.begin();
         ocr_line != ocr_lines.end() ; ++ocr_line) {
      std::ostringstream ocr_line_clean;
      // split to words
      std::vector<std::string> ocr_words;
      string_utils::StringSplit(*ocr_line, " ", &ocr_words);

      // check each word
      for (std::vector<std::string>::const_iterator ocr_word = ocr_words.begin();
           ocr_word != ocr_words.end() ; ++ocr_word) {
        // check if there is a weird char
        bool contains_weird_char = false;
        for (unsigned int letter_idx = 0 ; letter_idx < ocr_word->length() ;
             ++letter_idx) {
          char current_char = ocr_word->at(letter_idx);
          if (_letters.find(current_char) == _letters.end()) {
            contains_weird_char = true;
            break;
          }
        } // end for letter_idx
        if (contains_weird_char == false) {
          ocr_line_clean << *ocr_word << ' ';
          continue;
        } // end if contains_weird_char

        // if we reached here, remove it
        maggieDebug2("Removing word '%s'", ocr_word->c_str());
      } // end for ocr_word

      sentence_to_clean_no_weird_words << ocr_line_clean.str() << std::endl;
    } // end for ocr_line

    sentence_to_clean = sentence_to_clean_no_weird_words.str();
    maggieDebug3("After cleaning weird words:'%s'",
                 sentence_to_clean.c_str());
  } // end if (load_letter_dict())

  /*
     * check if each line contains at least a character
     */
  if (src_language == Translator::LANGUAGE_ENGLISH ||
      src_language == Translator::LANGUAGE_SPANISH ||
      src_language == Translator::LANGUAGE_FRENCH) {
    // split to lines
    std::ostringstream sentence_to_clean_no_empty_lines;
    std::vector<std::string> ocr_lines;
    string_utils::StringSplit(sentence_to_clean, "\n", &ocr_lines);

    // check each line
    for (std::vector<std::string>::const_iterator ocr_line = ocr_lines.begin();
         ocr_line != ocr_lines.end() ; ++ocr_line) {
      bool current_line_contains_any_letters =
          string_utils::contains_any_letter(*ocr_line);
      if (current_line_contains_any_letters)
        sentence_to_clean_no_empty_lines << *ocr_line << "\n";
    }
    sentence_to_clean = sentence_to_clean_no_empty_lines.str();
    maggieDebug3("After removing empty lines:'%s'",
                 sentence_to_clean.c_str());
  } // end if language = ENGLISH...
  return true;
}
