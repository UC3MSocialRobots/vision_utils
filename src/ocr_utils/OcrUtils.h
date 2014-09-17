/*!
 * \file OcrUtils.h
 *
 * A package for useful functions in geometry
 *
 * \date Dec 18, 2010
 * \author Arnaud Ramey
 */

#ifndef OcrUtils_H_
#define OcrUtils_H_

// languages ID
#include "src/translator/Translator.h"
// openCV
#include "opencv2/core/core.hpp"
// std
#include <string>
#include <set>

class OcrUtils {
public:
    const static std::string OCR_APP_NAME;

    typedef std::set<std::string> WordDict;
    typedef std::set<char> LetterDict;

    typedef std::string Language;
    static const Language ENGLISH;
    static const Language SPANISH;
    static const Language FRENCH;
    static const Language GREEK;
    static const Language JAPANESE;
    static const Language RUSSIAN;
    static const Language UNKNOWN;

    static const std::string TEMP_BASE_FILENAME;
    static const std::string TEMP_TIF_FILENAME;
    static const std::string TEMP_TXT_FILENAME;
    static const std::string TEMP_CONFIG_FILENAME;

    static bool load_word_dict(const std::string & dict_filename, WordDict & dict);
    static bool load_letter_dict(const Translator::LanguageId &src_language, LetterDict & dict);

    bool analyse_image(const cv::Mat & image,
                       const Translator::LanguageId src_language,
                       std::string & answer);

private:
    LetterDict _letters;

    bool clean_string_from_weird_chars(std::string & sentence_to_clean,
                                       const Translator::LanguageId src_language);
};

#endif /* OcrUtils_H_ */

