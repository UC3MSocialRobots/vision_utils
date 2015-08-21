/*!
 * \file StringUtils.h
 *
 * Some utilities for strings
 *
 * \date Dec 5, 2010
 * \author Arnaud Ramey
 */
#ifndef STRINGUTILS_H_
#define STRINGUTILS_H_

#include "vision_utils/utils/error.h"
#include "vision_utils/utils/cut_string_into_chunks.h"
#include "vision_utils/utils/file_io.h"
#include "vision_utils/utils/filename_handling.h"
#include "vision_utils/utils/find_and_replace.h"
#include "vision_utils/utils/string_casts.h"
#include "vision_utils/utils/string_casts_stl.h"
#include "vision_utils/utils/string_cleaning.h"
#include "vision_utils/utils/string_split.h"
#include "vision_utils/utils/timestamp.h"
// C
#include <stdio.h>
// C++
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>

//ROS 
#include "ros/ros.h"
#include <ros/package.h>

namespace string_utils {
inline std::string extract(const std::string & in,
                           const unsigned int max_length = 50) {
  if (in.size() < max_length)
    return in;
  return in.substr(0, max_length) + std::string("…");
}
} // end namespace string_utils

namespace StringUtils {

/*! get the content of a distant url
 * \param url for instance http://www.google.es
 * \param ans where we save the answer
 * \return true if success, false otherwise
 */
bool retrieve_url(const std::string & url, std::string & ans);
//! This is the writer call back function used by curl
int curl_writer(char *data, size_t size, size_t nmemb, std::string *buffer);

/*! Convert the text to url encode, in orde to use, for example,  on a url request
 *  from http://www.zedwood.com/article/111/cpp-urlencode-function
 * \param texto to convert
 * \example España -> Espa%C3%B1a
 * \return urlEncoded text
 */
std::string toUrlEncode (const std::string & text);

std::string concatenate_vector(const std::vector<std::string> & vec);

/*! change the encoding of a string
 * \param content the string to change
 * \param EUCSET the initial encoding of the string
 * \param OUTSET the encoding we want in output
 */
void convert_string_encoding(std::string & content,
                             const char * EUCSET, const char * OUTSET);

/*! change the encoding of a string UTF8 -> ISO-8859-1
 * \param content the string to change
 */
void convert_string_encoding_utf_to_iso(std::string & content);

/*! change the encoding of a string ISO-8859-1 -> UTF-8
 * \param content the string to change
 */
void convert_string_encoding_iso_to_utf(std::string & content);

/*! convert the HTML entities of a string -> UTF-8
 * \param content the string to change
 */
void convert_string_encoding_htlm_to_utf(std::string & content);


/*! \brief   returns the absolute path from a relative path.
 * For instance, absolutePath("../3", "C:/1/2/") = "C:/1/3"
 *
 * \param   filename the relative filename, after prefix
 * \param   prefix the beginning of the path
 * \return  the absolute path
 */
std::string absolutePath(const char* filename,
                         const std::string prefix);
//inline std::string absolutePath(const char* filename);


/*! extract an info from some tags
 * \param content
 * \param block_begin
 * \param block_end
 * \param initial_search_pos
 * \return "" if not found
 */
std::string extract_from_tags(const std::string & content,
                              const std::string & block_begin,
                              const std::string & block_end,
                              int & initial_search_pos);

/*! Generate a random string made of characters in the range a -> z
 * \param length the size of the wanted string
 */
std::string random_string(const int length);

//! convert a string to lowercase
void to_lowercase(std::string & sentence);

//! convert a string to uppercase
void to_uppercase(std::string & sentence);

//! return true if it contains a letter in A-Z || a-z
bool contains_any_letter(const std::string & sentence);

//! return a short URL of long URL (using google shortener)
std::string getShortURL(const std::string & longURL);

////////////////////////////////////////////////////////////////////////////////

//inline std::string image_utils::absolutePath(const char* filename) {
//  std::string current_file_path = (std::string) __FILE__;
//  ssize_t last_slash_pos = current_file_path.find_last_of('/');
//  std::string path = current_file_path.substr(0, 1 + (int) last_slash_pos);
//  maggieDebug2("filename_with_path:'%s', path:'%s'",current_file_path.c_str(), path.c_str());
//  return absolutePath(filename, path);
//}

} // end namespace StringUtils

#endif /* STRINGUTILS_H_ */

