/*!
 * \file StringUtils.cpp
 *
 * Some utilities for strings
 *
 * \date Dec 5, 2010
 * \author Arnaud Ramey
 */
// maggie
#include "vision_utils/utils/StringUtils.h"
#include "vision_utils/utils/system_utils.h"
//#include "vision_utils/utils/extract_utils.h"
// deps
//#include <recode.h>
// some doc for recode here:
// http://www.informatik.uni-hamburg.de/RZ/software/gnu/utilities/recode_4.html
#include <curl/curl.h>
#include <iconv.h>
// stl
#include <iostream>
#include <fstream>  // file streams
#include <sstream>  // ostringstream
#include <cctype>
#include <algorithm>
// C
#include <stdlib.h> // getenv()
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>

int StringUtils::curl_writer(char *data, size_t size, size_t nmemb,
                             std::string *buffer) {
  // What we will return
  int result = 0;

  // Is there anything in the buffer?
  if (buffer != NULL) {
    // Append the data to the buffer
    buffer->append(data, size * nmemb);

    // How much did we write?
    result = size * nmemb;
  }

  return result;
}

////////////////////////////////////////////////////////////////////////////////

bool StringUtils::retrieve_url(const std::string & url, std::string & ans) {
  maggieDebug2("retrieve_url('%s')", string_utils::extract(url).c_str());
  ans = "";

  // Write any errors in here
  static char errorBuffer[CURL_ERROR_SIZE];

  // Write all expected data in here
  //static std::string buffer;

  // Our curl objects
  CURL *curl;
  CURLcode result;

  // Create our curl handle
  curl = curl_easy_init();

  if (!curl) {
    maggiePrint("Could not initialize curl!");
    return false;
  }

  //$browsers = array(
  //        "Mozilla/5.0 (X11; U; Linux i686; en-US; rv:1.9.0.3) Gecko/2008092510 Ubuntu/8.04 (hardy) Firefox/3.0.3",
  //        "Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US; rv:1.8.1) Gecko/20060918 Firefox/2.0",
  //        "Mozilla/5.0 (Windows; U; Windows NT 6.0; en-US; rv:1.9.0.3) Gecko/2008092417 Firefox/3.0.3",
  //        "Mozilla/4.0 (compatible; MSIE 7.0; Windows NT 6.0; SLCC1; .NET CLR 2.0.50727; Media Center PC 5.0; .NET CLR 3.0.04506)");
  //$referers = array("google.com", "yahoo.com", "msn.com", "ask.com", "live.com");

  // Now set up all of the curl options
  // cheat the google "terms of service" with two fake navigator options
  // http://www.user-agents.org/index.shtml
  // http://www.useragentstring.com/pages/Chrome/
  curl_easy_setopt(curl, CURLOPT_USERAGENT,
                   //"Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US; rv:1.8.1) Gecko/20060918 Firefox/2.0"
                   //"Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US) AppleWebKit/525.19 (KHTML, like Gecko) Chrome/0.2.153.1 Safari/525.19  "
                   "Mozilla/5.0 (Windows NT 6.1; WOW64) AppleWebKit/536.6 (KHTML, like Gecko) Chrome/20.0.1092.0 Safari/536.6"
                   );
  curl_easy_setopt(curl, CURLOPT_REFERER, "google.com");
  curl_easy_setopt(curl, CURLOPT_COOKIEFILE, "cookie.txt");
  curl_easy_setopt(curl, CURLOPT_COOKIEJAR, "cookie.txt");
  // now the real options
  curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, errorBuffer);
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  // CURLOPT_HEADER - TRUE to include the header in the output.
  curl_easy_setopt(curl, CURLOPT_HEADER, 0); // 0 before
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_writer);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &ans);

  // Attempt to retrieve the remote page
  result = curl_easy_perform(curl);

  // Always cleanup
  curl_easy_cleanup(curl);

  // Did we succeed?
  if (result != CURLE_OK) {
    maggiePrint("Error: [%i] : '%s'", result, errorBuffer);
    return false;
  }

  std::string extract = ans.substr(0, 50);
  find_and_replace(extract, "\n", " ");
  maggieDebug3("Bytes fetched:%i, beginning:'%s'", (int) ans.length(), extract.c_str());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

std::string StringUtils::toUrlEncode(const std::string &c){
  // convert url encoding - http://curl.haxx.se/libcurl/c/curl_easy_escape.html
  // Create our curl handle
  CURL *curl;
  curl = curl_easy_init();
  if (!curl) {
    maggiePrint("Could not initialize curl!");
    return c;
  }
  std::string ans(curl_easy_escape(curl, c.c_str(), 0));
  curl_easy_cleanup(curl);
  return ans;
}

////////////////////////////////////////////////////////////////////////////////

/*
 * http://www.lemoda.net/c/iconv-example/iconv-example.html
 */

/* Print the hexadecimal bytes. */
void showhex(const char * what, const char * a, int len) {
  int i;
  ROS_DEBUG("%s: ", what);
  for (i = 0; i < len; i++) {
    ROS_DEBUG("%02X", (unsigned char) a[i]);
    if (i < len - 1)
      ROS_DEBUG(" ");
  }
  ROS_DEBUG("\n");
}

////////////////////////////////////////////////////////////////////////////////

/* Display values, for the purpose of showing what this is doing. */
void show_values(const char * before_after, const char * euc_start,
                 int len_start, const char * utf8_start, int utf8len_start) {
  ROS_DEBUG("%s:\n", before_after);
  showhex("EUC-JP string", euc_start, len_start);
  showhex("UTF-8 string", utf8_start, utf8len_start);
}

/* Initialize the library. */
iconv_t initialize(const char * EUCSET, const char * OUTSET) {
  iconv_t conv_desc;
  conv_desc = iconv_open(OUTSET, EUCSET);
  //if ((int) conv_desc == -1) {
  if (conv_desc == NULL) {
    /* Initialization failure. */
    if (errno == EINVAL) {
      maggieDebug1("Conversion from '%s' to '%s' is not supported.\n",
                   EUCSET, OUTSET);
    } else {
      maggieDebug1("Initialization failure: %s\n", strerror(errno));
    }
    exit(1);
  }
  return conv_desc;
}

////////////////////////////////////////////////////////////////////////////////

/* Convert EUC into UTF-8 using the iconv library. */

std::string euc2utf8(iconv_t conv_desc, const char * euc) {
  size_t iconv_value;
  char * utf8;
  //unsigned int len;
  //unsigned int utf8len;
  size_t len, utf8len;
  /* The variables with "start" in their name are solely for display
     of what the function is doing. As iconv runs, it alters the
     values of the variables, so these are for keeping track of the
     start points and start lengths. */
  char * utf8start;
  //const char * euc_start;
  //int len_start;
  //int utf8len_start;

  len = strlen(euc);
  if (!len) {
    maggieDebug1("Input std::string is empty.");
    return "";
  }
  /* Assign enough space to put the UTF-8. */
  utf8len = 2 * len;
  utf8 = (char*) calloc(utf8len, 1);
  /* Keep track of the variables. */
  utf8start = utf8;
  //len_start = len;
  //utf8len_start = utf8len;
  //euc_start = euc;
  /* Display what is in the variables before calling iconv. */
  //show_values("before", euc_start, len_start, utf8start, utf8len_start);
  iconv_value = iconv(conv_desc, (char**) &euc, &len, &utf8, &utf8len);
  /* Handle failures. */
  if (iconv_value == (size_t) -1) {
    maggieDebug1("iconv failed: in std::string '%s', length %d, "
                 "out std::string '%s', length %d", euc, (int) len, utf8start,
                 (int) utf8len);
    switch (errno) {
      /* See "man 3 iconv" for an explanation. */
      case EILSEQ:
        maggieError("Invalid multibyte sequence.\n");
        break;
      case EINVAL:
        maggieError("Incomplete multibyte sequence.\n");
        break;
      case E2BIG:
        maggieError("No more room.\n");
        break;
      default:
        maggieError("Error: %s.\n", strerror(errno));
    }
    exit(1);
  }
  /* Display what is in the variables after calling iconv. */
  //show_values("after", euc_start, len_start, utf8start, utf8len_start);
  return std::string(utf8start);
}

/* Close the connection with the library. */

void finalize(iconv_t conv_desc) {
  int v = iconv_close(conv_desc);
  if (v != 0) {
    maggieDebug1("iconv_close failed: %s\n", strerror(errno));
  }
}

////////////////////////////////////////////////////////////////////////////////

void StringUtils::convert_string_encoding(std::string & content, const char *EUCSET,
                                          const char *OUTSET) {

  /* Conversion descriptor. */
  iconv_t conv_desc = initialize(EUCSET, OUTSET);

  content = euc2utf8(conv_desc, (char*) content.c_str());
  finalize(conv_desc);

  //  if (out_string)
  //      ROS_DEBUG("Final iconv output: %s\n", out_string);
}

////////////////////////////////////////////////////////////////////////////////

void StringUtils::convert_string_encoding_utf_to_iso(std::string & content) {
  convert_string_encoding(content, "UTF-8", "ISO-8859-1");
}

////////////////////////////////////////////////////////////////////////////////

void StringUtils::convert_string_encoding_iso_to_utf(std::string & content) {
  convert_string_encoding(content, "ISO-8859-1", "UTF-8");
}

////////////////////////////////////////////////////////////////////////////////

void StringUtils::convert_string_encoding_htlm_to_utf(std::string & content) {
#if 0 // use lib - does not work, give a segfault (buggy lib?)
  RECODE_OUTER outer = recode_new_outer (true);
  RECODE_REQUEST request = recode_new_request (outer);
  const char* code = "HTML_4.0";
  recode_scan_request (request, code);
  char * content_out = new char [content.length()+1];
  std::strcpy (content_out, content.c_str());
  size_t output_allocated, output_length = content.size() + 1;
  recode_string_to_buffer(request, content.c_str(),
                          &content_out, &output_length, &output_allocated);
  content = std::string(content_out);
  delete content_out;
  recode_delete_request (request);
  recode_delete_outer (outer);
#else // make a system call
  StringUtils::find_and_replace(content, "\"", "\\\""); // escape "
  // we need to recode content to iso
  std::string content_iso = content;
  convert_string_encoding_utf_to_iso(content_iso);
  std::ostringstream command;
  // remove final line break with "-n"
  command << "echo -n \"" << content_iso << "\" | recode HTML_4.0..UTF-8";
  content = system_utils::exec_system_get_output(command.str().c_str());
#endif
}


////////////////////////////////////////////////////////////////////////////////


std::string StringUtils::absolutePath(const char* filename,
                                      const std::string prefix) {
  std::string abs_path = prefix + filename;
  std::vector<std::string> folders;
  StringSplit(abs_path, "/", &folders);

  /* eliminate the ".." from the vector 'folders' */
  std::vector<std::string> foldersLeft;
  for (unsigned int i = 0; i < folders.size(); i++) {
    //cout << "current word:" << folders.at(i) << endl;
    if (folders.at(i) == "..")
      foldersLeft.pop_back();
    else
      foldersLeft.push_back(folders.at(i));
  }

  /* copy the result in a std::string */
  std::ostringstream rep;
  for (unsigned int i = 0; i < foldersLeft.size(); i++)
    rep << "/" << foldersLeft.at(i);
  //cout << "abs path:" << abs_path << "\t path:" << rep.str() << endl;
  //maggieDebug2("absolutePath('%s', '%s') = '%s'", filename, prefix.c_str(), rep.str().c_str());

  return (char*) rep.str().c_str();
}

////////////////////////////////////////////////////////////////////////////////

std::string StringUtils::extract_from_tags(const std::string & content,
                                           const std::string & block_begin,
                                           const std::string & block_end,
                                           int & initial_search_pos) {
  // find the beginning
  size_t pos_begin = content.find(block_begin, initial_search_pos);
  if (pos_begin == std::string::npos) {
    ROS_DEBUG("block_begin '%s' could not be found", block_begin.c_str());
    return "";
  }
  // remove the block at the beginning
  pos_begin += block_begin.length();
  // find the end
  size_t pos_end = content.find(block_end, pos_begin);
  if (pos_end == std::string::npos) {
    ROS_DEBUG("block_end '%s' could not be found", block_end.c_str());
    return "";
  }
  // update the index
  initial_search_pos = pos_end + block_end.length();
  // extract the substring
  //maggieDebug2("value:'%s'", content.substr(pos_begin, pos_end - pos_begin).c_str());
  std::string ans = content.substr(pos_begin, pos_end - pos_begin);
  //cout << ans << endl;
  /*
     * clean the string
     */
  StringUtils::find_and_replace(ans, "&nbsp;", " ");
  StringUtils::remove_beginning_spaces(ans);
  StringUtils::remove_trailing_spaces(ans);
  return ans;
}

////////////////////////////////////////////////////////////////////////////////

std::string StringUtils::random_string(const int length) {
  ROS_DEBUG("random_string(%i)\n", length);

  // init the random generator
  //srand( time(NULL));

  // add letters in a loop
  std::ostringstream buffer;
  for (int char_idx = 0; char_idx< length; ++char_idx) {
    char new_char = (rand() % 26) + 'a';
    buffer << new_char;
  }

  return buffer.str();
}

////////////////////////////////////////////////////////////////////////////////

void StringUtils::to_lowercase(std::string & sentence) {
  std::transform(sentence.begin(), sentence.end(), sentence.begin(), tolower);
  find_and_replace(sentence, "Ó", "ó");
}

////////////////////////////////////////////////////////////////////////////////

void StringUtils::to_uppercase(std::string & sentence) {
  std::transform(sentence.begin(), sentence.end(), sentence.begin(), toupper);
}

////////////////////////////////////////////////////////////////////////////////

bool StringUtils::contains_any_letter(const std::string & sentence) {
  std::string sentence_lower = sentence;
  to_lowercase(sentence_lower);
  for (unsigned int char_idx = 0; char_idx< sentence_lower.size(); ++char_idx) {
    // 'a' = 97, 'z'=122
    int ascii = (int) sentence_lower.at(char_idx);
    if (ascii >= 97 && ascii <= 122)
      return true;
  }
  return false;
}



////////////////////////////////////////////////////////////////////////////////

std::string StringUtils::getShortURL(const std::string & longURL){


    std::string wget_output_filename="googleShortenerReply.txt";
    std::string googleShortenerULR = "https://www.googleapis.com/urlshortener/v1/url";
    std::ostringstream request;
    std::string output_content;


    request << "wget ";
    request << " --post-data "<<  "'{\"longUrl\": \"" << longURL << "\"}'";
    request << " --header=\"Content-Type: application/json\"";
    request << " --no-check-certificate";
    request << " -O " << wget_output_filename;
    request << " \"" << googleShortenerULR << "\" > /dev/null 2>&1";
    ROS_DEBUG("Executing this command: %s.\n\n\n", request.str().c_str());
    int return_value = system(request.str().c_str());

    if (return_value != 0){
        ROS_ERROR("Was impossible to get short URL of %s", longURL.c_str());
        return "";
    }else{
        StringUtils::retrieve_file(wget_output_filename, output_content);
        ROS_DEBUG ("Google shortener response: %s",output_content.c_str());

        //get the short url
        int pos = 0;

        std::string shortURL = StringUtils::extract_from_tags(output_content,"id\": ", ",", pos);

        return shortURL;
    }
}


