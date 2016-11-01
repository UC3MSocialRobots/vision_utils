/*!
  \file        string_encoding.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/8/22

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

\todo Description of the file
 */
#ifndef STRING_ENCODING_H
#define STRING_ENCODING_H

#include <string>

namespace vision_utils {

/*! Convert the text to url encode, in orde to use, for example,  on a url request
 *  from http://www.zedwood.com/article/111/cpp-urlencode-function
 * \param texto to convert
 * \example España -> Espa%C3%B1a
 * \return urlEncoded text
 */
std::string toUrlEncode (const std::string & text){
  // convert url encoding - http://curl.haxx.se/libcurl/c/curl_easy_escape.html
  // Create our curl handle
  CURL *curl;
  curl = curl_easy_init();
  if (!curl) {
      printf("Could not initialize curl!");
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
  //printf("%s: ", what);
  for (i = 0; i < len; i++) {
    //printf("%02X", (unsigned char) a[i]);
    if (i < len - 1)
      //printf(" ");
  }
  //printf("\n");
}

////////////////////////////////////////////////////////////////////////////////

/* Display values, for the purpose of showing what this is doing. */
void show_values(const char * before_after, const char * euc_start,
                 int len_start, const char * utf8_start, int utf8len_start) {
  //printf("%s:\n", before_after);
  showhex("EUC-JP string", euc_start, len_start);
  showhex("UTF-8 string", utf8_start, utf8len_start);
}

////////////////////////////////////////////////////////////////////////////////

/* Initialize the library. */
iconv_t initialize(const char * EUCSET, const char * OUTSET) {
  iconv_t conv_desc;
  conv_desc = iconv_open(OUTSET, EUCSET);
  //if ((int) conv_desc == -1) {
  if (conv_desc == NULL) {
    /* Initialization failure. */
    if (errno == EINVAL) {
      //printf("Conversion from '%s' to '%s' is not supported.\n",
                   EUCSET, OUTSET);
    } else {
      //printf("Initialization failure: %s\n", strerror(errno));
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
    //printf("Input std::string is empty.");
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
    //printf("iconv failed: in std::string '%s', length %d, "
                 "out std::string '%s', length %d", euc, (int) len, utf8start,
                 (int) utf8len);
    switch (errno) {
      /* See "man 3 iconv" for an explanation. */
      case EILSEQ:
        throw std::invalid_argument("Invalid multibyte sequence.\n");
        break;
      case EINVAL:
        throw std::invalid_argument("Incomplete multibyte sequence.\n");
        break;
      case E2BIG:
        throw std::invalid_argument("No more room.\n");
        break;
      default:
        throw std::invalid_argument("Error: %s.\n", strerror(errno));
    }
    exit(1);
  }
  /* Display what is in the variables after calling iconv. */
  //show_values("after", euc_start, len_start, utf8start, utf8len_start);
  return std::string(utf8start);
}

////////////////////////////////////////////////////////////////////////////////

/* Close the connection with the library. */

void finalize(iconv_t conv_desc) {
  int v = iconv_close(conv_desc);
  if (v != 0) {
    //printf("iconv_close failed: %s\n", strerror(errno));
  }
}

////////////////////////////////////////////////////////////////////////////////

/*! change the encoding of a string
 * \param content the string to change
 * \param EUCSET the initial encoding of the string
 * \param OUTSET the encoding we want in output
 */
void convert_string_encoding(std::string & content,
                             const char * EUCSET, const char * OUTSET) {

  /* Conversion descriptor. */
  iconv_t conv_desc = initialize(EUCSET, OUTSET);

  content = euc2utf8(conv_desc, (char*) content.c_str());
  finalize(conv_desc);

  //  if (out_string)
  //      //printf("Final iconv output: %s\n", out_string);
}

////////////////////////////////////////////////////////////////////////////////

/*! change the encoding of a string UTF8 -> ISO-8859-1
 * \param content the string to change
 */
void convert_string_encoding_utf_to_iso(std::string & content) {
  convert_string_encoding(content, "UTF-8", "ISO-8859-1");
}

////////////////////////////////////////////////////////////////////////////////

/*! change the encoding of a string ISO-8859-1 -> UTF-8
 * \param content the string to change
 */
void convert_string_encoding_iso_to_utf(std::string & content) {
  convert_string_encoding(content, "ISO-8859-1", "UTF-8");
}

////////////////////////////////////////////////////////////////////////////////

/*! convert the HTML entities of a string -> UTF-8
 * \param content the string to change
 */
void convert_string_encoding_htlm_to_utf(std::string & content) {
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
  find_and_replace(content, "\"", "\\\""); // escape "
  // we need to recode content to iso
  std::string content_iso = content;
  convert_string_encoding_utf_to_iso(content_iso);
  std::ostringstream command;
  // remove final line break with "-n"
  command << "echo -n \"" << content_iso << "\" | recode HTML_4.0..UTF-8";
  content = exec_system_get_output(command.str().c_str());
#endif
}

} // end namespace vision_utils

#endif // STRING_ENCODING_H

