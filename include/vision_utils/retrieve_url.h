/*!
  \file        retrieve_url.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/3
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

#ifndef RETRIEVE_URL_H
#define RETRIEVE_URL_H
// std includes
#include <stdio.h> // for printf(), etc
#include <string>
#include "vision_utils/find_and_replace.h"

namespace vision_utils {

//! This is the writer call back function used by curl
int curl_writer(char *data, size_t size, size_t nmemb, std::string *buffer) {
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

/*! get the content of a distant url
 * \param url for instance http://www.google.es
 * \param ans where we save the answer
 * \return true if success, false otherwise
 */
bool retrieve_url(const std::string & url, std::string & ans) {
  //printf("retrieve_url('%s')", extract(url).c_str());
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
      printf("Could not initialize curl!");
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
      printf("Error: [%i] : '%s'\n", result, errorBuffer);
      return false;
    }

  std::string extract = ans.substr(0, 50);
  find_and_replace(extract, "\n", " ");
  //printf("Bytes fetched:%i, beginning:'%s'", (int) ans.length(), extract.c_str());
  return true;
}

} // end namespace vision_utils

#endif // RETRIEVE_URL_H
