/*!
  \file        getShortURL.h
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

#ifndef GETSHORTURL_H
#define GETSHORTURL_H
// std includes
#include <sstream> // for ostringstream
#include <stdio.h> // for printf(), etc
#include <string>

namespace vision_utils {

//! return a short URL of long URL (using google shortener)
std::string getShortURL(const std::string & longURL){
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
  //printf("Executing this command: %s.\n\n\n", request.str().c_str());
  int return_value = system(request.str().c_str());

  if (return_value != 0){
      printf("Was impossible to get short URL of %s\n", longURL.c_str());
      return "";
    }else{
      retrieve_file(wget_output_filename, output_content);
      //printf("Google shortener response: %s\n",output_content.c_str());

      //get the short url
      int pos = 0;

      std::string shortURL = extract_from_tags(output_content,"id\": ", ",", pos);

      return shortURL;
    }
}

} // end namespace vision_utils

#endif // GETSHORTURL_H
