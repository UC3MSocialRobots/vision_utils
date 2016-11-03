/*!
  \file        google_images_search.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/5/16

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

Some utils to retrieve images from Google Images,
retrieve similar images, etc.

 */

#ifndef GOOGLE_IMAGES_SEARCH_H
#define GOOGLE_IMAGES_SEARCH_H

// OpenCV
#include <opencv2/highgui/highgui.hpp>
// CURL
#include <curl/curl.h>
// utils
#include "vision_utils/cast_to_string.h"
#include "vision_utils/exec_system.h"
#include "vision_utils/extract_utils.h"
#include "vision_utils/extract_utils.h"
#include "vision_utils/retrieve_url.h"
#include "vision_utils/save_file.h"
#include "vision_utils/timer.h"
#include "vision_utils/timestamp.h"

namespace vision_utils {

static const std::string tmp_filename = "/tmp/google_images_upload.jpg";
static const std::string similar_image_tmp_path = "/tmp/google_images_similar";

enum GoogleImagesLookStatus {
  LOOKUP_FAIL = 1,
  SIMILAR_IMAGES_NO_BEST_GUESS_NO = 2,
  SIMILAR_IMAGES_OK_BEST_GUESS_NO = 3,
  SIMILAR_IMAGES_NO_BEST_GUESS_OK = 4,
  SIMILAR_IMAGES_OK_BEST_GUESS_OK = 5
};

////////////////////////////////////////////////////////////////////////////////

/*!
  Call Google Images API to get the best guess about an image.
 \param img
    The image to identify.
 \param best_guess
    The answer of Google Images API. \example "An apple"
 \return bool
    true if success
 \param similar_images
    a vector that will be populated with the similar images of Google Images.
 \param want_similar_images
    false to skip the retrieval of \a similar_images
 \return GoogleImagesLookStatus
    one of API_FAIL, SIMILAR_IMAGES_OK_BEST_GUESS_NO, SIMILAR_IMAGES_OK_BEST_GUESS_OK,
    SIMILAR_IMAGES_NO_BEST_GUESS_NO, SIMILAR_IMAGES_NO_BEST_GUESS_OK

*/
inline GoogleImagesLookStatus google_images_lookup
(const cv::Mat & img,
 std::string & best_guess,
 std::vector<cv::Mat> & similar_images,
 bool want_similar_images = true,
 unsigned int max_similar_images = 100) {
  // cf http://curl.haxx.se/libcurl/c/postit2.html
  //Timer timer;

  // write the image into a file
  std::vector<int> params;
  params.push_back(CV_IMWRITE_JPEG_QUALITY);
  params.push_back(80);
  cv::imwrite(tmp_filename, img, params);
  //timer.printTime("after imwrite()");

  // build petition for google images
  CURL *curl = NULL;
  struct curl_httppost *formpost=NULL;
  struct curl_httppost *lastptr=NULL;
  struct curl_slist *headerlist=NULL;
  CURLcode res;

  static const char buf[] = "Expect:";

  if (curl == NULL) {
    curl_global_init(CURL_GLOBAL_ALL);
    /* Fill in the filename field */
    curl_formadd(&formpost, &lastptr,
                 CURLFORM_COPYNAME, "image_url",
                 CURLFORM_COPYCONTENTS, "",
                 CURLFORM_END);
    curl_formadd(&formpost, &lastptr,
                 CURLFORM_COPYNAME, "btnG",
                 CURLFORM_COPYCONTENTS, "",
                 CURLFORM_END);
    curl_formadd(&formpost, &lastptr,
                 CURLFORM_COPYNAME, "image_content",
                 CURLFORM_COPYCONTENTS, "",
                 CURLFORM_END);
    curl_formadd(&formpost, &lastptr,
                 CURLFORM_COPYNAME, "filename",
                 CURLFORM_COPYCONTENTS, "",
                 CURLFORM_END);
    curl_formadd(&formpost, &lastptr,
                 CURLFORM_COPYNAME, "hl",
                 CURLFORM_COPYCONTENTS, "en",
                 CURLFORM_END);
    curl_formadd(&formpost, &lastptr,
                 CURLFORM_COPYNAME, "safe",
                 CURLFORM_COPYCONTENTS, "off",
                 CURLFORM_END);

    /* Fill in the submit field too, even if this is rarely needed */
    curl_formadd(&formpost, &lastptr,
                 CURLFORM_COPYNAME, "submit",
                 CURLFORM_COPYCONTENTS, "send",
                 CURLFORM_END);
  } // end init

  /* Fill in the file upload field */
  curl_formadd(&formpost, &lastptr,
               CURLFORM_COPYNAME, "encoded_image",
               CURLFORM_FILE, tmp_filename.c_str(),
               CURLFORM_END);
  curl_formadd(&formpost, &lastptr,
               CURLFORM_COPYNAME, "bih",
               CURLFORM_COPYCONTENTS,
               cast_to_string(img.rows).c_str(),
               CURLFORM_END);
  curl_formadd(&formpost, &lastptr,
               CURLFORM_COPYNAME, "biw",
               CURLFORM_COPYCONTENTS,
               cast_to_string(img.cols).c_str(),
               CURLFORM_END);

  /* initalize custom header list (stating that Expect: 100-continue is not
         wanted */
  headerlist = curl_slist_append(headerlist, buf);

  curl = curl_easy_init();
  if (!curl) {
    printf("curl_easy_init() failed!");
    return LOOKUP_FAIL;
  }

  /* what URL that receives this POST */
  curl_easy_setopt(curl, CURLOPT_URL, "http://images.google.com/searchbyimage/upload");
  /* only disable 100-continue header if explicitly requested */
  //    if ( (argc == 2) && (!strcmp(argv[1], "noexpectheader")) )
  //      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
  curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);

  ///
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_writer);
  std::string first_step_content;
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &first_step_content);
  ///

  //timer.printTime("after CURL configuration.");

  // really call the API
  res = curl_easy_perform(curl);
  //timer.printTime("after curl_easy_perform().");
  if (res != CURLE_OK) {
    printf("Error: [%i]\n", res);
    return LOOKUP_FAIL;
  }
  //printf("res:%i, first_step_content:'%s'", res, first_step_content.c_str());

  /* then cleanup the formpost chain */
  //curl_formfree(formpost);
  /* free slist */
  //curl_slist_free_all (headerlist);
  /* always cleanup */
  //curl_easy_cleanup(curl);
  //timer.printTime("after curl_easy_cleanup().");

  ///
  // extract the URL
  int url_pos = 0;
  std::string results_url = extract_from_tags
      (first_step_content, "HREF=\"", "\"", url_pos);
  //printf("results_url:'%s'", results_url.c_str());

  // get the result page
  std::string results_content;
  bool results_retrieve_success = retrieve_url
      (results_url, results_content);
  if (!results_retrieve_success) {
    printf("Could not get the results URL '%s'\n", results_url.c_str());
    return LOOKUP_FAIL;
  }
  //timer.printTime("after retrieving the results page.");


  //printf("results_content:'%s'", results_content.c_str());
  // write content in a file
  std::ostringstream html_filename;
  html_filename << "/tmp/google_images_lookup_" << timestamp() << ".html";
  save_file(html_filename.str(), results_content);
  //timer.printTime("after saving a html copy of the results page.");

  // extract best guess
  std::string best_guess_header = "Best guess";
  std::string::size_type best_guess_header_pos =
      results_content.find(best_guess_header);
  if (best_guess_header_pos == std::string::npos) {
    printf("Could not find best_guess_header='%s'\n", best_guess_header.c_str());
    best_guess = "";
  }
  else { // found the best_guess_header
    int best_guess_header_pos_int = best_guess_header_pos;
    best_guess = extract_from_tags
        (results_content, ">", "<", best_guess_header_pos_int);
  }
  //timer.printTime("after finding best_guess.");

  if (!want_similar_images) {
    return (best_guess == "" ? SIMILAR_IMAGES_NO_BEST_GUESS_NO
                             : SIMILAR_IMAGES_NO_BEST_GUESS_OK);
  }

  // find similar images
  std::string visually_similar_header = "Visually similar images";
  std::string::size_type visually_similar_header_pos =
      results_content.find(visually_similar_header);
  if (visually_similar_header_pos == std::string::npos) {
    printf("Could not find visually_similar_header='%s'\n",
                visually_similar_header.c_str());
  }
  else { // found the visually_similar_header
    int img_url_pos = visually_similar_header_pos;
    similar_images.clear();
    while (similar_images.size() < max_similar_images) {
      // get the URL
      std::string visually_similar_url = extract_from_tags
          (results_content, "imgurl=", "&", img_url_pos);
      //printf("visually_similar_url:'%s'", visually_similar_url.c_str());
      if (visually_similar_url == "")
        break;
      // retrieve the file
      std::ostringstream cmd;
      cmd << "wget " << visually_similar_url
          << " -O " << similar_image_tmp_path
          << " --quiet --timeout=1";
      vision_utils::exec_system(cmd.str());
      cv::Mat similar_image = cv::imread(similar_image_tmp_path);
      if (similar_image.data == NULL) {
        printf("Could not load URL '%s'\n", visually_similar_url.c_str());
        continue;
      }
      similar_images.push_back(similar_image);
    } // end while (true)
  } // end if found visually_similar_header

  //timer.printTime("after finding similar images.");

  if (similar_images.size() > 0)
    return (best_guess == "" ? SIMILAR_IMAGES_OK_BEST_GUESS_NO
                             : SIMILAR_IMAGES_OK_BEST_GUESS_OK);
  else
    return (best_guess == "" ? SIMILAR_IMAGES_NO_BEST_GUESS_NO
                             : SIMILAR_IMAGES_NO_BEST_GUESS_OK);
} // end google_images_lookup()

////////////////////////////////////////////////////////////////////////////////

/*!
  Call Google Images API to get the best guess about an image.
 \param img
    The image to identify.
 \param best_guess
    The answer of Google Images API. \example "An apple"
 \return GoogleImagesLookStatus
    one of API_FAIL, SIMILAR_IMAGES_OK_BEST_GUESS_NO, SIMILAR_IMAGES_OK_BEST_GUESS_OK,
    SIMILAR_IMAGES_NO_BEST_GUESS_NO, SIMILAR_IMAGES_NO_BEST_GUESS_OK
*/
inline GoogleImagesLookStatus google_images_best_guess(const cv::Mat & img,
                                                       std::string & best_guess) {
  std::vector<cv::Mat> unused;
  return google_images_lookup(img, best_guess, unused, false);
}

////////////////////////////////////////////////////////////////////////////////


} // end namespace vision_utils

#endif // GOOGLE_IMAGES_SEARCH_H
