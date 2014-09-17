/*!
  \file        test_person_histogram_set_variables.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/24

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

#ifndef TEST_PERSON_HISTOGRAM_SET_VARIABLES_H
#define TEST_PERSON_HISTOGRAM_SET_VARIABLES_H

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <src/kinect_utils/kinect_openni_utils.h>
#include <vision_utils/img_path.h>

#define DEPTH_DIR  IMG_DIR "depth/"
#define BREAST_DIR  IMG_DIR "breast/"

namespace test_person_histogram_set_variables {

////////////////////////////////////////////////////////////////////////////////
/// images with one user
////////////////////////////////////////////////////////////////////////////////

// juggling: data as array
unsigned int juggling_label = 1, juggling_hists_nb = 3;
std::string juggling1_file(DEPTH_DIR "juggling1"),
juggling2_file(DEPTH_DIR "juggling2"),
juggling3_file(DEPTH_DIR "juggling3");
std::string juggling_filename_prefixes_array[] = {juggling1_file, juggling2_file, juggling3_file};
cv::Point juggling1_seed = cv::Point(370, 70),
juggling2_seed = cv::Point(400, 70),
juggling3_seed = cv::Point(360, 90);
cv::Point juggling_seeds_array[] = {juggling1_seed, juggling2_seed, juggling3_seed};
// convert it to vectors
std::vector<std::string> juggling_filename_prefixes
(juggling_filename_prefixes_array, juggling_filename_prefixes_array + juggling_hists_nb);
std::vector<std::string> juggling_kinect_serials(juggling_hists_nb, KINECT_SERIAL_LAB());
std::vector<cv::Point> juggling_seeds(juggling_seeds_array, juggling_seeds_array + juggling_hists_nb);
std::vector<int> juggling_labels(juggling_hists_nb, juggling_label);

// alberto: data as array
unsigned int alberto_label = 2, alberto_hists_nb = 2;
std::string  alberto1_file(DEPTH_DIR "alberto1"),
alberto2_file(DEPTH_DIR "alberto2");
std::string alberto_filename_prefixes_array[] =  {alberto1_file, alberto2_file};
cv::Point alberto1_seed = cv::Point(370, 70),
alberto2_seed = cv::Point(400, 85);
cv::Point alberto_seeds_array[] = {alberto1_seed, alberto2_seed};
// convert it to vectors
std::vector<std::string> alberto_filename_prefixes
(alberto_filename_prefixes_array, alberto_filename_prefixes_array + alberto_hists_nb);
std::vector<std::string> alberto_kinect_serials(alberto_hists_nb, KINECT_SERIAL_LAB());
std::vector<cv::Point> alberto_seeds(alberto_seeds_array, alberto_seeds_array + alberto_hists_nb);
std::vector<int> alberto_labels(alberto_hists_nb, alberto_label);

// alvaro: data as array
unsigned int alvaro_label = 3, alvaro_hists_nb = 2;
std::string  alvaro1_file(DEPTH_DIR "alvaro1"),
alvaro2_file(DEPTH_DIR "alvaro2");
std::string alvaro_filename_prefixes_array[] =  {alvaro1_file, alvaro2_file};
cv::Point alvaro1_seed = cv::Point(260, 75),
alvaro2_seed = cv::Point(260, 70);
cv::Point alvaro_seeds_array[] = {alvaro1_seed, alvaro2_seed};
// convert it to vectors
std::vector<std::string> alvaro_filename_prefixes
(alvaro_filename_prefixes_array, alvaro_filename_prefixes_array + alvaro_hists_nb);
std::vector<std::string> alvaro_kinect_serials(alvaro_hists_nb, KINECT_SERIAL_LAB());
std::vector<cv::Point> alvaro_seeds(alvaro_seeds_array, alvaro_seeds_array + alvaro_hists_nb);
std::vector<int> alvaro_labels(alvaro_hists_nb, alvaro_label);

// refset: data as array
unsigned int refset_hists_nb = 7;
std::string refset_filename_prefixes_array[] =
{juggling1_file, juggling2_file, juggling3_file,
 alberto1_file, alberto2_file, alvaro1_file, alvaro2_file};
cv::Point refset_seeds_array[] =
{juggling1_seed, juggling2_seed, juggling3_seed, alberto1_seed, alberto2_seed, alvaro1_seed, alvaro2_seed};
// convert it to vectors
std::vector<std::string> refset_filename_prefixes
(refset_filename_prefixes_array, refset_filename_prefixes_array + refset_hists_nb);
std::vector<std::string> refset_kinect_serials(refset_hists_nb, KINECT_SERIAL_LAB());
std::vector<cv::Point> refset_seeds(refset_seeds_array, refset_seeds_array + refset_hists_nb);
std::vector<int> refset_labels() {
  std::vector<int> ans;
  std::copy(juggling_labels.begin(), juggling_labels.end(), std::back_inserter(ans));
  std::copy(alberto_labels.begin(), alberto_labels.end(), std::back_inserter(ans));
  std::copy(alvaro_labels.begin(), alvaro_labels.end(), std::back_inserter(ans));
  return ans;
}

/****************** images with masks *****************************************/
// ainara: data as array
unsigned int ainara_label = 5, ainara_hists_nb = 22;
std::string ainara_filename_prefixes_array[] =  {BREAST_DIR "2013-10-05_15-46-03-286",
                                                 BREAST_DIR "2013-10-05_15-46-03-861",
                                                 BREAST_DIR "2013-10-05_15-46-05-027",
                                                 BREAST_DIR "2013-10-05_15-46-13-769",
                                                 BREAST_DIR "2013-10-05_15-46-14-841",
                                                 BREAST_DIR "2013-10-05_15-46-16-198",
                                                 BREAST_DIR "2013-10-05_15-46-17-635",
                                                 BREAST_DIR "2013-10-05_15-46-18-208",
                                                 BREAST_DIR "2013-10-05_15-46-18-804",
                                                 BREAST_DIR "2013-10-05_15-46-22-545",
                                                 BREAST_DIR "2013-10-05_15-46-23-350",
                                                 BREAST_DIR "2013-10-05_15-46-24-254",
                                                 BREAST_DIR "2013-10-05_15-46-26-315",
                                                 BREAST_DIR "2013-10-05_15-46-27-255",
                                                 BREAST_DIR "2013-10-05_15-46-29-673",
                                                 BREAST_DIR "2013-10-05_15-46-32-351",
                                                 BREAST_DIR "2013-10-05_15-46-35-989",
                                                 BREAST_DIR "2013-10-05_15-46-40-027",
                                                 BREAST_DIR "2013-10-05_15-46-41-242",
                                                 BREAST_DIR "2013-10-05_15-46-45-296",
                                                 BREAST_DIR "2013-10-05_15-46-45-998",
                                                 BREAST_DIR "2013-10-05_15-46-46-901"};
// convert it to vectors
std::vector<std::string> ainara_filename_prefixes
(ainara_filename_prefixes_array, ainara_filename_prefixes_array + ainara_hists_nb);
std::vector<std::string> ainara_kinect_serials(ainara_hists_nb, KINECT_SERIAL_ARNAUD());
std::vector<int> ainara_labels(ainara_hists_nb, ainara_label);
std::vector<int> ainara_user_idx(ainara_hists_nb, 4); // value of user mask in the user image

//////////////////////////////////////////////////////////////////////////////

inline std::vector<std::string> all_single_user_filename_prefixes() {
  std::vector<std::string> ans;
  std::copy(juggling_filename_prefixes.begin(), juggling_filename_prefixes.end(),
            std::back_inserter(ans));
  std::copy(alberto_filename_prefixes.begin(), alberto_filename_prefixes.end(),
            std::back_inserter(ans));
  std::copy(alvaro_filename_prefixes.begin(), alvaro_filename_prefixes.end(),
            std::back_inserter(ans));
  std::copy(ainara_filename_prefixes.begin(), ainara_filename_prefixes.end(),
            std::back_inserter(ans));
  return ans;
}

////////////////////////////////////////////////////////////////////////////////
//// images with several users
////////////////////////////////////////////////////////////////////////////////

// david: data as array
unsigned int david_label = 6, david_hists_nb = 3;
std::string david_filename_prefixes_array[] =
{DEPTH_DIR "david_arnaud1", DEPTH_DIR "david_arnaud2", DEPTH_DIR "david_arnaud3"};
// convert it to vectors
std::vector<std::string> david_filename_prefixes
(david_filename_prefixes_array, david_filename_prefixes_array + david_hists_nb);
std::vector<std::string> david_kinect_serials(david_hists_nb, KINECT_SERIAL_LAB());
std::vector<int> david_labels(david_hists_nb, david_label);
std::vector<int> david_user_idx(david_hists_nb, 1); // value of user mask in the user image

// arnaud: data as array
unsigned int arnaud_label = 7, arnaud_hists_nb = 3;
std::string arnaud_filename_prefixes_array[] =
{DEPTH_DIR "david_arnaud1", DEPTH_DIR "david_arnaud2", DEPTH_DIR "david_arnaud3"};
// convert it to vectors
std::vector<std::string> arnaud_filename_prefixes
(arnaud_filename_prefixes_array, arnaud_filename_prefixes_array + arnaud_hists_nb);
std::vector<std::string> arnaud_kinect_serials(arnaud_hists_nb, KINECT_SERIAL_LAB());
std::vector<int> arnaud_labels(arnaud_hists_nb, arnaud_label);
std::vector<int> arnaud_user_idx(arnaud_hists_nb, 2); // value of user mask in the user image

//////////////////////////////////////////////////////////////////////////////

inline std::vector<std::string> all_multi_user_filename_prefixes() {
  std::vector<std::string> ans;
  std::copy(arnaud_filename_prefixes.begin(), arnaud_filename_prefixes.end(),
            std::back_inserter(ans));
  return ans;
}

/* *****************************************************************************
 *data structured by owner
*******************************************************************************/
inline std::vector<std::vector<std::string> > all_filename_prefixes_struct() {
  std::vector<std::vector<std::string> > ans;
  ans.push_back(juggling_filename_prefixes);
  ans.push_back(alberto_filename_prefixes);
  ans.push_back(alvaro_filename_prefixes);
  return ans;
}
inline std::vector<std::vector<cv::Point> > all_seeds_struct() {
  std::vector<std::vector<cv::Point> > ans;
  ans.push_back(juggling_seeds);
  ans.push_back(alberto_seeds);
  ans.push_back(alvaro_seeds);
  return ans;
}
inline std::vector<std::vector<std::string> > all_kinect_serials_struct() {
  std::vector<std::vector<std::string> > ans;
  ans.push_back(juggling_kinect_serials);
  ans.push_back(alberto_kinect_serials);
  ans.push_back(alvaro_kinect_serials);
  return ans;
}
inline std::vector<int> all_labels_struct() {
  std::vector<int> ans;
  ans.push_back(juggling_label);
  ans.push_back(alberto_label);
  ans.push_back(alvaro_label);
  return ans;
}


} // end namespace test_person_histogram_set_variables

#endif // TEST_PERSON_HISTOGRAM_SET_VARIABLES_H
