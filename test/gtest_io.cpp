/*!
  \file        test_io.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/12/19

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
#include <gtest/gtest.h>
#include "vision_utils/accessible_to_string.h"
#include "vision_utils/foo_point.h"
#include "vision_utils/from_yaml.h"
#include "vision_utils/from_yaml_vector.h"
#include "vision_utils/printP2.h"
#include "vision_utils/to_yaml.h"
#include "vision_utils/to_yaml_vector.h"

class FooPointIo : public vision_utils::FooPoint2f {
public:
  FooPointIo() : vision_utils::FooPoint2f() {}

  FooPointIo(const float & x_, const float & y_) :
    vision_utils::FooPoint2f(x_, y_) {
  }

  /*!
   * Save to a stream.
   * CF http://docs.opencv.org/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html
   * \param fs
   * \param save_illus_images
   */
  void write(cv::FileStorage &fs) const {
    fs << "x" << x;
    fs << "y" << y;
  } // end write()

  //////////////////////////////////////////////////////////////////////////////

  void read(const cv::FileNode &fn) {
    fn["x"] >> x;
    fn["y"] >> y;
  } // end read
}; // end class FooPointIo

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, test1) {
  FooPointIo pt1(1,2), pt1_clone, pt2(3,4);
  vision_utils::to_yaml(pt1, "/tmp/foo", "FooPointIo");
  vision_utils::from_yaml(pt1_clone, "/tmp/foo", "FooPointIo");

  printf("pt1:%s, pt1_clone:%s\n\n",
         vision_utils::printP2(pt1).c_str(),
         vision_utils::printP2(pt1_clone).c_str());

  std::vector<FooPointIo> pts1, pts2;
  pts1.push_back(pt1);
  pts1.push_back(pt2);
  pts1.push_back(pt1_clone);
  vision_utils::to_yaml_vector(pts1, "/tmp/foo", "FooPointIoVector");
  vision_utils::from_yaml_vector(pts2, "/tmp/foo", "FooPointIoVector");
  printf("pts1:%s, pts2:%s\n",
         vision_utils::accessible_to_string(pts1).c_str(),
         vision_utils::accessible_to_string(pts2).c_str());
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
