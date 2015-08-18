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
#include "vision_utils/io.h"
#include "geom/geometry_utils.h"

class FooPointIo : public geometry_utils::FooPoint2f {
public:
  FooPointIo() : geometry_utils::FooPoint2f() {}

  FooPointIo(const float & x_, const float & y_) :
    geometry_utils::FooPoint2f(x_, y_) {
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
  image_utils::to_yaml(pt1, "/tmp/foo", "FooPointIo");
  image_utils::from_yaml(pt1_clone, "/tmp/foo", "FooPointIo");

  printf("pt1:%s, pt1_clone:%s\n\n",
         geometry_utils::printP2(pt1).c_str(),
         geometry_utils::printP2(pt1_clone).c_str());

  std::vector<FooPointIo> pts1, pts2;
  pts1.push_back(pt1);
  pts1.push_back(pt2);
  pts1.push_back(pt1_clone);
  image_utils::to_yaml_vector(pts1, "/tmp/foo", "FooPointIoVector");
  image_utils::from_yaml_vector(pts2, "/tmp/foo", "FooPointIoVector");
  printf("pts1:%s, pts2:%s\n",
         StringUtils::accessible_to_string(pts1).c_str(),
         StringUtils::accessible_to_string(pts2).c_str());
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
