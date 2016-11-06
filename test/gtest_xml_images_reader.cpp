/*!
  \file        gtest_test_xml_images_reader.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/8/22

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

Some tests for XmlImage

 */
// Bring in gtest
#include <gtest/gtest.h>
#include <vision_utils/img_path.h>
#include "vision_utils/xml_images_reader.h"

class FooXmlImagesReader : public vision_utils::XmlImagesReader {
public:
  //! \see XmlImagesReader::from_xml_node_custom()
  void from_xml_node_custom(const vision_utils::XmlDocument & doc, vision_utils::XmlDocument::Node* node) {
    _nb_reads = doc.get_node_attribute(node, "nb_reads", 0);
    // increment the number of reads
    ++_nb_reads;
  }

  //! \see XmlImagesReader::to_xml_node_custom()
  void to_xml_node_custom(vision_utils::XmlDocument & doc, vision_utils::XmlDocument::Node* node) const {
    doc.set_node_attribute(node, "nb_reads", _nb_reads);
  }

  //! a foo attribute, the number of reads
  int _nb_reads;
};


////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_object) {
  FooXmlImagesReader reader;
  EXPECT_TRUE(reader.get_nb_files() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_move_around) {
  FooXmlImagesReader reader;
  reader.go_to_next_file();
  EXPECT_TRUE(reader.get_current_file_index() == 0);
  reader.go_to_previous_file();
  EXPECT_TRUE(reader.get_current_file_index() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, missing_file) {
  FooXmlImagesReader reader;
  bool read_success = reader.from_xml_file("/tmp", "non_existing_file.xml");
  EXPECT_TRUE(!read_success);
  EXPECT_TRUE(reader.get_nb_files() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_content) {
  FooXmlImagesReader reader;
  bool read_success = reader.from_xml_content("/tmp", "");
  EXPECT_TRUE(read_success);
  EXPECT_TRUE(reader.get_nb_files() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_content2) {
  FooXmlImagesReader reader;
  bool read_success = reader.from_xml_content("/tmp", "<?xml version=\"1.0\" ?>");
  EXPECT_TRUE(read_success);
  EXPECT_TRUE(reader.get_nb_files() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, empty_content3) {
  FooXmlImagesReader reader;
  bool read_success = reader.from_xml_content("/tmp", "<?xml version=\"1.0\" ?><images/>");
  EXPECT_TRUE(read_success);
  EXPECT_TRUE(reader.get_nb_files() == 0);
  // cv::imshow("test", *reader.get_current_cv_img()); cv::waitKey(0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, single_file) {
  std::string filename1 = "balloon.png";
  std::ostringstream content;
  content << "<images>";
  content << "<image filename=\"" << filename1 << "\"/>";
  content << "</images>";
  FooXmlImagesReader reader;
  bool read_success = reader.from_xml_content(vision_utils::IMG_DIR(), content.str());
  EXPECT_TRUE(read_success);
  EXPECT_TRUE(reader.get_nb_files() == 1);
  //cv::imshow("test", *reader.get_current_cv_img()); cv::waitKey(0);

  // check it is the same image with a file reading
  EXPECT_TRUE(reader.get_current_file_index() == 0);
  EXPECT_TRUE(reader.get_current_filename() == filename1);
  cv::Mat3b im = cv::imread(vision_utils::IMG_DIR() + filename1, CV_LOAD_IMAGE_COLOR);
  EXPECT_TRUE(im.size() == reader.get_current_cv_img()->size());

  // move around
  reader.go_to_next_file();
  EXPECT_TRUE(reader.get_current_file_index() == 0);
  reader.go_to_previous_file();
  EXPECT_TRUE(reader.get_current_file_index() == 0);
}

////////////////////////////////////////////////////////////////////////////////

TEST(TestSuite, two_files) {
  std::string filename1 = "balloon.png";
  std::string filename2 = "frenadol.png";
  std::ostringstream content;
  content << "<images>";
  content << "<image filename=\"" << filename1 << "\"/>";
  content << "<image filename=\"" << filename2 << "\"/>";
  content << "</images>";
  FooXmlImagesReader reader;
  bool read_success = reader.from_xml_content(vision_utils::IMG_DIR(), content.str());
  EXPECT_TRUE(read_success);
  EXPECT_TRUE(reader.get_nb_files() == 2);
  //cv::imshow("test", *reader.get_current_cv_img()); cv::waitKey(0);

  // move around
  EXPECT_TRUE(reader.get_current_file_index() == 0);
  EXPECT_TRUE(reader.get_current_filename() == filename1);
  EXPECT_TRUE(reader._nb_reads == 1);
  reader.go_to_next_file(); // 1 -> 2
  EXPECT_TRUE(reader.get_current_file_index() == 1);
  EXPECT_TRUE(reader.get_current_filename() == filename2);
  EXPECT_TRUE(reader._nb_reads == 1);
  reader.go_to_next_file(); // 2 -> 1 forwards loop
  EXPECT_TRUE(reader.get_current_file_index() == 0);
  EXPECT_TRUE(reader.get_current_filename() == filename1);
  EXPECT_TRUE(reader._nb_reads == 2);
  reader.go_to_previous_file(); // 1 -> 2 backwards loop
  EXPECT_TRUE(reader.get_current_file_index() == 1);
  EXPECT_TRUE(reader.get_current_filename() == filename2);
  EXPECT_TRUE(reader._nb_reads == 2);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){
  // Run all the tests that were declared with TEST()
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
