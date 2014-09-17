#include "xml_images_reader.h"
#include <vision_utils/img_path.h>

#include "src/debug/error.h"

class FooXmlImagesReader : public XmlImagesReader {
public:
  //! \see XmlImagesReader::from_xml_node_custom()
  void from_xml_node_custom(const XmlDocument & doc, XmlDocument::Node* node) {
    _nb_reads = doc.get_node_attribute(node, "nb_reads", 0);
  }

  //! \see XmlImagesReader::to_xml_node_custom()
  void to_xml_node_custom(XmlDocument & doc, XmlDocument::Node* node) const {
    doc.set_node_attribute(node, "nb_reads", _nb_reads);
  }

  //! a function that uses the data
  void do_stuff() {
    maggieDebug2("do_stuff()");
    // increment the number of reads
    ++_nb_reads;
  }

private:
  //! a foo attribute, the number of reads
  int _nb_reads;
};

////////////////////////////////////////////////////////////////////////////////

int main() {
  maggieDebug2("main()");
  FooXmlImagesReader reader;
  reader.from_xml_file(IMG_DIR "pz/", "pz.xml");

  for (int file_change = 0; file_change < 10; ++file_change) {
    maggieDebug2("current_file:\n'%s'",
                 reader.get_current_filenode_to_string().c_str());
    reader.do_stuff();
    do {
      reader.go_to_next_file();
    } while (!reader.is_current_file_valid());
    cv::imshow("win1", *reader.get_current_cv_img());
    cv::waitKey(0);
  } // end loop file_change
  return 0;
}

