// Bring in gtest
#include <gtest/gtest.h>
#include "vision_utils/xml_images_interface.h"
#include "vision_utils/utils/error.h"
#include <vision_utils/img_path.h>

class XmlImageFooInterface : public XmlImagesInterface {
public:
  //! \see XmlImagesReader::from_xml_node_custom()
  void from_xml_node_custom(const XmlDocument & doc, XmlDocument::Node* node) {
    XmlDocument::Node* point_node = doc.get_node_at_direction(node, "interest_point");
    _interest_point.x = doc.get_node_attribute(point_node, "x", 0);
    _interest_point.y = doc.get_node_attribute(point_node, "y", 0);
  }

  //! \see XmlImagesReader::to_xml_node_custom()
  void to_xml_node_custom(XmlDocument & doc, XmlDocument::Node* node) const {
    XmlDocument::Node* point_node = doc.add_node(node, "interest_point", "");
    doc.set_node_attribute(point_node, "x", _interest_point.x);
    doc.set_node_attribute(point_node, "y", _interest_point.y);
  }

  //! \see XmlImagesInterface::action_at_left_click()
  virtual void action_at_left_click(int x, int y) {
    _interest_point = cv::Point2i(x, y);
  }

  //! \see XmlImagesInterface::refresh_window_custom()
  virtual void refresh_window_custom() {
    // draw a circle at the interest point
    maggieDebug2("Circle at (%i, %i)", _interest_point.x, _interest_point.y);
    cv::circle(_current_image_in_window, _interest_point,
               4, CV_RGB(255, 0, 0), -1);
  }

protected:
  //! a foo attribute, the point of interest
  cv::Point2i _interest_point;
};

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  maggieDebug2("main()");
  XmlImageFooInterface interf;
  //interf.from_xml_file(IMG_DIR "pz/", "pz.xml");

  // Run all the tests that were declared with TEST()
  // srand(time(NULL));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

