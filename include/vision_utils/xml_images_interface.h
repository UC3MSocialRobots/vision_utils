#ifndef XML_IMAGES_INTERFACE_H
#define XML_IMAGES_INTERFACE_H

#include "vision_utils/xml_images_reader.h"

/*! \class  XmlImagesInterface
 *
 */
class XmlImagesInterface : public XmlImagesReader {
public:
  /*! create an annotator from loading an xml file
      \param path_to_xml_file it should finish with /
      Example LONG_TERM_MEMORY_DIR "images/pz/"
      \param xml_filename Example foo.xml
      */
  virtual bool from_xml_file(const std::string & path_to_xml_file,
                             const std::string & xml_filename,
                             bool want_key_listening = true);

  //! jump to the next file and returns a pointer if wanted
  void go_to_next_file(bool want_key_listening = true);

  //! jump to the previous file and returns a pointer if wanted
  void go_to_previous_file(bool want_key_listening= true);

  /*! behaviour when showing a new picture.
   *  Should modify _current_image_in_window .
    Does nothing by default.
    Can be overloaded.
        */
  virtual void refresh_window_custom() {
    maggieDebug2("refresh_window_custom()");
    // do nothing
  }

  /*! behaviour when left click.
        Does nothing by default
        Can be overloaded.
        */
  virtual void action_at_left_click(int x, int y) {
    maggieDebug2("action_at_left_click(%i, %i)", x, y);
    // do nothing
  }

  /*! behaviour when right click.
        Does nothing by default
        Can be overloaded.
        */
  virtual void action_at_right_click(int x, int y)  {
    maggieDebug2("action_at_right_click(%i, %i)", x, y);
    // do nothing
  }

  //! behaviour for when space is pressed. Can be overloaded.
  virtual void action_at_key_pressed(char c) {
    maggieDebug2("action_at_key_pressed(%c)", c);
    // do nothing
  }


  //! behaviour for when space is pressed. Can be overloaded.
  virtual void action_at_space_pressed() {
    maggieDebug2("action_at_space_pressed()");
    go_to_next_file();
  }

  //! behaviour for when backspace is pressed. Can be overloaded.
  virtual void action_at_backspace_pressed() {
    maggieDebug2("action_at_backspace_pressed()");
    go_to_previous_file();
  }

  //! refresh the display
  void refresh_window();

protected:
  cv::Mat3b _current_image_in_window;

  inline void init_cb() {
    // create window
    cvNamedWindow(_window_name.c_str());
    // mouse callback
    cvSetMouseCallback(_window_name.c_str(), mouse_callback, this);
    refresh_window();
  }

  void key_listening();

  static void mouse_callback(int event, int x, int y, int flags, void* param);

  std::string _window_name;
};

#endif // XML_IMAGES_INTERFACE_H
