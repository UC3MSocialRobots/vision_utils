#ifndef XMLIMAGESINTERFACE_H
#define XMLIMAGESINTERFACE_H

#include "vision_utils/xml_images_reader.h"

namespace vision_utils {

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
                             bool want_key_listening = true)
  {
    if (!XmlImagesReader::from_xml_file(path_to_xml_file, xml_filename))
      return false;
    _window_name = xml_filename;
    init_cb();
    if (want_key_listening)
      key_listening();
    return true;
  }

  //! jump to the next file and returns a pointer if wanted
  void go_to_next_file(bool want_key_listening = true) {
    XmlImagesReader::go_to_next_file();
    refresh_window();
    if (want_key_listening)
      key_listening();
  }

  //! jump to the previous file and returns a pointer if wanted
  void go_to_previous_file(bool want_key_listening= true) {
    XmlImagesReader::go_to_previous_file();
    refresh_window();
    if (want_key_listening)
      key_listening();
  }

  /*! behaviour when showing a new picture.
   *  Should modify _current_image_in_window .
    Does nothing by default.
    Can be overloaded.
        */
  virtual void refresh_window_custom() {
    //printf("refresh_window_custom()");
    // do nothing
  }

  /*! behaviour when left click.
        Does nothing by default
        Can be overloaded.
        */
  virtual void action_at_left_click(int /*x*/, int /*y*/) {
    //printf("action_at_left_click(%i, %i)", x, y);
    // do nothing
  }

  /*! behaviour when right click.
        Does nothing by default
        Can be overloaded.
        */
  virtual void action_at_right_click(int /*x*/, int /*y*/)  {
    //printf("action_at_right_click(%i, %i)", x, y);
    // do nothing
  }

  //! behaviour for when space is pressed. Can be overloaded.
  virtual void action_at_key_pressed(char /*c*/) {
    //printf("action_at_key_pressed(%c)", c);
    // do nothing
  }


  //! behaviour for when space is pressed. Can be overloaded.
  virtual void action_at_space_pressed() {
    //printf("action_at_space_pressed()");
    go_to_next_file();
  }

  //! behaviour for when backspace is pressed. Can be overloaded.
  virtual void action_at_backspace_pressed() {
    //printf("action_at_backspace_pressed()");
    go_to_previous_file();
  }

  //! refresh the display
  void refresh_window() {
    get_current_cv_img()->copyTo(_current_image_in_window);

    // call function
    refresh_window_custom();

    // refresh content
    cv::imshow(_window_name, _current_image_in_window);
  }

protected:
  cv::Mat3b _current_image_in_window;

  inline void init_cb() {
    // create window
    cvNamedWindow(_window_name.c_str());
    // mouse callback
    cvSetMouseCallback(_window_name.c_str(), mouse_callback, this);
    refresh_window();
  }

  void key_listening() {
    while (true) {
      char c = cv::waitKey(0);
      int key_code = (char) c;
      printf("key_code:%i\n", key_code);
      if (key_code == 27) {
        break;
      }
      else if (key_code == 83 || key_code == 32) {
        action_at_space_pressed();
        break;
      }
      else if (key_code == 81 || key_code == 8) {
        action_at_backspace_pressed();
        break;
      }
      else
        action_at_key_pressed(c);
    } // end key listening
  }

  static void mouse_callback(int event, int x, int y, int flags, void* param) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      XmlImagesInterface* this_p = (XmlImagesInterface*) param;
      this_p->action_at_left_click(x, y);
      // reload the image
      this_p->refresh_window();
    }
    else if (event == CV_EVENT_RBUTTONDOWN) {
      XmlImagesInterface* this_p = (XmlImagesInterface*) param;
      this_p->action_at_right_click(x, y);
      // reload the image
      this_p->refresh_window();
    }
  }

  std::string _window_name;
};

} // end namespace vision_utils

#endif //XMLIMAGESINTERFACE_H
