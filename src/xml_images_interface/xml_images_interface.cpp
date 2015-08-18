#include "vision_utils/xml_images_interface.h"

///////// implementations /////////////////////////////////////////////

bool XmlImagesInterface::from_xml_file
(const std::string & path_to_xml_file,
 const std::string & xml_filename,
 bool want_key_listening /*= true*/)
{
  if (!XmlImagesReader::from_xml_file(path_to_xml_file, xml_filename))
    return false;
  _window_name = xml_filename;
  init_cb();
  if (want_key_listening)
    key_listening();
  return true;
}

///////////////////////////////////////////////////////////////////////

void XmlImagesInterface::go_to_next_file(bool want_key_listening /*= true*/) {
  XmlImagesReader::go_to_next_file();
  refresh_window();
  if (want_key_listening)
    key_listening();
}

///////////////////////////////////////////////////////////////////////

void XmlImagesInterface::go_to_previous_file(bool want_key_listening /*= true*/) {
  XmlImagesReader::go_to_previous_file();
  refresh_window();
  if (want_key_listening)
    key_listening();
}

///////////////////////////////////////////////////////////////////////

void XmlImagesInterface::key_listening() {
  while (true) {
    char c = cv::waitKey(0);
    int key_code = (char) c;
    maggieDebug2("key_code:%i", key_code);
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

///////////////////////////////////////////////////////////////////////

void XmlImagesInterface::refresh_window() {
  get_current_cv_img()->copyTo(_current_image_in_window);

  // call function
  refresh_window_custom();

  // refresh content
  cv::imshow(_window_name, _current_image_in_window);
}

///////////////////////////////////////////////////////////////////////

void XmlImagesInterface::mouse_callback(int event, int x, int y, int, void* param) {
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

