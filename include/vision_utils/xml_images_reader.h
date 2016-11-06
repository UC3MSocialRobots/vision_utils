#ifndef XMLIMAGESREADER_H
#define XMLIMAGESREADER_H

#include "vision_utils/XmlDocument.h"
// std
#include <vector>
// opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace vision_utils {

/*! \class  XmlImagesReader
 *
 */
class XmlImagesReader {
public:
  // stuff that should be overloaded
  ////////////////////////////////////////////////////////////////////////////
  //! load from a given xml XmlDocument::Node
  virtual void from_xml_node_custom(const XmlDocument & doc,
                                    XmlDocument::Node* node) = 0;

  //! replace the content of a XmlDocument::Node with the new data
  virtual void to_xml_node_custom(XmlDocument & doc,
                                  XmlDocument::Node* node) const = 0;
  ////////////////////////////////////////////////////////////////////////////

  //! ctor
  XmlImagesReader() {
    _is_current_file_valid = false;
    _current_file_idx  = 0;
  }

  /*! create an annotator from loading an xml file
      \param path_to_xml_file it should finish with /
      Example IMG_DIR() + "pz/"
      \param xml_filename Example foo.xml
      */
  virtual bool from_xml_file(const std::string & path_to_xml_file,
                             const std::string & xml_filename)
  {
    printf("from_xml_file('%s'; '%s')\n",
           path_to_xml_file.c_str(), xml_filename.c_str());

    // store the values
    _path_to_xml_file = path_to_xml_file + std::string("/");
    // save to another xml file with the xml suffix
    std::string xml_filename_no_ext = xml_filename.substr
        (0, xml_filename.find_last_of("."));
    std::ostringstream full_xml_backup_path;
    full_xml_backup_path << _path_to_xml_file
                         << xml_filename_no_ext << "_backup.xml";
    _xml_backup_filename = full_xml_backup_path.str();

    // load the file
    std::ostringstream full_xml_path;
    full_xml_path << _path_to_xml_file << xml_filename;
    bool load_succes = _doc.load_from_file(full_xml_path.str());
    if (!load_succes) {
      printf("Could not load XML file '%s', exiting.\n", full_xml_path.str().c_str());
      _file_nodes.clear(); // clear previous data
      return false;
    }

    return parse_doc();
  }

  virtual bool from_xml_content(const std::string & path_to_xml_file,
                                const std::string & xml_content)
  {
    printf("from_xml_content('%s'; content)\n",
           path_to_xml_file.c_str());

    // store the values
    _path_to_xml_file = path_to_xml_file + std::string("/");
    _xml_backup_filename = _path_to_xml_file + std::string("backup.xml");

    // load the string
    bool load_succes = _doc.load_from_string(xml_content);
    if (!load_succes) {
      printf("Could not parse XML content '%s', exiting.\n", xml_content.c_str());
      _file_nodes.clear(); // clear previous data
      return false;
    }

    return parse_doc();
  }

  //! destructor
  virtual ~XmlImagesReader() {
    //printf("dtor\n");
    if (_file_nodes.size() == 0)
      return; // no backup
    printf("Writing backup file'%s'\n", _xml_backup_filename.c_str());
    _doc.write_to_file(_xml_backup_filename);
  }

  //! \return a string showing the content of the current file
  std::string get_current_filenode_to_string() const {
    return _doc.to_string_node(current_node());
  }

  //! \return the name of the current file
  std::string get_current_filename() const {
    return _current_filename;
  }

  //! \return the current file as OpenCV image
  cv::Mat3b* get_current_cv_img() {
    return &_current_cv_image;
  }

  //! jump to the next file and returns a pointer if wanted
  virtual void go_to_next_file() {
    //printf("go_to_next_file()\n");
    save_current_file_to_xml();
    int nb_iters = 0;
    do { // go one step forward
      ++_current_file_idx;
      ++nb_iters;
      // check if we reached the end
      if (_current_file_idx >= (int) _file_nodes.size())
        _current_file_idx = 0;
      load_current_file_from_xml();
    } while (!is_current_file_valid() && nb_iters <= get_nb_files());
    if (nb_iters > get_nb_files()) {
      printf("It seems no file is readable in XmlFile...\n");
    }
  }

  //! jump to the previous file and returns a pointer if wanted
  virtual void go_to_previous_file() {
    printf("go_to_previous_file()\n");
    save_current_file_to_xml();
    int nb_iters = 0;
    do { // go one step backward
      --_current_file_idx;
      ++nb_iters;
      // if we are at the beginning, jump at the end
      if (_current_file_idx == -1)
        _current_file_idx = (_file_nodes.empty() ? 0 : _file_nodes.size() - 1);
      load_current_file_from_xml();
    } while (!is_current_file_valid() && nb_iters <= get_nb_files());
    if (nb_iters > get_nb_files()) {
      printf("It seems no file is readable in XmlFile...\n");
    }
  }

  inline bool is_current_file_valid() const {
    return _is_current_file_valid;
  }

  //! \return the index of the file in the XML
  inline int get_current_file_index() const {
    return _current_file_idx;
  }

  //! \return the number of files
  inline ssize_t get_nb_files() const {
    return _file_nodes.size();
  }

protected:
  inline bool parse_doc() {
    // get the images nodes
    _file_nodes.clear(); // clear previous data
    _doc.get_all_nodes_at_direction(_doc.root(), "images.image", _file_nodes);
    printf("Found %li files\n", get_nb_files());

    // init the current file
    _current_file_idx  = 0;
    // now load file
    return load_current_file_from_xml();
  }

  //! a pointer to current node
  inline XmlDocument::Node* current_node() const {
    return _file_nodes[_current_file_idx];
  }

  bool load_current_file_from_xml() {
    //printf("load_current_file_from_xml()");
    _is_current_file_valid = false;
    if (get_nb_files() == 0) {
      printf("load_current_file_from_xml():No file in XML, nothing to load.\n");
      create_error_cv_image();
      return true;
    }

    // load the filename
    _current_filename = _doc.get_node_attribute(current_node(), "filename");
    if (_current_filename == "") {
      printf("Error getting filename from node '%s'\n",
             _doc.to_string_node(current_node()).c_str());
      create_error_cv_image();
      return false;
    }

    // call the custom callback
    from_xml_node_custom(_doc, current_node());

    // load the opencv image
    std::ostringstream full_xml_path;
    full_xml_path << _path_to_xml_file << _current_filename;
    //printf("loading image '%s'\n", full_xml_path.str().c_str());
    // load as color to ensure it is a cv::Mat3b
    _current_cv_image = cv::imread(full_xml_path.str(), CV_LOAD_IMAGE_COLOR);
    if (_current_cv_image.empty()) {
      printf("Error reading file '%s'\n", full_xml_path.str().c_str());
      create_error_cv_image();
      return false;
    }
    _is_current_file_valid = true;
    return true;
  }
  bool save_current_file_to_xml() {
    if (!is_current_file_valid()) {
      printf("save_current_file_to_xml(): file not valid, skipping writing.\n");
      return false;
    }

    //printf("save_current_file_to_xml()");
    current_node()->remove_all_nodes();
    current_node()->remove_all_attributes();
    // save the filename
    _doc.set_node_attribute(current_node(), "filename", _current_filename);
    to_xml_node_custom(_doc, current_node());
    return true;
  }

  //! create an OpenCV error image when file not loaded succesfully
  void create_error_cv_image() {
    int side = 200;
    _current_cv_image.create(side, side);
    _current_cv_image.setTo(255);
    cv::line(_current_cv_image, cv::Point(0, 0), cv::Point(side, side),
             CV_RGB(255, 0, 0), 10);
    cv::line(_current_cv_image, cv::Point(0, side), cv::Point(side, 0),
             CV_RGB(255, 0, 0), 10);
  }

  //////////////////////////////////////////////////////////////////////////////

  XmlDocument _doc;
  //! where the xml file is found. Also the root of the files into it
  std::string _path_to_xml_file;
  //! the name of the xml filename
  std::string _xml_backup_filename;

  typedef std::vector<XmlDocument::Node*> NodeVector;
  NodeVector _file_nodes;
  int _current_file_idx;
  bool _is_current_file_valid;

  //! the opencv image corresponding to the current iterator
  cv::Mat3b _current_cv_image;

  std::string _current_filename;
};

////////////////////////////////////////////////////////////////////////////////

inline void update_timer_ntimes(double & total_time,
                                const int n_occurences,
                                const double & new_time) {
  total_time = (1.f * n_occurences * total_time + new_time) / (n_occurences + 1);
}

} // end namespace vision_utils

#endif // XMLIMAGESREADER_H
