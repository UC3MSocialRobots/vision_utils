#ifndef XML_IMAGES_READER_H
#define XML_IMAGES_READER_H

#include "vision_utils/utils/XmlDocument.h"
// std
#include <vector>
// opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
  XmlImagesReader();

  /*! create an annotator from loading an xml file
      \param path_to_xml_file it should finish with /
      Example IMG_DIR "pz/"
      \param xml_filename Example foo.xml
      */
  virtual bool from_xml_file(const std::string & path_to_xml_file,
                             const std::string & xml_filename);

  virtual bool from_xml_content(const std::string & path_to_xml_file,
                                const std::string & xml_content);

  //! destructor
  virtual ~XmlImagesReader();

  //! \return a string showing the content of the current file
  std::string get_current_filenode_to_string() const;

  //! \return the name of the current file
  std::string get_current_filename() const;

  //! \return the current file as OpenCV image
  cv::Mat3b* get_current_cv_img();

  //! jump to the next file and returns a pointer if wanted
  virtual void go_to_next_file();

  //! jump to the previous file and returns a pointer if wanted
  virtual void go_to_previous_file();

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
  inline bool parse_doc();

  //! a pointer to current node
  inline XmlDocument::Node* current_node() const {
    return _file_nodes[_current_file_idx];
  }

  bool load_current_file_from_xml();
  bool save_current_file_to_xml();

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

#endif // XML_IMAGES_READER_H
