#ifndef IO_H
#define IO_H

#include "opencv2/highgui/highgui.hpp"
#include <system/system_utils.h>
#include <debug/error.h>
#include "compressed_rounded_image_transport/cv_conversion_float_uchar.h"
#include <kinect_utils/user_image_to_rgb.h>
#include "convert_n_colors.h"

namespace image_utils {

/*!
 *\brief   returns a string with information about the image
 *(size, depth...)
 */
inline std::string infosImage(const cv::Mat & i) {
  std::ostringstream concl;
  concl << "size:(" << i.cols << "x" << i.rows << "), ";
  concl << i.channels() << " channels";
  concl << ", depth:" << i.depth();
  concl << ", type:" << i.type();
  concl << ", isContinuous:" << i.isContinuous();
  return concl.str();
}

////////////////////////////////////////////////////////////////////////////////

//#define FILENAME "image_utils_temp.yml"

/*!
 *cast a matrix to a string
 *\param mat
 *\param s its string representation
 */
inline void to_string(const cv::Mat & mat, std::string & s) {
  maggieDebug3("to_string()");
  if (mat.isContinuous() == false)
    maggieError("Cannot convert an image that is not continuous !");
  size_t data_length = mat.cols * mat.rows * mat.elemSize();
  std::ostringstream s_stream;
  s_stream << mat.cols << ' ' << mat.rows << ' ' << mat.type() << ' '
           << data_length << ' ';
  s_stream << std::string((char*) mat.data, data_length);
  s = s_stream.str();
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *populate a matrix with a string
 *\param mat
 *\param s
 */
inline void from_string(cv::Mat & mat, const std::string & s) {
  maggieDebug3("from_string()");
  /*
     * with short stream 1
     */
  std::istringstream s_stream(s);
  int cols, rows, type;
  size_t data_length;
  s_stream >> cols;
  s_stream >> rows;
  s_stream >> type;
  s_stream >> data_length;
  maggieDebug3("Creating an image of size %i rows x %i cols, type:%i.", rows, cols, type);
  if (rows > 3000 || cols > 3000 || rows <= 0 || cols <= 0)
    maggieError("Soemthing is wrong with those dimensions %ix%i.",
                rows, cols);
  mat.create(rows, cols, type);
  if (mat.isContinuous() == false)
    maggieError("Cannot convert to an image that is not continuous !");

  // get everything left in the stream
  int data_stream_begin_position = 1 + (int) s_stream.tellg();
  std::string::const_iterator s_ptr = s.begin();
  // go to the wanted position
  for (int var = 0; var < data_stream_begin_position; ++var)
    ++s_ptr;
  // read the data
  maggieDebug3("Ready to read %i bytes.", data_length);
  memcpy(mat.data, &s.at(data_stream_begin_position), data_length);
}

////////////////////////////////////////////////////////////////////////////////

inline void compress_jpeg(const cv::Mat3b & input, const int factor,
                          std::string & ans) {
  std::vector<int> params;
  params.push_back(CV_IMWRITE_JPEG_QUALITY);
  params.push_back(factor);
  std::vector<uchar> buf;
  bool encode_OK = cv::imencode(".jpg", input, buf, params);
  if (!encode_OK)
    maggiePrint("Encoding failed !");
  ans = std::string(buf.begin(), buf.end());
}

////////////////////////////////////////////////////////////////////////////////

inline void uncompress_jpeg(cv::Mat3b & output, const std::string & ans) {
  std::vector<uchar> ans_vector (ans.begin(), ans.end());
  output = cv::imdecode(cv::Mat (ans_vector), -1);
  maggieDebug2("output:%s", image_utils::infosImage(output).c_str());
}

////////////////////////////////////////////////////////////////////////////////

inline void write_rgb_and_depth_to_yaml_file(const std::string & yaml_filename_prefix,
                                             const cv::Mat & rgb, const cv::Mat & depth,
                                             bool debug_info = true) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_rgb_depth.yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::WRITE);
  fs << "rgb" << rgb;
  fs << "depth" << depth;
  fs.release();
  if (debug_info)
    printf("Succesfully written depth and rgb in '%s'\n",
           full_filename.str().c_str());
} // end write_rgb_and_depth_to_yaml_file();

////////////////////////////////////////////////////////////////////////////////

inline bool read_depth_rgb_from_yaml_file(const std::string & yaml_filename_prefix,
                                          cv::Mat & rgb, cv::Mat & depth) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_rgb_depth.yaml";
  if (!system_utils::file_exists(full_filename.str())) {
    printf("File '%s' does not exist!", full_filename.str().c_str());
    return false;
  }
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::READ);
  fs["rgb"] >> rgb;
  fs["depth"] >> depth;
  fs.release();
  if (rgb.empty() || depth.empty()) {
    printf("read_depth_rgb_from_yaml_file('%s'): "
           "depth(%s) or RGB(%s) images are empty!",
           full_filename.str().c_str(),
           image_utils::infosImage(depth).c_str(),
           image_utils::infosImage(rgb).c_str());
    return false;
  }
  return true;
} // end read_depth_rgb_from_yaml_file();

////////////////////////////////////////////////////////////////////////////////

bool read_rgb_depth_user_image_from_image_file(const std::string & filename_prefix,
                                               cv::Mat* rgb = NULL,
                                               cv::Mat* depth = NULL,
                                               cv::Mat1b* user_mask = NULL,
                                               FileFormat format = FILE_PNG) {
  // read depth and rgb
  if (!image_utils::read_rgb_and_depth_image_from_image_file
      (filename_prefix, rgb, depth, format))
    return false;
  // read user mask
  if (user_mask == NULL)
    return true;
  std::ostringstream user_mask_filename;
  user_mask_filename << filename_prefix << "_user_mask"  << format2extension(format);
  if (!file_exists(user_mask_filename.str())) {
    printf("user mask file '%s' does not exist, cannot read it!\n",
           user_mask_filename.str().c_str());
    return false;
  }
  *user_mask = cv::imread(user_mask_filename.str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (user_mask->empty()) {
    printf("user mask file '%s' is corrupted!\n", user_mask_filename.str().c_str());
    return false;
  }
  return true;
  // printf("Read user mask file '%s'.\n", user_mask_filename.str().c_str());
}

////////////////////////////////////////////////////////////////////////////////

inline bool write_rgb_depth_user_to_image_file
(const std::string & filename_prefix,
 const cv::Mat3b * rgb = NULL,
 const cv::Mat1f * depth = NULL,
 const cv::Mat1b * user = NULL,
 cv::Mat3b * user_illus = NULL,
 FileFormat format = FILE_PNG,
 bool debug_info = true)
{
  if (!write_rgb_and_depth_image_to_image_file
      (filename_prefix, rgb, depth, format, debug_info))
    return false;

  if((user != NULL && user_illus == NULL)
     || (user == NULL && user_illus != NULL)) {
    printf("write_rgb_depth_user_to_image_file(): user and user_illus "
           "should both be either NULL or non NULL\n");
    return false;
  }
  std::string extension = format2extension(format);
  ParamsVec params = format2params(format);
  std::ostringstream user_mask_filename;
  user_mask_filename << filename_prefix << "_user_mask" << extension;
  if (!cv::imwrite(user_mask_filename.str(), *user, params)) {
    printf("write_user_mask_and_depth_image_as_uchar_to_image_file(): "
           "could not write user_mask image '%s'\n", user_mask_filename.str().c_str());
    return false;
  }
  if (debug_info)
    printf("Written user file '%s'.\n", user_mask_filename.str().c_str());

  user_image_to_rgb(*user, *user_illus, 8);
  std::ostringstream user_mask_illus_filename;
  user_mask_illus_filename << filename_prefix << "_user_mask_illus" << extension;
  if (!cv::imwrite(user_mask_illus_filename.str(), *user_illus, params)) {
    printf("write_user_mask_illus_and_depth_image_as_uchar_to_image_file(): "
           "could not write user_mask_illus image '%s'\n", user_mask_illus_filename.str().c_str());
    return false;
  }
  if (!convert_n_colors(user_mask_illus_filename.str(), 256,
                        user_mask_illus_filename.str()))
    return false;
  if (debug_info)
    printf("Written user illus file '%s'.\n", user_mask_illus_filename.str().c_str());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *build a string to represent a rectangle
 *\param rect
 *\return something like (x, y)+(w, h)
 */
inline std::string rectangle_to_string(const cv::Rect & rect) {
  std::ostringstream concl;
  concl << "(" << rect.x << ", " << rect.y << ")";
  concl << "+(" << rect.width << ", " << rect.height << ")";
  return concl.str();
}

////////////////////////////////////////////////////////////////////////////

template<class _T>
static inline void print_random_pts_float(const cv::Mat & img, const int & nb_pts) {
  maggieDebug2("print_random_pts(nb_pts:%i)", nb_pts);
  for (int pt_idx = 0; pt_idx < nb_pts; ++pt_idx) {
    int col = rand() % img.cols;
    int row = rand() % img.rows;
    maggiePrint("row:%i, col:%i, val:%g", row, col, img.at<_T>(col, row));
  } // end loop pt_idx
} // end print_random_pts

////////////////////////////////////////////////////////////////////////////////

template<class _T>
static inline void print_random_pts_int(const cv::Mat & img, const int & nb_pts) {
  maggieDebug2("print_random_pts(nb_pts:%i)", nb_pts);
  for (int pt_idx = 0; pt_idx < nb_pts; ++pt_idx) {
    int col = rand() % img.cols;
    int row = rand() % img.rows;
    maggiePrint("row:%i, col:%i, val:%i", row, col, (int) img.at<_T>(col, row));
  } // end loop pt_idx
} // end print_random_pts

////////////////////////////////////////////////////////////////////////////////

/*!
 * Save file to YAML / XML
 * \param yaml_filename_prefix
 */
template<class _T>
inline void to_yaml(_T & obj,
                    const std::string & yaml_filename_prefix,
                    const std::string & key) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_" << key << ".yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::WRITE);
  obj.write(fs);
  fs.release();
  printf("Succesfully written '%s' to file '%s'\n",
         key.c_str(), full_filename.str().c_str());
} // end to_yaml();

////////////////////////////////////////////////////////////////////////////////

/*!
 * Load from YAML / XML file
 * \param yaml_filename_prefix
 */
template<class _T>
inline void from_yaml(_T & obj,
                      const std::string & yaml_filename_prefix,
                      const std::string & key) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_" << key << ".yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::READ);
  read(fs.root(), obj, _T());
  fs.release();
  printf("Succesfully read '%s' from file '%s'\n",
         key.c_str(), full_filename.str().c_str());
} // end from_yaml();

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline void write(const std::vector<_T> & phs,
                  cv::FileStorage &fs,
                  const std::string & key) {
  fs << key << "[";
  for( unsigned int i = 0; i < phs.size(); i++ ) {
    printf("write key '%s' #%i\n", key.c_str(), i);
    fs << "{";
    //fs << phs[i];
    phs[i].write(fs);
    fs << "}";
  }
  fs << "]";
} // end write()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Save file to YAML / XML
 * \param yaml_filename_prefix
 */
template<class _T>
inline void to_yaml_vector(const std::vector<_T> & phs,
                           const std::string & yaml_filename_prefix,
                           const std::string & key) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_" << key << ".yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::WRITE);
  write(phs, fs, key);
  fs.release();
  printf("Succesfully written vector<_T> to file '%s'\n", full_filename.str().c_str());
} // end to_yaml_vector();

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline void read(std::vector<_T> & phs,
                 const cv::FileNode &fn,
                 const std::string & key) {
  cv::FileNode hist_nodes = fn[key];
  phs.resize(hist_nodes.size());
  for (unsigned int hist_idx = 0; hist_idx < hist_nodes.size(); ++hist_idx)
    hist_nodes[hist_idx] >> phs[hist_idx];
} // end write

////////////////////////////////////////////////////////////////////////////////

/*!
 * Load from YAML / XML file
 * \param yaml_filename_prefix
 */
template<class _T>
inline void from_yaml_vector(std::vector<_T> & obj,
                             const std::string & yaml_filename_prefix,
                             const std::string & key) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_" << key << ".yaml";
  cv::FileStorage fs(full_filename.str(), cv::FileStorage::READ);
  //read(obj, fs.getFirstTopLevelNode());
  fs[key] >> obj;
  fs.release();
  printf("Succesfully read vector<_T> from file '%s'\n",
         full_filename.str().c_str());
} // end from_yaml_vector();

////////////////////////////////////////////////////////////////////////////////

enum NbColors {
  COLOR_24BITS = 0,
  COLORS256 = 1,
  MONOCHROME = 2
};

inline bool imwrite_debug(const std::string& filename, cv::InputArray img,
                          NbColors ncolors = COLOR_24BITS,
                          const std::vector<int>& params=std::vector<int>()) {
  if (!cv::imwrite(filename, img, params)) {
    printf("/!\\ Could not write file '%s'\n", filename.c_str());
    return false;
  }
  // color reduction
  if (ncolors == COLORS256 && !convert_n_colors(filename, 256, filename)) {
    printf("/!\\ Could not reduce file '%s' to 256 colors \n", filename.c_str());
    return false;
  }
  else if (ncolors == MONOCHROME && !reduce_monochrome(filename, filename)) {
    printf("/!\\ Could not reduce file '%s' to monochrome \n", filename.c_str());
    return false;
  }
  printf("Succesfully written file '%s'\n", filename.c_str());
  return true;
} // end imwrite_debug()

} // end namespace image_utils

////////////////////////////////////////////////////////////////////////////////

/*!
 * It’s possible to serialize this through the OpenCV I/O XML/YAML interface
 *(just as in case of the OpenCV data structures) by adding a read and
 * a write function inside and outside of your class. Cf:
 * http://docs.opencv.org/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html
 * \param fs
 *  the stream
 * \param x
 *  the value to serialize
 */
template<class _T>
void write(cv::FileStorage & fs, const std::string &, const _T& x) {
  x.write(fs);
} // end write()

////////////////////////////////////////////////////////////////////////////////

/*!
 * It’s possible to serialize this through the OpenCV I/O XML/YAML interface
 *(just as in case of the OpenCV data structures) by adding a read and
 * a write function inside and outside of your class. Cf:
 * http://docs.opencv.org/doc/tutorials/core/file_input_output_with_xml_yml/file_input_output_with_xml_yml.html
 * \param node
 * \param x
 * \param default_value
 *
 */
template<class _T>
void read(const cv::FileNode & node, _T & x, const _T& default_value = _T()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
} //end read()

#endif // IO_H
