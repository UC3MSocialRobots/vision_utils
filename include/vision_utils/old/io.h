#ifndef IO_H
#define IO_H

#include "opencv2/highgui/highgui.hpp"
#include "vision_utils/convert_n_colors.h"

#include "vision_utils/infosimage.h"

#include "vision_utils/user_image_to_rgb.h"

namespace vision_utils {

////////////////////////////////////////////////////////////////////////////////
//cut:to_string
//#define FILENAME "image_utils_temp.yml"

/*!
 *cast a matrix to a string
 *\param mat
 *\param s its string representation
 */
inline void to_string(const cv::Mat & mat, std::string & s) {
  //printf("to_string()");
  if (mat.isContinuous() == false)
    throw std::invalid_argument("Cannot convert an image that is not continuous !");
  size_t data_length = mat.cols * mat.rows * mat.elemSize();
  std::ostringstream s_stream;
  s_stream << mat.cols << ' ' << mat.rows << ' ' << mat.type() << ' '
           << data_length << ' ';
  s_stream << std::string((char*) mat.data, data_length);
  s = s_stream.str();
}

////////////////////////////////////////////////////////////////////////////////
//cut:from_string
/*!
 *populate a matrix with a string
 *\param mat
 *\param s
 */
inline void from_string(cv::Mat & mat, const std::string & s) {
  //printf("from_string()");
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
  //printf("Creating an image of size %i rows x %i cols, type:%i.", rows, cols, type);
  if (rows > 3000 || cols > 3000 || rows <= 0 || cols <= 0) {
    char error_msg[1000];
    sprintf(error_msg, "Soemthing is wrong with those dimensions %ix%i.", rows, cols);
    throw std::invalid_argument(error_msg);
  }
  mat.create(rows, cols, type);
  if (mat.isContinuous() == false)
    throw std::invalid_argument("Cannot convert to an image that is not continuous !");

  // get everything left in the stream
  int data_stream_begin_position = 1 + (int) s_stream.tellg();
  std::string::const_iterator s_ptr = s.begin();
  // go to the wanted position
  for (int var = 0; var < data_stream_begin_position; ++var)
    ++s_ptr;
  // read the data
  //printf("Ready to read %i bytes.", data_length);
  memcpy(mat.data, &s.at(data_stream_begin_position), data_length);
}

////////////////////////////////////////////////////////////////////////////////
//cut:compress_jpeg

inline void compress_jpeg(const cv::Mat3b & input, const int factor,
                          std::string & ans) {
  std::vector<int> params;
  params.push_back(CV_IMWRITE_JPEG_QUALITY);
  params.push_back(factor);
  std::vector<uchar> buf;
  bool encode_OK = cv::imencode(".jpg", input, buf, params);
  if (!encode_OK)
    printf("Encoding failed !");
  ans = std::string(buf.begin(), buf.end());
}

////////////////////////////////////////////////////////////////////////////////
//cut:uncompress_jpeg

inline void uncompress_jpeg(cv::Mat3b & output, const std::string & ans) {
  std::vector<uchar> ans_vector (ans.begin(), ans.end());
  output = cv::imdecode(cv::Mat (ans_vector), -1);
  //printf("output:%s", infosImage(output).c_str());
}

////////////////////////////////////////////////////////////////////////////////
//cut:write_rgb_and_depth_to_yaml_file

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
//cut:read_depth_rgb_from_yaml_file

inline bool read_depth_rgb_from_yaml_file(const std::string & yaml_filename_prefix,
                                          cv::Mat & rgb, cv::Mat & depth) {
  std::ostringstream full_filename;
  full_filename << yaml_filename_prefix << "_rgb_depth.yaml";
  if (!file_exists(full_filename.str())) {
    printf("File '%s' does not exist!\n", full_filename.str().c_str());
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
           infosImage(depth).c_str(),
           infosImage(rgb).c_str());
    return false;
  }
  return true;
} // end read_depth_rgb_from_yaml_file();

////////////////////////////////////////////////////////////////////////////////
//cut:read_rgb_depth_user_image_from_image_file

bool read_rgb_depth_user_image_from_image_file(const std::string & filename_prefix,
                                               cv::Mat* rgb = NULL,
                                               cv::Mat* depth = NULL,
                                               cv::Mat1b* user_mask = NULL,
                                               FileFormat format = FILE_PNG) {
  // read depth and rgb
  if (!read_rgb_and_depth_image_from_image_file
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
  //printf("Read user mask file '%s'.\n", user_mask_filename.str().c_str());
}

////////////////////////////////////////////////////////////////////////////////
//cut:write_rgb_depth_user_to_image_file

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
//cut:rectangle_to_string

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
//cut:print_random_pts_float

template<class _T>
static inline void print_random_pts_float(const cv::Mat & img, const int & nb_pts) {
  //printf("print_random_pts(nb_pts:%i)", nb_pts);
  for (int pt_idx = 0; pt_idx < nb_pts; ++pt_idx) {
    int col = rand() % img.cols;
    int row = rand() % img.rows;
    printf("row:%i, col:%i, val:%g\n", row, col, img.at<_T>(col, row));
  } // end loop pt_idx
} // end print_random_pts

////////////////////////////////////////////////////////////////////////////////
//cut:print_random_pts_int

template<class _T>
static inline void print_random_pts_int(const cv::Mat & img, const int & nb_pts) {
  //printf("print_random_pts(nb_pts:%i)", nb_pts);
  for (int pt_idx = 0; pt_idx < nb_pts; ++pt_idx) {
    int col = rand() % img.cols;
    int row = rand() % img.rows;
    printf("row:%i, col:%i, val:%i\n", row, col, (int) img.at<_T>(col, row));
  } // end loop pt_idx
} // end print_random_pts

////////////////////////////////////////////////////////////////////////////////
//cut:to_yaml

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
//cut:from_yaml

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
//cut:write_vec

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
//cut:to_yaml_vector

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
//cut:read_vec

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
//cut:to_yaml_vector

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
//cut:imwrite_debug

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

} // end namespace vision_utils

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

//cut
#endif // IO_H
