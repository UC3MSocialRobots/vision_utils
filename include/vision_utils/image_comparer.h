#ifndef IMAGE_COMPARER_H
#define IMAGE_COMPARER_H

/***************************************************************************//**
 * \class ImageComparer
 *
 * \brief an object to find the closest object from an unknown object
 * among a list of models
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date April 2009
 *******************************************************************************/

////// my imports
#include "vision_utils/utils/hausdorff_distances.h"
#include "vision_utils/utils/debug_utils.h"
#include "vision_utils/utils/file_io.h"
#include "vision_utils/utils/filename_handling.h"
#include "vision_utils/content_processing.h"

///// STL imports
#include <opencv2/core/core.hpp>
#include <sstream>          // for std::sstreams

template<class Pt2Iterable>
class ImageComparer_ {
public:
  //typedef std::vector<cv::Point> Pt2Iterable;
  //typedef std::deque<cv::Point> Pt2Iterable;

  ImageComparer_() : _default_size(32, 32){
  }

  //////////////////////////////////////////////////////////////////////////////

  ~ImageComparer_() {}

  //////////////////////////////////////////////////////////////////////////////

  void set_models(const std::string & index_filename,
                  const cv::Size & default_size,
                  bool keep_ratio = false) {
    maggieDebug2("set_models('%s')", index_filename.c_str());
    std::vector<std::string> lines, lines_cleaned;
    string_utils::retrieve_file_split(index_filename, lines);
    // add path to each line
    std::string folder = string_utils::extract_folder_from_full_path(index_filename);
    for (unsigned int line_idx = 0; line_idx < lines.size(); ++line_idx) {
      std::string line (lines[line_idx]);
      if (line.length() == 0)
        continue; // empty line
      if (line.size() >= 2 && line.substr(0, 2) == "//")
        continue; // comment
      std::ostringstream full_path;
      full_path << folder << line;
      lines_cleaned.push_back(full_path.str());
    }
    set_models(lines_cleaned, default_size, keep_ratio);
  }

  //////////////////////////////////////////////////////////////////////////////

  void set_models(const std::vector<std::string> & filenames,
                  const cv::Size & default_size,
                  bool keep_ratio = false) {
    maggieDebug2("set_models(%i files)", filenames.size());
    std::vector<cv::Mat> imgs;
    unsigned int nimgs = filenames.size();
    for (unsigned int img_idx = 0; img_idx < nimgs; ++img_idx) {
      cv::Mat1b curr_img = cv::imread(filenames[img_idx].c_str(), CV_LOAD_IMAGE_GRAYSCALE);
      if (curr_img.empty()) {
        printf("set_models(): cant read file '%s'\n", filenames[img_idx].c_str());
        continue;
      }
      imgs.push_back(curr_img);
    } // end loop img_idx
    set_models(imgs, default_size, keep_ratio);
  } // end set_models();

  //////////////////////////////////////////////////////////////////////////////

  void set_models(const std::vector<cv::Mat> & imgs,
                  const cv::Size & default_size,
                  bool keep_ratio = false) {
    _default_size = default_size;
    _resized_array.create(_default_size.height, _default_size.width);
    _keep_ratio = keep_ratio;

    unsigned int nimgs = imgs.size();
    models_points.clear();
    models_points.reserve(nimgs);
    Pt2Iterable curr_pts, curr_pts_resized;
    for (unsigned int img_idx = 0; img_idx < nimgs; ++img_idx) {
      if (imgs[img_idx].empty()) {
        printf("set_modesl(): model #%i is empty\n", img_idx);
        continue;
      }
      /* add the points */
      // get the non nul points
      image_utils::nonNulPoints(imgs[img_idx], curr_pts);
      if (curr_pts.size() == 0) {
        printf("set_modesl(): model #%i does not contain null points!\n", img_idx);
        continue;
      }
      // redim the content
      image_utils::redimContent_vector_without_repetition_given_resized_array
          (curr_pts, curr_pts_resized,
           cv::Rect(0, 0, _default_size.width, _default_size.height),
           _resized_array.data, _keep_ratio);
      models_points.push_back(curr_pts_resized);
    } // end loop img_idx
  } // end set_models();

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int get_models_nb() const {
    return models_points.size();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! get the default size of the loaded images
  inline cv::Size getDefaultSize() const {
    return _default_size;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * \brief   forbid to compare some model files
   * \param   newForbidden an array of boolean, 0 = allowed, 1 = forbidden
   */
  inline void setMask(const std::vector<bool> & newForbidden) {
    maggieDebug2("setMask()");
    _forbidden_images = newForbidden;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! compare a file with all the others
  bool compareFile(const cv::Mat1b & unknown) {
    // maggieDebug2("compareFile()");
    if (get_models_nb() == 0) {
      printf("compareFile(): no model loaded\n");
      return false;
    }
#if 0
    unknown_image.create(_default_size);
    bool ok = image_utils::redimContent(unknown, unknown_image);
    if (!ok)
      return false;
    printf("unknown_image:'%s'\n", image_utils::img_to_string(unknown_image).c_str());
    _unknown_point_vec.clear();
    image_utils::nonNulPoints(unknown_image, _unknown_point_vec);
    return compare_internal_data();
#else
    image_utils::nonNulPoints(unknown, _unknown_point_vec_unscaled);
    //printf("_unknown_point_vec_unscaled:'%s'\n", image_utils::img_to_string(_unknown_point_vec_unscaled).c_str());
    return compareVector(_unknown_point_vec_unscaled);
#endif
  }

  //////////////////////////////////////////////////////////////////////////////

  //! compare a vector of white points with the preloaded data
  bool compareVector(const Pt2Iterable & unknown) {
    // maggieDebug2("compareVector()");
    if (get_models_nb() == 0) {
      printf("compareVector(): no model loaded\n");
      return false;
    }
    if (unknown.size() == 0) {
      printf("compareVector(): empty unknown Pt2Iterable\n");
      return false;
    }
    image_utils::redimContent_vector_without_repetition_given_resized_array
        (unknown, _unknown_point_vec,
         cv::Rect(0, 0, _default_size.width, _default_size.height),
         _resized_array.data, _keep_ratio);
    return compare_internal_data();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \return the filename of the best match model
  inline int get_best_index() const {
    return _best_index;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! \return the error of the best match model
   *  It is equal to 0 in the best case
   *  (that is, if the query was strictly equal to one of the models)
   */
  inline float getBestResult() const {
    return _best_result;
  }

  /*! the Hausdorff distance between the skeleton of the user
   *  and the reference skeleton is multiplied by this factor.
   *  The final likelihood is exp(- CONFIDENCE_SCALE_FACTOR * hausdorf_dist).
   *  Set to 1 for no scaling.
   */
  inline double get_confidence(const double CONFIDENCE_SCALE_FACTOR = 10) const {
    double hausdorff_dist_norm = getBestResult() /
                                 std::max(getDefaultSize().width, getDefaultSize().height);
    hausdorff_dist_norm *= CONFIDENCE_SCALE_FACTOR; // experimental factor for stretching results
    double ans = std::exp(-hausdorff_dist_norm);
    return (ans < 0 ? 0 : (ans > 1 ? 1 : ans)); // clamp in [0..1]
  }

  inline std::string model_to_string(unsigned int model_idx) const {
    if (model_idx >= get_models_nb())
      return "<bad model_idx>";
    return image_utils::point_vec_to_string(models_points[model_idx], _default_size);
  }

  inline std::string unknown_to_string() const {
    return image_utils::point_vec_to_string(_unknown_point_vec, _default_size);
  }

  inline std::string results_to_string() const {
    return string_utils::iterable_to_string(_results);
  }

private:
  //////////////////////////////////////////////////////////////////////////////

  //! compare _unknown_point_vec with the preloaded model data
  bool compare_internal_data() {
    //    maggieDebug2("compare_internal_data(), nb of points in the unknown object:%i",
    //                 (int) _unknown_point_vec.size());
    // printf("unknown_to_string():'%s'\n", unknown_to_string().c_str());
    _best_result = std::numeric_limits<float>::max();
    _best_index = -1;
    bool using_mask = (_forbidden_images.size() == models_points.size());
    unsigned int nmodels = models_points.size();
    _results.resize(nmodels, -1);

    /* iteratively compare with all the images */
    for (unsigned int i = 0; i < nmodels; i++) {
      // skip if we use the model_mask
      if (using_mask && _forbidden_images[i])
        continue;
      float curr_result = hausdorff_distances::D22_with_min<cv::Point, Pt2Iterable>
          (_unknown_point_vec, models_points[i], _best_result,
           hausdorff_distances::dist_L1_int);
      _results[i] = curr_result;

      // printf("model_to_string(%i):'%s'\n", i, model_to_string(i).c_str());
      //    maggieDebug2("-> comparing with model #%i, result of the comparison: %f",
      //                 i, curr_result);
      if (curr_result < _best_result) { // new best curr_result !
        _best_result = curr_result;
        _best_index = i;
      }
    } // end loop i
    return (_best_index >= 0);
  } // end compare_internal_data();

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  //! the default size of the model, at which we will resize unknown objects
  cv::Size _default_size;
  //! the vector of the points of all the model filenames
  std::vector<Pt2Iterable> models_points;
  //! true to keep ratio of the models and of the unknown queries
  bool _keep_ratio;

  //! the image of the unknown object
  cv::Mat1b unknown_image;
  //! the points of the unknown object
  Pt2Iterable _unknown_point_vec_unscaled;
  //! the points of the unknown object; fitted to _default_size
  Pt2Iterable _unknown_point_vec;
  //! a buffer for redimContent_vector_without_repetition_given_resized_array()
  cv::Mat1b _resized_array;
  //! the filename of the best match model
  int _best_index;
  //! the result of the best match model
  float _best_result;
  std::vector<float> _results;
  //! the mask which allow some images
  std::vector<bool> _forbidden_images;
}; // end class ImageComparer_

typedef ImageComparer_<std::vector<cv::Point> > ImageComparer;

#endif
