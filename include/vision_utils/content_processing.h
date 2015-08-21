#ifndef CONTENT_PROCESSING_H
#define CONTENT_PROCESSING_H

// C++
#include <queue>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
// AD
#include "vision_utils/utils/geometry_utils.h"
#include "vision_utils/utils/cmatrix.h"
// vision
#include "vision_utils/io.h"
#include "vision_utils/mask.h"

namespace image_utils {

#if 0
/*!
 *\brief   a simple loop on an image
 */
inline void simpleLoop3b(const cv::Mat3b & img) {
  //    for(cv::Mat3b::const_iterator img_it = img.begin(); img_it != img.end() ; ++img_it) {
  //        std::cout << (*img_it)[0] << ", "
  //                  << (*img_it)[1] << ", "
  //                  << (*img_it)[2] << std::endl;
  //    }

  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const uchar* data = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; ++col) {
      // for RGB color
      std::cout << "(" << col << ", " << row << "):"
                << *data++ << ", "
                << *data++ << ", "
                << *data++ << std::endl;
    } // end loop col
  } // end loop row
} // end simpleLoop3b
#endif

////////////////////////////////////////////////////////////////////////////////

static std::string img2string(const cv::Mat1b & img) {
  std::ostringstream ans;
  ans << "(" << img.cols << "x" << img.rows << "):" << std::endl;
  int ncols = img.rows, nrows = img.rows;
  for (int row = 0; row < nrows; ++row) {
    const uchar* rowdata = img.ptr<uchar>(row);
    for (int col = 0; col < ncols; ++col)
      ans << (rowdata[col] ? 'X' : '-');
    ans << std::endl;
  }
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

template<class Pt2Iterable>
static std::string point_vec_to_string(const Pt2Iterable & in, const cv::Size size) {
  //cv::Rect r = geometry_utils::boundingBox_vec<cv::Rect, cv::Point>(in);
  //cv::Mat1b out(r.width, r.height);
  cv::Mat1b out(size, (uchar) 0);
  int cols = size.width;
  uchar* outdata = out.ptr(0);
  for (unsigned int pt_idx = 0; pt_idx < in.size(); ++pt_idx) {
    if (in[pt_idx].x >= 0 && in[pt_idx].x < size.width &&
        in[pt_idx].y >= 0 && in[pt_idx].y < size.height)
      outdata[in[pt_idx].y * cols + in[pt_idx].x] = 255;
  } // end loop pt_idx
  return img2string(out);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   resize a monochroome image in another one -
     *
 *\param   src monochrome image
 *\param   dest monochrome image
 */
inline void resizeMonochromeImage(const cv::Mat1b & src, cv::Mat1b & dest) {
  maggieDebug2("resizeMonochromeImage('%s', '%s'')",
               infosImage(src).c_str(), infosImage(dest).c_str());
  cv::resize(src, dest, dest.size(), 0, 0, cv::INTER_NEAREST);
}

////////////////////////////////////////////////////////////////////////////////

//! \return a bounding box corresponding to the full image
inline cv::Rect bbox_full(const cv::Mat & img) {
  return cv::Rect(0, 0, img.cols, img.rows);
}

////////////////////////////////////////////////////////////////////////////////

//! \return the center point of an image
inline cv::Point center(const cv::Mat & img) {
  return cv::Point(img.cols / 2, img.rows / 2);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   get the bounding box of the non null points of an image
 *\param   img a monochrome image
 *\return the bounding box of the non null points,
 *        cv::Rect(-1, -1, -1, -1) if the image is empty
 */
template<class _T>
static inline cv::Rect boundingBox(const cv::Mat_<_T> & img) {
  if (img.empty())
    return cv::Rect(-1, -1, -1, -1);
  assert(img.isContinuous());
  int xMin = 0, yMin = 0, xMax = 0, yMax = 0;
  bool was_init = false;
  const _T* img_it = (_T*) img.ptr(0);
  int nrows = img.rows, ncols = img.cols;
  for (int y = 0; y < nrows; ++y) {
    for (int x = 0; x < ncols; ++x) {
      if (*img_it++) {
        if (!was_init) {
          xMin = xMax = x;
          yMin = yMax = y;
          was_init = true;
          continue;
        }
        if (x < xMin)
          xMin = x;
        else if (x > xMax)
          xMax = x;

        if (y < yMin)
          yMin = y;
        else if (y > yMax)
          yMax = y;
      }
    } // end loop x
  } // end loop y

  if (!was_init) { // no white point found
    printf("boundingBox: no non null point found!\n");
    return cv::Rect(-1, -1, -1, -1);
  }
  // from http://docs.opencv.org/java/org/opencv/core/Rect.html
  // OpenCV typically assumes that the top and left boundary of the rectangle
  // are inclusive, while the right and bottom boundaries are not.
  // For example, the method Rect_.contains returns true if
  // x <= pt.x < x+width,   y <= pt.y < y+height
  return cv::Rect(xMin, yMin, 1 + xMax - xMin,  1 + yMax - yMin);
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   fit the non nul bounding box of src to fit to the size of dest
 *\param   src monochrome image
 *\param   dest monochrome image
 */
inline bool redimContent(const cv::Mat1b & src, cv::Mat1b & dest) {
  maggieDebug2("redimContent('%s', '%s'')",
               infosImage(src).c_str(), infosImage(dest).c_str());
  //cout << "bbox 1:" << cvGetImageROI(src).cols << "," << cvGetImageROI(src).rows << endl;
  //    cv::Rect bbox_orig = cvGetImageROI(src);
  //    //cout << "bbox res:" << bbox.cols << "," << bbox.rows << endl;
  //    cvSetImageROI(src, boundingBox(src));
  //    resizeMonochromeImage(src, dest);
  //    cvResetImageROI(src);
  //    cvSetImageROI(src, bbox_orig);

  cv::Rect bbox_src = boundingBox(src);
  if (!geometry_utils::bboxes_included(bbox_full(src), bbox_src))
    return false;
  resizeMonochromeImage(src(bbox_src), dest);
  return true;
}

////////////////////////////////////////////////////////////////////////////

#define USE_MAT1B
#ifdef USE_MAT1B
typedef uchar MyBool;
#else // USE_MAT1B
typedef bool MyBool;
#endif // USE_MAT1B

template<class Pt2Iterable>
inline void redimContent_vector_without_repetition_given_src_bbox(
    const Pt2Iterable & src,
    Pt2Iterable & rep,
    const cv::Rect & src_bbox,
    const cv::Rect & dst_bbox,
    MyBool* resized_array_ptr,
    bool keep_ratio = false) {
  assert(src != rep);
  maggieDebug3("redimContent_vector_without_repetition_given_src_bbox"
               "(bbox_src:%s, dst_bbox:%s, keep_ratio:%i)",
               rectangle_to_string(src_bbox).c_str(),
               rectangle_to_string(dst_bbox).c_str(), keep_ratio);

  /* compute the transformation
          { x' = aX * x + bX
          { y' = aY * y + bY
           with (src_bbox.x, src_bbox.y) matching to (0, 0)
           and  (src_bbox.width, src_bbox.height) matching
                   to (dst_bbox.width, dst_bbox.height)
    */
  float aX = (src_bbox.width != 1 ?
                                  1. * dst_bbox.width / src_bbox.width :
                                  0);
  float aY = (src_bbox.height != 1 ?
                                   1. * dst_bbox.height / src_bbox.height :
                                   0);
  if (keep_ratio) {
    if (fabs(aX) < fabs(aY)) // adjust Y : y' = aX * y + bY
      aY = aX;
    else // adjust x : x' = aY * x + bX
      aX = aY;
  } // end if keep_ratio

  // this is true, keep_ratio or not
  float bX = - aX * src_bbox.x;
  float bY = - aY * src_bbox.y;
  //maggieDebug2("x' = %g x + %g,  y' = %g y + %g", aX, bX, aY, bY);

  /* stock the results in an array */
  /* first init the array */
  unsigned int npts = src.size(), dstcols = dst_bbox.width, dstrows = dst_bbox.height;
  bzero(resized_array_ptr, sizeof(MyBool) * dstcols * dstrows);

  /* then mark the present points */
  int nb_points = 0;
  for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
    unsigned int index = dstcols
                         *(int) (aY * src[pt_idx].y + bY) // y
                         + (int) (aX * src[pt_idx].x + bX); // x
    MyBool* resized_array_pos_ptr = &(resized_array_ptr[index]);
    if (! *resized_array_pos_ptr) { // not initialized before
      ++nb_points;
      *resized_array_pos_ptr = true;
    }
  } // end loop pt_idx
  //maggieDebug2("nb_points:%i", nb_points);

  /* recopy the points marked in the array in the std::vector */
  rep.clear();
  rep.resize(nb_points);
  int rep_idx = 0;
  MyBool* resized_array_pos_ptr = &resized_array_ptr[0];
  for (unsigned int y = 0; y < dstrows; ++y) {
    for (unsigned int x = 0; x < dstcols; ++x) {
      if (*resized_array_pos_ptr++) {
        rep[rep_idx].x = dst_bbox.x + x;
        rep[rep_idx].y = dst_bbox.y + y;
        ++rep_idx;
      }
    } // end loop x
  } // end loop y

  //maggieDebug2("rep_size:%i", rep.size());
}

////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   fit a vector of points to a bounding box without repetition.
 * src and rep have to be different
 * resized_array_ptr needs to be allocated beforehand to (dst_bbox.height * dst_bbox.width)
 */
template<class Pt2Iterable>
inline void redimContent_vector_without_repetition_given_resized_array(
    const Pt2Iterable & src,
    Pt2Iterable & rep,
    const cv::Rect & dst_bbox,
    MyBool* resized_array_ptr,
    bool keep_ratio = false) {

  maggieDebug3("redimContent_vector_without_repetition_given_resized_array"
               "(dst_bbox:%s, keep_ratio:%i)",
               rectangle_to_string(dst_bbox).c_str(), keep_ratio);
  cv::Rect src_bbox = geometry_utils::boundingBox_vec
                      <Pt2Iterable, cv::Rect>(src);
  redimContent_vector_without_repetition_given_src_bbox<Pt2Iterable>
      (src, rep,
       src_bbox,
       dst_bbox,
       resized_array_ptr,
       keep_ratio);
} // end redimContent_vector_without_repetition_given_resized_array()

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   fit a vector of points to a bounding box without repetition.
 *src and rep have to be different
 */
template<class Pt2Iterable>
inline void redimContent_vector_without_repetition(
    const Pt2Iterable & src,
    Pt2Iterable & rep,
    const cv::Rect & dst_bbox,
    bool keep_ratio = false) {

  maggieDebug3("redimContent_vector_without_repetition"
               "(dst_bbox:%s, keep_ratio:%i)",
               rectangle_to_string(dst_bbox).c_str(), keep_ratio);
  assert(src != rep);
#ifdef USE_MAT1B
  cv::Mat1b resized_array;
  resized_array.create(dst_bbox.height, dst_bbox.width); // ROWS, COLS
#else // USE_MAT1B
  bool resized_array [dst_bbox.width * dst_bbox.height];
#endif // USE_MAT1B

  redimContent_vector_without_repetition_given_resized_array<Pt2Iterable>
      (src, rep,
       dst_bbox,
     #ifdef USE_MAT1B
       resized_array.data,
     #else // USE_MAT1B
       &resized_array[0],
    #endif // USE_MAT1B
      keep_ratio);
  //delete resized_array;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   get a vector from an image with all the non nul points
 */
template<class Pt2Iterable>
inline void nonNulPoints(const cv::Mat1b & img, Pt2Iterable & rep) {
  maggieDebug3("nonNulPoints()");
#if 0
  rep.clear();
  // rep.reserve( cv::countNonZero(img) );
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const uchar* data = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; ++col) {
      if (*data++)
        rep.push_back(cv::Point(col, row));
    } // end loop col
  } // end loop row
#else
  rep.resize( cv::countNonZero(img) );
  int rep_idx = 0;
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const uchar* data = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; ++col) {
      if (*data++) {
        rep[rep_idx].x = col;
        rep[rep_idx].y = row;
        ++rep_idx;
      }
    } // end loop col
  } // end loop row
#endif
}

////////////////////////////////////////////////////////////////////////////////

/*! Find the non null point with the lowest row index
 * in a mask, among non zero pixels.
 * We only search in columns with an index close to the vertical center
 * of the bounding box of the content.
 * \arg width_ratio_to_bbox in [0, 1]
 *   around 0, very narrow search window
 *   around 1, very wide search window (almost equal to image bbox)
 * \return (-1, -1) if not found
 */
template<class _T>
cv::Point find_top_point_centered(const cv::Mat_<_T> & image,
                                  bool compute_bbox,
                                  const double width_ratio_to_bbox = .3,
                                  const _T zero_value = _T(0),
                                  cv::Rect* search_window_out = NULL) {
  assert(width_ratio_to_bbox > 0 && width_ratio_to_bbox <= 1);
  // find bounding box and allowed columns
  cv::Rect bbox = (compute_bbox ? image_utils::boundingBox(image) : bbox_full(image));
  int
      min_col = bbox.x + (1.f - width_ratio_to_bbox) * bbox.width / 2,
      max_col = bbox.x + (1.f + width_ratio_to_bbox) * bbox.width / 2,
      min_row = bbox.y,
      max_row = bbox.y + bbox.height;
  //min_col = std::max(min_col, 0);
  //max_col = std::min(max_col, image.cols - 1);
  // return search window if user curious about it
  if (search_window_out != NULL)
    *search_window_out = cv::Rect(min_col, min_row, max_col - min_col + 1, max_row - min_row + 1);

  // run among the allowed window
  for (int row = min_row; row < max_row; ++row) {
    // get the address of row
    const _T* data = image.ptr(row);
    for (int col_begin = min_col; col_begin <= max_col; ++col_begin) {
      if (data[col_begin] != zero_value) {
        // now see where the non zeros stop
        int col_end = col_begin;
        while (col_end < image.cols && data[col_end] != zero_value)
          ++col_end;
        // return the middle of these non zero values
        return cv::Point((col_begin + col_end) / 2, row);
      }
    } // end loop col_begin
  } // end loop row
  printf("find_top_point_centered(img:%ix%i, search_window:(%ix%i)->(%ix%i)): "
         "could not find a non zero point!\n",
         image.cols, image.rows, min_col, min_row, max_col, max_row);
  return cv::Point(-1, -1);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 *\brief   get a vector from an image with all the non nul points
 */
template<class CoordIterable>
inline void nonNulPoints(const cv::Mat1b & img, CoordIterable & x, CoordIterable & y) {
  maggieDebug3("nonNulPoints()");
  x.resize( cv::countNonZero(img) );
  y.resize(x.size());
  int rep_idx = 0;
  for (int row = 0; row < img.rows; ++row) {
    // get the address of row
    const uchar* data = img.ptr<uchar>(row);
    for (int col = 0; col < img.cols; ++col) {
      if (*data++) {
        x[rep_idx] = col;
        y[rep_idx] = row;
        ++rep_idx;
      }
    } // end loop col
  } // end loop row
}

////////////////////////////////////////////////////////////////////////////

/* ************************************************************************
 *stuff with biggest components
 *************************************************************************/
#define NOT_SEEN  false
#define SEEN      true

inline void region_growth(const cv::Mat1b & mat,
                          bool* seenPoints,
                          const cv::Point2i seed,
                          const double maxDistWithNeighbour,
                          const double maxDistWithSeedValue,
                          std::vector<cv::Point2i>& queue) {

  maggieDebug3("region_growth()");

  unsigned int positionInQueue = 0;
  queue.clear();

  // fill seenPoints with false
  bool* seenPoints_ptr = seenPoints;
  for(int i = 0 ; i < mat.cols * mat.rows ; ++i)
    *seenPoints_ptr++ = NOT_SEEN;

  // init the queue with the seed
  queue.push_back(seed);
  int seedValue = mat(seed.y, seed.x);
  //maggieDebug2("seedValue:%i", seedValue);
  // mark as seen
  seenPoints[seed.y * mat.cols + seed.x] = SEEN;

  do {
    cv::Point2i head = queue.at(positionInQueue);
    ++positionInQueue;
    //maggieDebug2("Checking:(%i,%i)", head.x, head.y);
    int currentVal = mat(head.y, head.x);

    // check the neighbours
    for (int neighbour_id = 1; neighbour_id <= 4; neighbour_id ++) {

      int xNeighbour = head.x, yNeighbour = head.y;
      if (neighbour_id == 1) {
        ++xNeighbour;
        if (xNeighbour >= mat.cols)
          continue;
      }
      else if (neighbour_id == 2) {
        --xNeighbour;
        if (xNeighbour < 0)
          continue;
      }
      else if (neighbour_id == 3) {
        ++yNeighbour;
        if (yNeighbour >= mat.rows)
          continue;
      }
      else {
        --yNeighbour;
        if (yNeighbour < 0)
          continue;
      }

      // if already in the queue => skip
      if (seenPoints[yNeighbour * mat.cols + xNeighbour] == SEEN)
        continue;

      // mark as seen
      seenPoints[yNeighbour * mat.cols + xNeighbour] = SEEN;

      //maggieDebug2("Examining %i, %i", xNeighbour, yNeighbour);

      int neighbourValue = mat(yNeighbour, xNeighbour);

      //maggieDebug2("Dist:%i", abs(neighbourValue - currentVal));
      if ( (abs(neighbourValue - currentVal) > maxDistWithNeighbour)
           || (abs(neighbourValue - seedValue) > maxDistWithSeedValue)) {
        continue;
      }

      //maggieDebug2("adding it");

      // push in the queue
      queue .push_back(cv::Point2i(xNeighbour, yNeighbour));

    } // for neighbour_id

    //      imshow("seen", seenPoints);
    //      cv::waitKey(100);

  } while (positionInQueue < queue.size());
  // while queue

  //imshow("seen", seenPoints);
  //cv::waitKey(10000);
}


////////////////////////////////////////////////////////////////////////////

inline void region_growth_no_seen_points(
    const cv::Mat1b & mat,
    const cv::Point2i seed,
    const double maxDistWithNeighbour,
    const double maxDistWithSeedValue,
    std::vector<cv::Point2i>& queue) {
  // init seenPoints as an aray of bools
  bool seenPoints[mat.cols * mat.rows];
  region_growth(mat, seenPoints, seed, maxDistWithNeighbour,
                maxDistWithSeedValue, queue);
}

////////////////////////////////////////////////////////////////////////////////

#define QUEUEDIM unsigned short
#if 1 // queue
#define QUEUE              std::queue<QUEUEDIM>
#define QUEUECLEAR(q)      while (!(q).empty()) (q).pop()
#define QUEUEPUSH(q, v)    (q).push(v)
#define QUEUEPOP(q)        (q).pop()
#define QUEUEFRONT(q)      (q).front()
#elif 1 // list
#define QUEUE              std::priority_queue<QUEUEDIM>
#define QUEUECLEAR(q)      (q).clear()
#define QUEUEPUSH(q, v)    (q).push_back(v)
#define QUEUEPOP(q)        (q).erase((q).begin())
#define QUEUEFRONT(q)      (q).front()
#elif 1 // vector
#define QUEUE              std::vector<QUEUEDIM>
#define QUEUECLEAR(q)      (q).clear()
#define QUEUEPUSH(q, v)    (q).push_back(v)
#define QUEUEPOP(q)        (q).erase((q).begin())
#define QUEUEFRONT(q)      (q).front()
#else // deque
#define QUEUE              std::deque<QUEUEDIM>
#define QUEUECLEAR(q)      (q).clear()
#define QUEUEPUSH(q, v)    (q).push_back(v)
#define QUEUEPOP(q)        (q).pop_front()
#define QUEUEFRONT(q)      (q).front()
#endif

#define ADD2QUEUE_IF(test, nextkey, col, row, curr_queueval)   \
  if ((test) && !_queued_data[nextkey] && should_pt_be_added(nextkey)) { \
  QUEUEPUSH(_colsqueue, (col)); \
  QUEUEPUSH(_rowsqueue, (row)); \
  _queued_data[nextkey] = curr_queueval; \
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * The template <QueueType> corresponds to the kind of data we want to store
 * in the queued elements.
 * If we only want to mark elements as seen or unseen, then uchar is enough.
 * If we want to count the number of the path from the seed to this element,
 *  il will most likely be over 255, the short would be appropriate.
 */
template<class QueueType>
class PropagationTemplate {
public:
  PropagationTemplate() : _seen_pts_nb(0) {}

  /*! \return the number of pixels (including the seed)
   *  put in the queue before propagate() terminated.
   *  This can happen because the queue ran empty or is_over() returned true
   *  on one of the pixels.
   */
  inline int get_seen_pts_nb() const {
    return _seen_pts_nb;
  }

  //! get a const access to the queued matrix
  inline const cv::Mat_<QueueType>& get_queued() const {
    return _queued;
  }

  //! get a modifiable access to the queued matrix
  inline cv::Mat_<QueueType>& get_queued() {
    return _queued;
  }

  //! \return the size  of _queued
  inline cv::Size get_queued_size() const {
    return _queued.size();
  }

protected:
  /*! \return the key corresponding to is_over()
   *  or -1 if not found
   *  \arg propagate_along_diagonals: true if at each iteration,
   *    try to add the C8 neighbours of the current point instead of the C4 ones.
   */
  template<class _T>
  inline int propagate(const cv::Mat_<_T> & mask,
                       const cv::Point & seed,
                       bool propagate_along_diagonals = true) {
    _seen_pts_nb = 0;
    // check if seed valid
    if (!image_utils::bbox_full(mask).contains(seed)) {
      printf("seed %s outside of mask '%s'\n",
             geometry_utils::printP2(seed).c_str(), infosImage(mask).c_str());
      return -1;
    }
    _cols = mask.cols;
    _rows = mask.rows;
    _colsm = _cols - 1, _rowsm = _rows - 1;

    // stupid check: maybe seed is in mask
    _curr_key = seed.y * _rows + seed.x;
    if (is_over())
      return _curr_key;

    // init data
    _queued.create(mask.size());
    assert(mask.isContinuous());
    assert(_queued.isContinuous());
    _queued.setTo(0);
    _queued_data = (QueueType*) _queued.ptr();
    // clear queues and add seed
    QUEUECLEAR(_colsqueue);
    QUEUECLEAR(_rowsqueue);
    QUEUEPUSH(_colsqueue, seed.x);
    QUEUEPUSH(_rowsqueue, seed.y);
    _queued(seed) = true;

    while (!_colsqueue.empty()) {
      // check current point
      _curr_row = QUEUEFRONT(_rowsqueue);
      _curr_col = QUEUEFRONT(_colsqueue);
      _curr_key = _curr_row * _cols + _curr_col;
      // printf("Checking (%i, %i)\n", col, row);
      if (is_over()) // found!
        return _curr_key;
      // mark as seen
      QUEUEPOP(_colsqueue);
      QUEUEPOP(_rowsqueue);
      ++_seen_pts_nb;
      QueueType curr_queueval = custom_queue_val();
      // add C8 neighbours
      if (propagate_along_diagonals) {
        bool left_ok = _curr_col > 0, right_ok = _curr_col < _colsm,
            up_ok = _curr_row > 0, down_ok = _curr_row < _rowsm;
        ADD2QUEUE_IF(left_ok,             _curr_key-1, _curr_col-1, _curr_row, curr_queueval); // left
        ADD2QUEUE_IF(right_ok,            _curr_key+1, _curr_col+1, _curr_row, curr_queueval); // right
        ADD2QUEUE_IF(up_ok,               _curr_key-_cols, _curr_col, _curr_row-1, curr_queueval); // up
        ADD2QUEUE_IF(down_ok,             _curr_key+_cols, _curr_col, _curr_row+1, curr_queueval); // down
        ADD2QUEUE_IF(left_ok && up_ok,    _curr_key-1-_cols, _curr_col-1, _curr_row-1, curr_queueval); // left up
        ADD2QUEUE_IF(right_ok && up_ok,   _curr_key+1-_cols, _curr_col+1, _curr_row-1, curr_queueval); // right up
        ADD2QUEUE_IF(left_ok && down_ok,  _curr_key-1+_cols, _curr_col-1, _curr_row+1, curr_queueval); // left down
        ADD2QUEUE_IF(right_ok && down_ok, _curr_key+1+_cols, _curr_col+1, _curr_row+1, curr_queueval); // right down
      } // end if (propagate_along_diagonals)

      else { // not propagate_along_diagonals
        // add C4 neighbours
        ADD2QUEUE_IF(_curr_col > 0,     _curr_key-1, _curr_col-1, _curr_row, curr_queueval); // left
        ADD2QUEUE_IF(_curr_col < _colsm, _curr_key+1, _curr_col+1, _curr_row, curr_queueval); // right
        ADD2QUEUE_IF(_curr_row > 0,     _curr_key-_cols, _curr_col, _curr_row-1, curr_queueval); // up
        ADD2QUEUE_IF(_curr_row < _rowsm, _curr_key+_cols, _curr_col, _curr_row+1, curr_queueval); // down
      } // end if (!propagate_along_diagonals)
    } // end while (!_colsqueue.empty())

    // empty queue, point could not be found
    // printf("PropagationTemplate: queue empty, no point was satisfying is_over()\n");
    return -1;
  } // end propagate()

  //////////////////////////////////////////////////////////////////////////////

  /*! \return Key that is used for marking a point as seen. Must be > 0.
      It is called once for each examined point (ie, each iteration).
      Its value can change for each iteration, you can use for instance _seen_pts_nb.
      It is not const, enabling you to do stuff at each iteration
      (for instance keeping a log of the seen keys).
      You can use _curr_key if needed. */
  virtual QueueType custom_queue_val() = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! \return true if this neighbour should be added.
      \param next_key = row * cols + col of the point to be added,
      certified to be in image bounds.
      It is not const, enabling you to do stuff at each iteration
      (for instance keeping a log of the seen keys).
      You can use _curr_key if needed. */
  virtual bool should_pt_be_added(const int & next_key) = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*! \return true if curr_pt corresponds to the end of the search.
      You can use _curr_key if needed. */
  virtual bool is_over() const = 0;

  //////////////////////////////////////////////////////////////////////////////

  QUEUE _colsqueue;
  QUEUE _rowsqueue;
  cv::Mat_<QueueType> _queued;
  QueueType* _queued_data;
  int _seen_pts_nb;
  QUEUEDIM _cols, _rows, _colsm, _rowsm;
  QUEUEDIM _curr_row, _curr_col;
  int _curr_key;
}; // end class PropagationTemplate

////////////////////////////////////////////////////////////////////////////////

template<class _T>
class RegionGrower : public PropagationTemplate<uchar> {
public:
  /*!
   * Find recursively all the points around a seed that are
   * both close in value to their neighbour and to the seed.
   * \param img
   * \param seed
   * \param maxDistWithNeighbour
   * \param maxDistWithSeedValue
   * \param pts
   *    the list of points that match the given max distances.
   * \return
   *    true if success
   */
  bool grow(const cv::Mat_<_T> & img,
            const cv::Point & seed,
            const float & maxDistWithNeighbour,
            const float & maxDistWithSeedValue,
            std::vector<cv::Point> & pts) {
    _maxDistWithNeighbour = maxDistWithNeighbour;
    _maxDistWithSeedValue = maxDistWithSeedValue;
    pts.clear();
    if (img.empty() || !bbox_full(img).contains(seed))
      return false;
    _seed_img_value = img(seed);
    _imgdata = img.ptr(); // needed for should_pt_be_added()
    propagate(img, seed, true);
    // printf("nonzero:%i\n", cv::countNonZero(_queued));
    // cv::imshow("_queued", _queued); cv::waitKey(0);
    nonNulPoints(_queued, pts);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  //! the value of the key has no importance
  inline uchar custom_queue_val() { return 255; }
  //! add pts that are not different
  inline bool should_pt_be_added(const int & next_key) {
    float _nextvalue = (float) _imgdata[next_key];
    return (fabs(_nextvalue - (float) _imgdata[_curr_key]) <= _maxDistWithNeighbour
            && fabs(_nextvalue - _seed_img_value) <= _maxDistWithSeedValue);
  }
  //! \return false: dont stop till queue over
  inline bool is_over() const { return false; }

  float _maxDistWithNeighbour, _maxDistWithSeedValue;
  float _seed_img_value;
  const _T* _imgdata;
}; // end class RegionGrower

////////////////////////////////////////////////////////////////////////////////

template<class _T>
class ClosestPointInMask2 : public PropagationTemplate<uchar> {
public:
  /*!
   * For a given seed, find the closest point in a given mask
   * \param mask
   * \param seed
   * \return cv::Point(-1, -1) if could not be found
   * \param mask_minvalue, mask_maxvalue
   *    allowed values for the closest point
   * \return
   */
  cv::Point find(const cv::Mat_<_T> & mask,
                 const cv::Point & seed,
                 const _T mask_minvalue = 1, const _T mask_maxvalue = 255) {
    _mask_minvalue = mask_minvalue;
    _mask_maxvalue = mask_maxvalue;
    _maskdata = mask.ptr(); // needed for is_over()
    int key = propagate(mask, seed, false);
    if (key == -1) {
      printf("propagate() returned a negative key, most probably the mask is empty\n");
      return cv::Point(-1, -1);
    }
    return cv::Point(key % mask.cols, key / mask.cols);
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  //! the value of the key has no importance
  inline uchar custom_queue_val() { return 255; }
  //! always add pts in bounds
  inline bool should_pt_be_added(const int &) {  return true; }
  //! \return true when mask[key] has a value > 0 and in bounds
  inline bool is_over() const {
    return (_maskdata[_curr_key] >= _mask_minvalue
            && _maskdata[_curr_key] <= _mask_maxvalue);
  }

  _T _mask_minvalue, _mask_maxvalue;
  const _T* _maskdata;
}; // end class ClosestPointInMask2

////////////////////////////////////////////////////////////////////////////////

template<class _T>
class PropagativeFloodfiller : public PropagationTemplate<short> {
public:
  /*!
   * Make a propagative floodfill from a given seed
   * and store the distances in a buffer.
   * \param img
   *  Where to make the floodfill from.
   *  The propatation will be done on non-zero pixels (like a mask).
   * \param seed
   *  Where to start the propagation from.
   * \param search_top_point_centered
   *  if true, instead of using \a seed, use the point obtained by
   *  calling find_top_point_centered(img, seed)
   * \param lookup_function, lookup_result, lookup_cookie
   *  optionnaly, a lookup function can be called.
   *  For each pixel of the propagation , it converts the distance from the seed,
   *  in pixels, into whatever you want.
   *  The result of this lookup is then stored into \a lookup_result.
   *  Optionnaly, some data can be passed to \a lookup_function()
   *  thanks to the \a lookup_cookie.
   */
  bool floodfill(const cv::Mat_<_T> & mask,
                 const cv::Point & seed,
                 bool search_top_point_centered = false,
                 bool propagate_along_diagonals = true,
                 float (*lookup_function)
                 (const int propagation_pixel_dist, const int row, const int col, void* cookie) = NULL,
                 cv::Mat1f* lookup_result = NULL,
                 void* lookup_cookie = NULL) {
    cv::Point real_seed = (search_top_point_centered ?
                             find_top_point_centered(mask, .4) :
                             seed);
    if (!bbox_full(mask).contains(real_seed)) {
      printf("seed %s outside of mask '%s'\n",
             geometry_utils::printP2(real_seed).c_str(), infosImage(mask).c_str());
      return false;
    }
    _maskdata = mask.ptr(); // needed for should_pt_be_added()
    // we dont care about return value of propagate()
    propagate(mask, real_seed, propagate_along_diagonals);
    if (lookup_function == NULL)
      return true;

    // now use the lookup function if wanted
    int rows = mask.rows, cols = mask.cols;
    lookup_result->create(mask.size());
    lookup_result->setTo(0);
    for (int row = 0; row < rows; ++row) {
      // get the address of row
      short* queued_data = _queued.ptr<short>(row);
      float* lookup_result_data = lookup_result->ptr<float>(row);
      for (int col = 0; col < cols; ++col) {
        if (queued_data[col])
          lookup_result_data[col] =
              lookup_function(queued_data[col], row, col, lookup_cookie);
      } // end loop col
    } // end loop row
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline short value_at(const cv::Point & pos) {
    return _queued.at<short>(pos);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void illus_img(cv::Mat3b & queued_illus,
                        bool paint_nans_in_black = true) {
    _queued.convertTo(queued_float_buffer, CV_32F);
    // convert seen buffer into a visualisation image
    image_utils::depth_image_to_vizualisation_color_image
        (queued_float_buffer, queued_illus, image_utils::FULL_RGB_STRETCHED, paint_nans_in_black);
  }

protected:
  //! increment one of the value of the key
  inline short custom_queue_val() { return 1 + _queued_data[_curr_key]; }
  //! add points belonging to mask
  inline bool should_pt_be_added(const int & next_key) {  return _maskdata[next_key]; }
  //! \return false: propagate as long as queue not empty
  inline bool is_over() const { return false; }

  const _T* _maskdata;
  cv::Mat1f queued_float_buffer;
}; // end class PropagativeFloodfiller

////////////////////////////////////////////////////////////////////////////////

template<class _T>
class GradientDescender : public PropagationTemplate<uchar> {
public:
  /*!
   * Recursively propagate among mask non null pixels,
   * such as each pixel has a strictly inferior value to the current one.
   * \param mask
   * \param seed
   * \param pts
   *    the found path, starting at seed
   * \param propagate_along_diagonals
   * \return
   *    true if there is an existing path
   */
  bool descend(const cv::Mat_<_T> & mask,
               const cv::Point & seed,
               std::vector<cv::Point> & pts,
               bool propagate_along_diagonals = true) {
    pts.clear();
    if (!bbox_full(mask).contains(seed)) {
      printf("seed %s outside of mask '%s'\n",
             geometry_utils::printP2(seed).c_str(), infosImage(mask).c_str());
      return false;
    }
    _pts_ptr = &pts;
    _cols = mask.cols;
    _maskdata = (_T*) mask.ptr(); // needed for is_over()
    _min_queued_value = mask(seed);
    // we dont care about return value of propagate()
    propagate(mask, seed, propagate_along_diagonals);
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  //! the value of the key has no importance
  inline uchar custom_queue_val() {
    // we can add this next point
    _pts_ptr->push_back(cv::Point(_curr_col, _curr_row));
    return 255;
  }
  //! add points belonging to mask
  inline bool should_pt_be_added(const int & next_key) {
    _T _next_queued_value = _maskdata[next_key];
    if (!_next_queued_value || _next_queued_value >= _min_queued_value)
      return false;
    _min_queued_value = _next_queued_value;
    return true;
  }
  //! \return false: descend as long as queue not empty
  inline bool is_over() const { return false; }

  int _cols;
  const _T* _maskdata;
  _T _min_queued_value;
  std::vector<cv::Point>* _pts_ptr;
}; // end class GradientDescender

////////////////////////////////////////////////////////////////////////////////

template<class _T>
class ShortestPathFinder : public PropagationTemplate<short> {
public:
  /*!
   * Find the closest point to a seed that has a non null value in a given mask
   * \param mask
   *    the authorized paths
   * \param begin
   *    the starting point (seed)
   * \param end
   *    the end point
   * \param is_propagation_already_done
   *    true if the floodfill propagation from \arg begin to \arg end was already done.
   *    In that case, that propagation (of type cv::Mat1s)
   *    should be copied to \a _queued
   *    and also given as \arg mask
   * \return cv::Point(-1, -1) if no path could be found between
   *    \arg begin and \arg end
   */
  bool find(const cv::Mat_<_T> & mask,
            const cv::Point & begin,
            const cv::Point & end,
            std::vector<cv::Point> & path,
            bool propagate_along_diagonals = true,
            bool is_propagation_already_done = false) {
    path.clear();
    if (!bbox_full(mask).contains(end)) {
      printf("end %s outside of mask '%s'\n",
             geometry_utils::printP2(end).c_str(), infosImage(mask).c_str());
      return false;
    }
    _maskdata = (_T*) mask.ptr(); // needed for is_over()
    _endkey = end.y * mask.cols + end.x;
    if (is_propagation_already_done) {
      // a few proofs here...
    }
    else {
      int key = propagate(mask, begin, propagate_along_diagonals);
      if (key != _endkey) {
        printf("find(begin:(%i,%i), end:(%i,%i)) returned %i != endkey:%i, "
               "there is no path between these points.\n",
               begin.x, begin.y, end.x, end.y, key, _endkey);
        return false;
      }
    } // end if (!is_propagation_already_done)
    // std::cout << "_queued:" << _queued << std::endl;

    // now start from end and go back from start,
    // following the decreasing _queue values
    int path_length = _queued(end);
    _path_reversed.reserve(path_length);
    bool ok = _descender.descend(_queued, end, _path_reversed, propagate_along_diagonals);
    if (!ok)
      return false;
    // reverse _path_reversed -> path
    path.reserve(_path_reversed.size());
    std::copy(_path_reversed.rbegin(), _path_reversed.rend(), std::back_inserter(path));
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  //! increment one of the value of the key
  inline short custom_queue_val() { return 1 + _queued_data[_curr_key]; }
  //! add points belonging to mask
  inline bool should_pt_be_added(const int & next_key) {  return _maskdata[next_key]; }
  //! \return true when we reach end point
  inline bool is_over() const { return _curr_key == _endkey; }

  _T _mask_minvalue, _mask_maxvalue;
  const _T* _maskdata;
  int _endkey;
  GradientDescender<short> _descender;
  std::vector<cv::Point> _path_reversed;
}; // end class ShortestPathFinder

////////////////////////////////////////////////////////////////////////////////

/*! \todo reimplement with A* ?
 http://www.boost.org/doc/libs/1_50_0/libs/graph/doc/astar_search.html
 http://www.boost.org/doc/libs/1_46_1/libs/graph/example/astar_maze.cpp
*/
class ClosestPointInMask {
public:
  /*!
   * Find the closest point to a seed that has a non null value in a given mask
   * \param mask
   * \param seed
   * \return cv::Point(-1, -1) if could not be found
   * \param mask_minvalue, mask_maxvalue
   *    allowed values for the closest point
   * \return
   */
  template<class _T>
  cv::Point find(const cv::Mat_<_T> & mask,
                 const cv::Point & seed,
                 const _T mask_minvalue = 1, const _T mask_maxvalue = 255) {
    // check if seed valid
    if (!bbox_full(mask).contains(seed)) {
      printf("seed %s outside of mask '%s'\n",
             geometry_utils::printP2(seed).c_str(), infosImage(mask).c_str());
      return cv::Point(-1, -1);
    }
    // stupid check: maybe seed is in mask
    if (mask(seed) >= mask_minvalue && mask(seed) <= mask_maxvalue)
      return seed;

    // init data
    int cols = mask.cols, rows = mask.rows;
    queued.create(mask.size());
    assert(mask.isContinuous());
    assert(queued.isContinuous());
    queued.setTo(0);
    _T* maskdata = (_T*) mask.ptr();
    uchar* queued_data = queued.ptr();
    // clear queues and add seed
    QUEUECLEAR(colsqueue);
    QUEUECLEAR(rowsqueue);
    QUEUEPUSH(colsqueue, seed.x);
    QUEUEPUSH(rowsqueue, seed.y);
    queued(seed) = true;

    while (!colsqueue.empty()) {
      // check current point
      QUEUEDIM row = QUEUEFRONT(rowsqueue), col = QUEUEFRONT(colsqueue);
      int key = row * cols + col;
      // printf("Checking (%i, %i)\n", col, row);
      if (maskdata[key] >= mask_minvalue && maskdata[key] <= mask_maxvalue) // found!
        return cv::Point(col, row);
      // mark as seen
      QUEUEPOP(colsqueue);
      QUEUEPOP(rowsqueue);
      // add C4 neighbours
      if (col > 0 && !queued_data[key-1]) { // left
        QUEUEPUSH(colsqueue, col-1);
        QUEUEPUSH(rowsqueue, row);
        queued_data[key-1] = true;
      }
      if (col < cols-1 && !queued_data[key+1]) { // right
        QUEUEPUSH(colsqueue, col+1);
        QUEUEPUSH(rowsqueue, row);
        queued_data[key+1] = true;
      }
      if (row > 0 && !queued_data[key-cols]) { // up
        QUEUEPUSH(colsqueue, col);
        QUEUEPUSH(rowsqueue, row-1);
        queued_data[key-cols] = true;
      }
      if (row < rows-1 && !queued_data[key+cols]) { // down
        QUEUEPUSH(colsqueue, col);
        QUEUEPUSH(rowsqueue, row+1);
        queued_data[key+cols] = true;
      }
    } // end while (!colsqueue.empty())
    // empty queue, point could not be found
    return cv::Point(-1, -1);
  }

private:
  QUEUE colsqueue;
  QUEUE rowsqueue;
  cv::Mat1b queued;
}; // end class ClosestPointInMask

////////////////////////////////////////////////////////////////////////////////

template<class _T>
class HighestPointFinder : public PropagationTemplate<uchar> {
public:
  /*!
   * For a given seed, find the closest point in a given mask
   * \param mask
   * \param seed
   * \return cv::Point(-1, -1) if could not be found
   * \param mask_minvalue, mask_maxvalue
   *    allowed values for the closest point
   * \return
   */
  cv::Point find(const cv::Mat_<_T> & mask,
                 const cv::Point & seed) {
    _maskdata = mask.ptr(); // needed for should_pt_be_added()
    _best_pt = seed;
    propagate(mask, seed, false); // key does not matter
    if (_seen_pts_nb < 1) {
      printf("find(): there is no point in the mask\n");
      return cv::Point(-1, -1);
    }
    return _best_pt;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  //! the value of the key has no importance
  inline uchar custom_queue_val() {
    if (_best_pt.y > _curr_row) { // keep highest pt
      _best_pt.x = _curr_col;
      _best_pt.y = _curr_row;
    }
    return 255;
  }
  //! add pts in the upper row and in the mask
  inline bool should_pt_be_added(const int & key) {
    return (key / _cols <= _curr_row) && _maskdata[key];
  }
  //! run till queue over
  inline bool is_over() const { return false; }
  const _T* _maskdata;
  cv::Point _best_pt;
}; // end class HighestPointFinder

////////////////////////////////////////////////////////////////////////////////

/*!
 * If a blob is well-defined thanks to an edge, except at its lower bound,
 * close it, ie extend its edge.
 * Typical use: the contour of an user obtained by depth canny.
 * The feet in contact with the ground "open" the contour at its base.
 * This function will close this blob.
 * \param src
 *  the mask where to propagate.
 *    all points that are not \a edge_value let propagation flow.
 * \param seed
 *   where to start the propagation from
 * \param floodfill_buffer
 *    keep track of seen pixels. \see floodfill_row()
 * \param clean_closing
 *    if true, in the row containing an opening,
 *    only paint in black the pixels detected as edge.
 *    otherwise draw a black line across the image at this row (dirtier, safer)
 * \param edge_value
 *    if a pixel has this value, it stops the row floodfill
 * \param prev_line_diff_thres, src_width_ratio_thres
 *   parameter for determining if there is an horizontal opening.
 *   we call C the width of the current row and P the width of the previous one.
 *   if  C > \a prev_line_diff_thres + P ,
 *   or C > \a src_width_ratio_thres * src.cols,
 *    we consider the current row as an opening and close it.
 */
class FloodFillEdgeCloser {
public:
  static const double DEFAULT_PREV_LINE_DIFF_THRES = 60;
  static const double DEFAULT_SRC_WIDTH_RATIO_THRES = .6;

  FloodFillEdgeCloser () {}

  bool close(cv::Mat1b & src,
             const cv::Point & seed,
             bool find_highest_point = true,
             bool clean_closing = true,
             const uchar edge_value = 0,
             double prev_line_diff_thres = DEFAULT_PREV_LINE_DIFF_THRES,
             double src_width_ratio_thres = DEFAULT_SRC_WIDTH_RATIO_THRES) {
    cv::Point final_seed = seed;
    if (find_highest_point)
      final_seed = _highest_pt_finder.find(src, seed);

    // check seed well defined
    if (!bbox_full(src).contains(final_seed)) {
      printf("seed %s outside of img '%s'\n",
             geometry_utils::printP2(final_seed).c_str(), infosImage(src).c_str());
      return false;
    }
    // allocate buffer
    floodfill_buffer.resize(src.rows, src.cols);
    floodfill_buffer.set_to_zero();
    // init buffer to seed
    int rows = src.rows, cols = src.cols;
    int curr_row = final_seed.y + 1;
    int prev_row_size = floodfill_row(src, final_seed.y, final_seed.x, edge_value);

    while(curr_row < rows) {
      int curr_row_size = 0;
      // propagate floodfill from previous row
      for (int col = 0; col < cols; ++col) {
        // upper pixel already in floodfill && this pixel not seen
        if (floodfill_buffer[curr_row - 1][col] && !floodfill_buffer[curr_row][col])
          curr_row_size += floodfill_row(src, curr_row, col, edge_value);
      } // end loop col

      // printf("curr_row:%i, curr_row_size:%i\n", curr_row, curr_row_size);
#if 1 // determine if it corresponds to a big opening
      bool is_line_opening = false;
      if (curr_row_size > prev_line_diff_thres + prev_row_size) {
        printf("curr_row:%i is an opening: "
               "curr_row_size:%i > %g = prev_line_diff_thres:%g + prev_row_size:%i\n",
               curr_row, curr_row_size, prev_line_diff_thres + prev_row_size,
               prev_line_diff_thres, prev_row_size);
        is_line_opening = true;
      }
      else if (curr_row_size > 1. * src_width_ratio_thres * cols) {
        printf("curr_row:%i is an opening: "
               "curr_row_size:%i > %g = src_width_ratio_thres:%g x cols:%i\n",
               curr_row, curr_row_size, src_width_ratio_thres * cols,
               src_width_ratio_thres, cols);
        is_line_opening = true;
      }
      if (is_line_opening) { // close the opening
        if (clean_closing) {
          for (int col = 0; col < cols; ++col) {
            if (floodfill_buffer[curr_row - 1][col])
              src(curr_row, col) = edge_value;
          } // end loop col
        }
        else // diry closing
          cv::line(src, cv::Point(0, curr_row), cv::Point(cols, curr_row), cv::Scalar::all(0), 1);
        return true; // job is over
      } // end if is_line_opening
#endif

      // go to next row
      prev_row_size = curr_row_size;
      ++ curr_row;
    } // end while()
    return true;
  } // end floodfill_edge_closer

  //////////////////////////////////////////////////////////////////////////////

private:
  /*!
   * Make a floodfill on an image, only for a given row.
   * \param src
   *    the input image for the row floodfill.
   *    all points that are not \a edge_value let propagation flow.
   * \param seen_points_buffer
   *    keeps track of the seen points.
   *    the points already marked as true stop propagation.
   *    the points marked as false, and where we propagate, are then marked as true.
   * \param row
   *    the indice of the row of \a src where to make the row floodfill
   * \param col
   *    the start column for the row floodflill
   * \param edge_value
   *    if a pixel has this value, it stops the row floodfill
   * \return
   *   the number of pixels that were filled.
   */
  template<class _T>
  inline int floodfill_row(const cv::Mat_<_T> & src,
                           const int & row,
                           const int & col,
                           const _T edge_value = _T(0)) {
    const _T* curr_row_data = src.ptr(row);
    int curr_row_incr_size = 0;
    int incr = 0; // expand on the right
    while(col + incr < src.cols
          && curr_row_data[col + incr] != edge_value
          && !floodfill_buffer[row][col + incr]) {
      floodfill_buffer[row][col + incr] = true;
      ++curr_row_incr_size;
      ++incr;
    }
    incr = 1; // expand on the left - do not repeat points
    while(col - incr >= 0
          && curr_row_data[col - incr] != edge_value
          && !floodfill_buffer[row][col - incr]) {
      floodfill_buffer[row][col - incr] = true;
      ++curr_row_incr_size;
      ++incr;
    }
    return curr_row_incr_size;
  } // end floodfill_row()

  //////////////////////////////////////////////////////////////////////////////

  HighestPointFinder<uchar> _highest_pt_finder;
  CMatrix<bool> floodfill_buffer;
}; // end class FloodFillEdgeCloser


////////////////////////////////////////////////////////////////////////////////

/*!
 * Find the min non zero value in an image, starting at the bottom row
 * and going upwards.
 * \param img
 *    the image
 * \param minnonzero
 *    the minimum non null value found on the lowest non null row
 * \param minnonzero_pos
 *    the exact position where \a minnonzero was found
 * \return
 *    true if there was a non null value found,
 *    false if img == 0
 */
template<class _T>
inline bool minnonzero_from_botton_row(const cv::Mat_<_T> & img,
                                       _T & minnonzero,
                                       cv::Point & minnonzero_pos) {
  minnonzero = std::numeric_limits<_T>::max();
  // start at one-before-last row
  unsigned int row = img.rows - 2, ncols = img.cols;
  bool was_found = false;
  // const _T* data = img.data;
  while (row > 0) {
    const _T* data = (const _T*) (img.ptr(row));
    for (unsigned int col = 0; col < ncols; ++col) {
      if (data[col]) {
        if (minnonzero > data[col]) {
          minnonzero = data[col];
          minnonzero_pos.x = col;
          minnonzero_pos.y = row;
        }
        was_found = true;
      } // end if (data[col])
    } // end loop col
    if (was_found) // stop if found something
      break;
    --row; // otherwise go to previous row
  } // end
  return (was_found);
}

////////////////////////////////////////////////////////////////////////////////

typedef short BufferElem;
//#define PROPAGATE_FLOODFILL_USE_STDVEC

#ifdef PROPAGATE_FLOODFILL_USE_STDVEC
#define SEEN_BUFFER_TYPE std::vector<short>
#define SEEN_BUFFER_ACCESS(row, col) queued_short[(row) * cols + (col)]
#else // no CMatrix
#define SEEN_BUFFER_TYPE cv::Mat_<short>
#define SEEN_BUFFER_ACCESS(row, col) queued_short_data[(row) * cols + (col)]
#endif // PROPAGATE_FLOODFILL_USE_STDVEC

/*!
 * Make a propagative floodfill from a given seed
 * and store the distances in a buffer.
 * \param img
 *  Where to make the floodfill from.
 *  The propatation will be done on non-zero pixels (like a mask).
 * \param seed
 *  Where to start the propagation from.
 * \param queued_short
 *  Where we store the propagation distance in pixel:
 *  the value at (x, y) is the length of the shortest C1 path from \a seed to (x, y).
 * \param search_top_point_centered
 *  if true, instead of using \a seed, use the point obtained by
 *  calling find_top_point_centered(img, seed)
 * \param lookup_function, lookup_result, lookup_cookie
 *  optionnaly, a lookup function can be called.
 *  For each pixel of the propagation , it converts the distance from the seed,
 *  in pixels, into whatever you want.
 *  The result of this lookup is then stored into \a lookup_result.
 *  Optionnaly, some data can be passed to \a lookup_function()
 *  thanks to the \a lookup_cookie.
 */
void propagative_floodfill(const cv::Mat1b & img,
                           cv::Point & seed,
                           SEEN_BUFFER_TYPE & queued_short,
                           bool search_top_point_centered = false,
                           float (*lookup_function)
                           (const int propagation_pixel_dist, const int row, const int col, void* cookie) = NULL,
                           cv::Mat1f* lookup_result = NULL,
                           void* lookup_cookie = NULL,
                           bool propagate_along_diagonals = true) {
  if (!img.isContinuous()) {
    printf("floodfill() only works with continous images!\n");
    return;
  }
  IplImage img_ipl = img;
  int cols = img.cols, rows = img.rows;
  int max_col = cols - 1, max_row = rows - 1;

  // search for top point if wanted
  // cv::Point seed = seed;
  if (search_top_point_centered)
    seed = find_top_point_centered(img, .4);

  // will we call the lookup_function
  bool want_lookup_function = (lookup_function != NULL);
  IplImage lookup_result_ipl;
  if (want_lookup_function) {
    lookup_result->create(img.size());
    lookup_result->setTo(0);
    lookup_result_ipl = *lookup_result;
  }

  // init queue with seed
  std::queue<int> queue_row, queue_col;
  queue_col.push(seed.x);
  queue_row.push(seed.y);
  // set the image as unseen except in seed
#ifdef PROPAGATE_FLOODFILL_USE_STDVEC
  queued_short.resize(rows * cols);
  std::fill(queued_short.begin(), queued_short.end(), 0);
#else // no CMatrix
  queued_short.create(img.size());
  queued_short.setTo(0);
  BufferElem* queued_short_data = queued_short.ptr<BufferElem>(0);
#endif // PROPAGATE_FLOODFILL_USE_STDVEC
  SEEN_BUFFER_ACCESS(seed.y, seed.x) = 1;

  while (queue_row.size() > 0) {
    // get next point in queue
    int curr_row = queue_row.front();
    queue_row.pop();
    int curr_col = queue_col.front();
    queue_col.pop();
    BufferElem curr_buff_val = SEEN_BUFFER_ACCESS(curr_row, curr_col);
    // lookup
    if (want_lookup_function) {
      CV_IMAGE_ELEM(&lookup_result_ipl, float, curr_row, curr_col) =
          lookup_function(curr_buff_val, curr_row, curr_col, lookup_cookie);
    } // end if (want_lookup_function)

    // printf("queue front:(%i, %i), curr_buff_val:%i\n", curr_col, curr_row, curr_buff_val);

    // check each of the neigbours
    // up
    if (curr_row > 0
        && SEEN_BUFFER_ACCESS(curr_row - 1, curr_col) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row - 1, curr_col) != 0) {
      SEEN_BUFFER_ACCESS(curr_row - 1, curr_col) = curr_buff_val + 1;
      queue_row.push(curr_row - 1);
      queue_col.push(curr_col);
    }
    // down
    if (curr_row < max_row
        && SEEN_BUFFER_ACCESS(curr_row + 1, curr_col) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row + 1, curr_col) != 0) {
      SEEN_BUFFER_ACCESS(curr_row + 1, curr_col) = curr_buff_val + 1;
      queue_row.push(curr_row + 1);
      queue_col.push(curr_col);
    }
    // left
    if (curr_col > 0
        && SEEN_BUFFER_ACCESS(curr_row, curr_col - 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row, curr_col - 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row, curr_col - 1) = curr_buff_val + 1;
      queue_row.push(curr_row);
      queue_col.push(curr_col - 1);
    }
    // right
    if (curr_col < max_col
        && SEEN_BUFFER_ACCESS(curr_row, curr_col + 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row, curr_col + 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row, curr_col + 1) = curr_buff_val + 1;
      queue_row.push(curr_row);
      queue_col.push(curr_col + 1);
    }
    if (!propagate_along_diagonals)
      continue;
    // up left
    if (curr_row > 0
        && curr_col > 0
        && SEEN_BUFFER_ACCESS(curr_row - 1, curr_col - 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row - 1, curr_col - 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row - 1, curr_col - 1) = curr_buff_val + 1;
      queue_row.push(curr_row - 1);
      queue_col.push(curr_col - 1);
    }
    // up right
    if (curr_row > 0
        && curr_col < max_col
        && SEEN_BUFFER_ACCESS(curr_row - 1, curr_col + 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row - 1, curr_col + 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row - 1, curr_col + 1) = curr_buff_val + 1;
      queue_row.push(curr_row - 1);
      queue_col.push(curr_col + 1);
    }
    // down left
    if (curr_row < max_row
        && curr_col > 0
        && SEEN_BUFFER_ACCESS(curr_row + 1, curr_col - 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row + 1, curr_col - 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row + 1, curr_col - 1) = curr_buff_val + 1;
      queue_row.push(curr_row + 1);
      queue_col.push(curr_col - 1);
    }
    // down right
    if (curr_row < max_row
        && curr_col < max_col
        && SEEN_BUFFER_ACCESS(curr_row + 1, curr_col + 1) == 0 // not seen before
        && CV_IMAGE_ELEM(&img_ipl, uchar, curr_row + 1, curr_col + 1) != 0) {
      SEEN_BUFFER_ACCESS(curr_row + 1, curr_col + 1) = curr_buff_val + 1;
      queue_row.push(curr_row + 1);
      queue_col.push(curr_col + 1);
    }
  } // end while queue not empty
} // end propagative_floodfill();

////////////////////////////////////////////////////////////////////////////////

/*!
 * Convert the queued modified by propagative_floodfill()
 * into a neat image for debugging.
 * \param queued_short
 *   the buffer, once modified by propagative_floodfill()
 * \param queued_float_buffer
 *   a buffer. It will contain the same values as queued_short,
 *   but as floats
 * \param greyscale_buffer
 *   Needed for depth_image_to_vizualisation_color_image()
 * \param queued_illus
 *   The output image.
 */
void propagative_floodfill_seen_buffer_to_viz_image(const SEEN_BUFFER_TYPE & queued_short,
                                                    cv::Mat1f & queued_float_buffer,
                                                    cv::Mat3b & queued_illus) {
#ifdef PROPAGATE_FLOODFILL_USE_STDVEC
  // Mat_(int _rows, int _cols, _Tp* _data, size_t _step=AUTO_STEP);
  cv::Mat1s queued_short_cv(img.rows, img.cols, queued_short.data());
  queued_float_buffer.convertTo(queued_float, CV_32F);
#else // no CMatrix
  queued_short.convertTo(queued_float_buffer, CV_32F);
#endif // PROPAGATE_FLOODFILL_USE_STDVEC
  // convert seen buffer into a visualisation image
  image_utils::depth_image_to_vizualisation_color_image
      (queued_float_buffer, queued_illus, image_utils::FULL_RGB_STRETCHED);
} // end propagative_floodfill_seen_buffer_to_viz_image()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Find all different values of an image and put them into a vector.
 * There is no repetition in that vector
 * \param img
 * \param out
 *    the vector containing all values of img with no repetition
 * \param mask
 *    optionnaly, a mask to apply on the image
 */
template<class _T>
inline void get_all_different_values(const cv::Mat_<_T> & img,
                                     std::set<_T> & out,
                                     bool ignore_zeros = true,
                                     const cv::Mat1b & mask = cv::Mat1b()) {
  assert(img.isContinuous());
  // determine if use mask
  bool use_mask = (!mask.empty());
  const uchar* mask_value = (use_mask ? mask.ptr<uchar>() : NULL);
  // iterate on image
  out.clear();
  const _T* img_value = (const _T*) img.ptr();
  unsigned int n_pixels = img.cols * img.rows;
  for (unsigned int pixel_idx = 0; pixel_idx < n_pixels; ++pixel_idx) {
    if ((!ignore_zeros || *img_value) && (!use_mask || *mask_value++))
      out.insert(*img_value);
    ++img_value;
  } // end loop pixel_idx
} // end get_all_different_values()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Find all different values of an image and put them into a vector.
 * There is no repetition in that vector
 * \param img
 * \param out
 *    the vector containing all values of img with no repetition
 * \param mask
 *    optionnaly, a mask to apply on the image
 */
template<class _T>
inline void get_all_different_values(const cv::Mat_<_T> & img,
                                     std::vector<_T> & out,
                                     bool ignore_zeros = true,
                                     const cv::Mat1b & mask = cv::Mat1b()) {
  std::set<_T> img_labels;
  get_all_different_values(img, img_labels, ignore_zeros, mask);
  // copy set to vector
  // cf http://stackoverflow.com/questions/5034211/c-copy-set-to-vector
  out.clear();
  out.reserve(img_labels.size());
  std::copy(img_labels.begin(), img_labels.end(), std::back_inserter(out));
} // end get_all_different_values()

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline bool get_all_non_null_values_and_com(const cv::Mat_<_T> & img,
                                            std::map<_T, cv::Point> & out,
                                            bool ignore_zeros = true,
                                            bool ensure_com_on_mask = false,
                                            const cv::Mat1b & mask = cv::Mat1b()) {
  if (img.empty()) {
    out.clear();
    return true;
  }
  assert(img.isContinuous());
  typedef typename std::map<_T, cv::Point3i> ComsMap;
  ComsMap coms;
  // determine if use mask
  bool use_mask = (!mask.empty());
  const uchar* mask_value = (use_mask ? mask.ptr<uchar>() : NULL);
  // iterate on image
  int cols = img.cols, rows = img.rows;
  out.clear();
  for (int row = 0; row < rows; ++row) {
    // get the address of row
    const _T* imgdata = img.ptr(row);
    for (int col = 0; col < cols; ++col) {
      if ((!ignore_zeros || imgdata[col]) && (!use_mask || *mask_value++)) {
        typename ComsMap::iterator coms_pos = coms.find(imgdata[col]);
        if (coms_pos == coms.end())
          coms.insert(coms_pos,
                      std::pair<_T, cv::Point3i>(imgdata[col], cv::Point3i(col, row, 1)));
        else {
          coms_pos->second.x += col;
          coms_pos->second.y += row;
          ++coms_pos->second.z;
        }
      } // end if if ((!use_mask || *mask_value++) && data[col])
    } // end loop col
  } // end loop row

  // convert coms -> out
  out.clear();
  typename ComsMap::const_iterator coms_it = coms.begin();
  ClosestPointInMask cl;
  while (coms_it != coms.end()) { // normalize by the number of points
    _T user_idx = coms_it->first;
    cv::Point com(1.f * coms_it->second.x / coms_it->second.z,
                  1.f * coms_it->second.y / coms_it->second.z);
    if (ensure_com_on_mask) // find closest poiint on mask
      com = cl.find(img, com, user_idx, user_idx);
    if (!bbox_full(img).contains(com)) {
      printf("com %s outside of img '%s'\n",
             geometry_utils::printP2(com).c_str(), infosImage(img).c_str());
      return false;
    }
    out.insert(std::pair<_T, cv::Point>(user_idx, com));
    ++coms_it;
  }
  return true;
} // end get_all_different_values()

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline bool get_all_non_null_values_and_com_fast(const cv::Mat_<_T> & img,
                                                 std::map<_T, cv::Point> & out,
                                                 bool ignore_zeros = true,
                                                 bool ensure_com_on_mask = false,
                                                 const cv::Mat1b & mask = cv::Mat1b(),
                                                 int maxval = 255) {
  if (img.empty()) {
    out.clear();
    return true;
  }
  assert(img.isContinuous());
  std::vector<cv::Point3i> coms(maxval + 1, cv::Point3i(0, 0, 0));
  // determine if use mask
  bool use_mask = (!mask.empty());
  const uchar* mask_value = (use_mask ? mask.ptr<uchar>() : NULL);
  // iterate on image
  int cols = img.cols, rows = img.rows;
  for (int row = 0; row < rows; ++row) {
    // get the address of row
    const _T* imgdata = img.ptr(row);
    for (int col = 0; col < cols; ++col) {
      if ((!ignore_zeros || imgdata[col]) && (!use_mask || *mask_value++)) {
        cv::Point3i* curr_pt = &(coms[imgdata[col]]);
        curr_pt->x += col;
        curr_pt->y += row;
        ++curr_pt->z;
      } // end if if ((!use_mask || *mask_value++) && data[col])
    } // end loop col
  } // end loop row

  // convert coms -> out
  out.clear();
  ClosestPointInMask cl;
  // if user_idx of type _T, infinite loop with _T=uchar and maxval = 255
  int max_val_int = maxval;
  for (int user_idx = 0; user_idx <= max_val_int; ++user_idx) {
    if (!coms[user_idx].z)
      continue;
    cv::Point3i* curr_pt = &(coms[user_idx]);
    cv::Point com(1.f * curr_pt->x / curr_pt->z,
                  1.f * curr_pt->y / curr_pt->z);
    if (ensure_com_on_mask) // find closest poiint on mask
      com = cl.find(img, com, (_T) user_idx, (_T) user_idx);
    if (!bbox_full(img).contains(com)) {
      printf("com %s outside of img '%s'\n",
             geometry_utils::printP2(com).c_str(), infosImage(img).c_str());
      return false;
    }
    out.insert(std::pair<_T, cv::Point>(user_idx, com));
  } // end loop i
  return true;
} // end get_all_different_values()

////////////////////////////////////////////////////////////////////////////////

template<class _T>
inline bool detect_end_points(const cv::Mat_<_T> & img,
                              std::vector<cv::Point> & end_pts) {
  // printf("detect()\n");
  end_pts.clear();
  if (img.empty()) {
    maggiePrint("EndFinder: empty input image.");
    return true;
  }
  int rows = img.rows, rowsm = img.rows - 1,
      cols = img.cols, colsm = img.cols - 1;
  //  const _T *up_row = NULL,
  //      *curr_row = (rows > 0 ? img.ptr(0) : NULL),
  //      *down_row = (rows > 1 ? img.ptr(1) : NULL);
  for (int row = 0; row < rows; ++row) {
    bool up_ok = row > 0, down_ok = row < rowsm;
    const _T *up_row = (up_ok ? img.ptr(row-1) : NULL),
        *curr_row = img.ptr(row),
        *down_row = (down_ok ? img.ptr(row+1) : NULL);
    for (int col = 0; col < cols; ++col) {
      // printf("img(%i, %i) = %i\n", col, row, (int) curr_row[col]);
      if (!curr_row[col])
        continue;
      bool left_ok = col > 0, right_ok = col < colsm;
      // count nb of neighbours
      int neigh_nb = 0;
      if (left_ok && curr_row[col-1] && (++neigh_nb) >= 2)
        continue;
      if (right_ok && curr_row[col+1] && (++neigh_nb) >= 2)
        continue;
      if (up_ok && up_row[col] && (++neigh_nb) >= 2)
        continue;
      if (down_ok && down_row[col] && (++neigh_nb) >= 2)
        continue;
      // C8
      if (left_ok && up_ok && up_row[col-1] && (++neigh_nb) >= 2)
        continue;
      if (right_ok && up_ok && up_row[col+1] && (++neigh_nb) >= 2)
        continue;
      if (left_ok && down_ok && down_row[col-1] && (++neigh_nb) >= 2)
        continue;
      if (right_ok && down_ok && down_row[col+1] && (++neigh_nb) >= 2)
        continue;
      // found an end!
      // printf("adding (%i, %i)\n", col, row);
      end_pts.push_back(cv::Point(col, row));
    } // end loop col
    // shifting data pointers
    //      printf("Shifting\n");
    //      up_row = curr_row;
    //      curr_row = down_row;
    //      if (down_ok)
    //        down_row = img.ptr(row+1);
  } // end loop row
  return true;
} // end detect_end_points()

////////////////////////////////////////////////////////////////////////////////

} // end namespace image_utils

#endif // CONTENT_PROCESSING_H
