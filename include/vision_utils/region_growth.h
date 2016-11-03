/*!
  \file        region_growth.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/11/2
________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________
 */

#ifndef REGION_GROWTH_H
#define REGION_GROWTH_H
// std includes
#include <deque>
#include <queue>
#include <stdio.h> // for printf(), etc
#include <vector>
#include <opencv2/core/core.hpp>
#include <vision_utils/bbox_full.h>
#include <vision_utils/cmatrix.h>
#include "vision_utils/depth_image_to_vizualisation_color_image.h"
#include "vision_utils/find_top_point_centered.h"
#include "vision_utils/infosimage.h"
#include <vision_utils/nonNulPoints.h>
#include <vision_utils/printP2.h>

namespace vision_utils {

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

  //printf("region_growth()");

  unsigned int positionInQueue = 0;
  queue.clear();

  // fill seenPoints with false
  bool* seenPoints_ptr = seenPoints;
  for(int i = 0 ; i < mat.cols * mat.rows ; ++i)
    *seenPoints_ptr++ = NOT_SEEN;

  // init the queue with the seed
  queue.push_back(seed);
  int seedValue = mat(seed.y, seed.x);
  //printf("seedValue:%i", seedValue);
  // mark as seen
  seenPoints[seed.y * mat.cols + seed.x] = SEEN;

  do {
    cv::Point2i head = queue.at(positionInQueue);
    ++positionInQueue;
    //printf("Checking:(%i,%i)", head.x, head.y);
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

      //printf("Examining %i, %i", xNeighbour, yNeighbour);

      int neighbourValue = mat(yNeighbour, xNeighbour);

      //printf("Dist:%i", abs(neighbourValue - currentVal));
      if ( (abs(neighbourValue - currentVal) > maxDistWithNeighbour)
           || (abs(neighbourValue - seedValue) > maxDistWithSeedValue)) {
        continue;
      }

      //printf("adding it");

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
    if (!bbox_full(mask).contains(seed)) {
      printf("seed %s outside of mask '%s'\n",
             printP2(seed).c_str(), infosImage(mask).c_str());
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
      //printf("Checking (%i, %i)\n", col, row);
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
    //printf("PropagationTemplate: queue empty, no point was satisfying is_over()\n");
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
    //printf("nonzero:%i\n", cv::countNonZero(_queued));
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
             printP2(real_seed).c_str(), infosImage(mask).c_str());
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
    depth_image_to_vizualisation_color_image
        (queued_float_buffer, queued_illus, FULL_RGB_STRETCHED, paint_nans_in_black);
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
             printP2(seed).c_str(), infosImage(mask).c_str());
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
             printP2(end).c_str(), infosImage(mask).c_str());
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
             printP2(seed).c_str(), infosImage(mask).c_str());
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
      //printf("Checking (%i, %i)\n", col, row);
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
             printP2(final_seed).c_str(), infosImage(src).c_str());
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

      //printf("curr_row:%i, curr_row_size:%i\n", curr_row, curr_row_size);
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

} // end namespace vision_utils

#endif // REGION_GROWTH_H
