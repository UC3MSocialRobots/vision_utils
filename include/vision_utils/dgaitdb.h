/*!
  \file        dgaitdb.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/12

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

An interface for reading ONI files from the database available at
\link http://www.cvc.uab.es/DGaitDB/Summary.html .

The user mask is unfortunately not provided in the files.
This class also computes them.
 */

#ifndef DGAITDB_H
#define DGAITDB_H

#include "vision_utils/utils/timer.h"
#include "vision_utils/oni2imgs.h"
#include "vision_utils/blob_segmenter.h"
#include "vision_utils/io.h"

class DGaitDB : public Oni2Imgs {
public:
  static const unsigned int MIN_USER_SIZE = 3000; // pixels

  DGaitDB() : _nusers(0) {}

  bool go_to_next_frame() {
    if (!Oni2Imgs::go_to_next_frame())
      return false;
    // find all objects on the ground -  do not recompute ground
    if (!_segmenter.find_all_blobs
        (_depth32f, components_pts, boundingBoxes,
         BlobSegmenter::GROUND_PLANE_FINDER, NULL, 1., 5., 1, MIN_USER_SIZE, false))
      return false;
    // paint main component
    _nusers = _segmenter.all_blobs_to_user_img
              (_depth32f.size(), components_pts, _user8);
    _has_user = true;
    if (_nusers == 0)
      return true;
    return true;
  }

  inline unsigned int nusers() const { return _nusers; }

private:
  BlobSegmenter _segmenter;
  std::vector< DisjointSets2::Comp > components_pts;
  std::vector<cv::Rect> boundingBoxes;
  unsigned int _nusers;
};

#endif // DGAITDB_H
