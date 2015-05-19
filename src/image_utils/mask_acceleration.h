/*!
  \file        mask_acceleration.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/5/10

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

\todo Description of the file
 */
#ifndef MASK_ACCELERATION_H
#define MASK_ACCELERATION_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//AD
#include <image_utils/draw_arrow.h>
#include <geom/geometry_utils.h>
#include <time/timer.h>
// trying to solve the problem of the white pixels at edge
// cf https://stackoverflow.com/questions/20418416/how-to-fill-contours-that-touch-the-image-border

class MaskAcceleration {
public:
  static const unsigned int MIN_CONTOUR_SIZE = 10;
  static const double DEFAULT_MIN_ACC_NORM = 10;
  static const double DEFAULT_ACC_DRAWING_SCALE = 1; // pixels per meters
  struct PixelAcceleration {
    cv::Point origin;
    double orien; //! radians
    double norm; //! meters
    std::string to_string() const {
      std::ostringstream out;
      out << "(" << origin.x << "," << origin.y << "), orien:" << orien
          << ", norm:" << norm;
      return out.str();
    }
  };

  MaskAcceleration() {}

  //////////////////////////////////////////////////////////////////////////////

  bool from_mask(const cv::Mat1b & mask,
                 const cv::Point & offset = cv::Point(),
                 double min_acc_norm = DEFAULT_MIN_ACC_NORM) {
    printf("MaskAcceleration::from_mask(mask size:%ix%i)\n", mask.cols, mask.rows);
    // copy to prev
    _prev_simplified_contour = _simplified_contour;
    // reset variables
    _contours.clear();
    _concat_contour.clear();
    _simplified_contour.clear();
    _pixel_accelerations.clear();

    if (mask.empty()) {
      printf("MaskAcceleration:empty mask!\n");
      return false;
    }

    // find contours of user of interest
    static const  unsigned int EDGE_OFFSET = 1; // pixels
    _offset = cv::Point(offset.x - EDGE_OFFSET, offset.y - EDGE_OFFSET); // do not forget 1 pixel margin added to _user_mask_extended
    _user_mask_extended.create(mask.rows + 2 * EDGE_OFFSET,
                               mask.cols + 2 * EDGE_OFFSET);
    _user_mask_extended.setTo(0);
    cv::Rect roi(EDGE_OFFSET, EDGE_OFFSET, mask.cols, mask.rows);
    cv::Mat1b user_mask_roi = _user_mask_extended(roi);
    user_mask_roi = (mask != 0);
    //cv::imwrite("/tmp/user_mask_extended.png", _user_mask_extended);
    //cv::imshow("user_mask_extended", _user_mask_extended); cv::waitKey(0);
    cv::findContours(_user_mask_extended, _contours, // a vector of contours
                     CV_RETR_EXTERNAL, // retrieve the external contours
                     //CV_RETR_LIST, // retrieve the external contours
                     // CV_CHAIN_APPROX_NONE // all pixels of each contours
                     CV_CHAIN_APPROX_TC89_L1,
                     _offset);

    if (_contours.empty()) { // no contours -> do nothing
      printf("MaskAcceleration:_contours empty!\n");
      return false;
    }

    // concatenate all contours
    // first determine the size of the concat_contour
    unsigned int concat_contour_size = 0, ncontours = _contours.size();
    for (unsigned int contour_idx = 0; contour_idx < ncontours; ++contour_idx)
      concat_contour_size += _contours[contour_idx].size();
    _concat_contour.clear();
    _concat_contour.reserve(concat_contour_size);
    // then really add points
    for (unsigned int contour_idx = 0; contour_idx < ncontours; ++contour_idx) {
      // do not insert if contour to small
      //if (_contours[contour_idx].size() < MIN_CONTOUR_SIZE)
      //continue;
      _concat_contour.insert
          (_concat_contour.end(),
           _contours[contour_idx].begin(), _contours[contour_idx].end());
    }
    if (_concat_contour.empty()) { // no contours -> do nothing
      printf("MaskAcceleration:_concat_contour empty!\n");
      return false;
    }
    //  printf("MaskAcceleration:concat_contour of size %i\n",
    //         _concat_contour.size());

    // simplify contour
    //cv::approxPolyDP(cv::Mat(concat_contour), simplified_contour, 5, false);
    _simplified_contour = _concat_contour;
    simplify_contour(_simplified_contour);
    unsigned int contour_size = _simplified_contour.size();
    if (contour_size == 0) {
      printf("MaskAcceleration:_simplified_contour empty!\n");
      return false;
    }
    //  printf("MaskAcceleration:_simplified_contour size %i -> %i\n",
    //         _concat_contour.size(), contour_size);

    // do not determine accelerations if prev_simplified_contour empty
    // (initialization)
    if (_prev_simplified_contour.empty()) {
      return true; // not a failure
    }

    // determine accelerations
    for (unsigned int curr_pt_idx = 0; curr_pt_idx < contour_size; ++curr_pt_idx) {
      cv::Point curr_pt = _simplified_contour[curr_pt_idx];
      double curr_depth = 1;//depth(*curr_pt);
      // find the closest point from the prev_simplified_contour
      cv::Point closest_prev_pt;
      double min_dist_sq;
      if (!geometry_utils::distance_point_polygon_squared
          (curr_pt, _prev_simplified_contour, closest_prev_pt, min_dist_sq, false))
        continue;

      // now that we have the closest point from prev_simplified_contour
      // add the acceleration if it is strong enough
      double curr_acc = sqrt(min_dist_sq) * curr_depth;
      if (curr_acc < min_acc_norm)
        continue;
      PixelAcceleration new_acc;
      new_acc.origin =  curr_pt;
      new_acc.orien = geometry_utils::oriented_angle_of_a_vector
                      (curr_pt - closest_prev_pt);
      new_acc.norm = curr_acc;
      _pixel_accelerations.push_back(new_acc);
    } // end loop curr_pt_idx

    return true; // success
  } // end from_mask();

  //////////////////////////////////////////////////////////////////////////////

  static inline void draw_points_with_offset(cv::Mat3b & img,
                                             const std::vector<cv::Point> & pts,
                                             const cv::Point & offset,
                                             const cv::Scalar edge_color,
                                             const unsigned int edge_thickness,
                                             const cv::Scalar vertex_color,
                                             const unsigned int vertex_thickness) {
    unsigned int npts = pts.size();
    if (edge_thickness)
      for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
        cv::line(img, pts[pt_idx] + offset, pts[(pt_idx+1)%npts] + offset,
            edge_color, edge_thickness);
    if (vertex_thickness)
      for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx)
        cv::circle(img, pts[pt_idx] + offset, vertex_thickness, vertex_color, -1);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void draw_illus(cv::Mat3b & img_out,
                         double acc_drawing_scale = DEFAULT_ACC_DRAWING_SCALE,
                         const cv::Scalar bg_color = cv::Scalar()) const {
    static const unsigned int ILLUS_BORDER = 50; // pixels
    cv::Point final_offset(-_offset.x + ILLUS_BORDER, -_offset.y + ILLUS_BORDER);
    // clear img_out
    img_out.create(_user_mask_extended.size() + cv::Size(2 * ILLUS_BORDER, 2 * ILLUS_BORDER));
    img_out.setTo(bg_color);
    draw_points_with_offset(img_out, _prev_simplified_contour,
                            final_offset, CV_RGB(0, 100, 0), 1, CV_RGB(0, 100, 0), 3);
    draw_points_with_offset(img_out, _concat_contour,
                            final_offset, CV_RGB(255, 255, 255), 0, CV_RGB(255, 255, 255), 3);
    draw_points_with_offset(img_out, _simplified_contour,
                            final_offset, CV_RGB(0, 255, 0), 1, CV_RGB(0, 255, 0), 3);

    //  printf("_simplified_contour:%i pts\n", _simplified_contour.size());
    //  printf("_pixel_accelerations:%i pts\n", _pixel_accelerations.size());
    // draw accelerations
    if (_pixel_accelerations.empty())
      return;
    const PixelAcceleration* curr_acc = &(_pixel_accelerations[0]);
    for (unsigned int acc_idx = 0; acc_idx < _pixel_accelerations.size(); ++acc_idx) {
      cv::Point end =  curr_acc->origin + cv::Point(acc_drawing_scale * curr_acc->norm * cos(curr_acc->orien),
                                                    acc_drawing_scale * curr_acc->norm * sin(curr_acc->orien));
      image_utils::draw_arrow (img_out, curr_acc->origin + final_offset,
                               end + final_offset, CV_RGB(255, 0, 0), 2);
      ++ curr_acc;
    } // end loop acc_idx
  } // end draw_img_out();

  //////////////////////////////////////////////////////////////////////////////

  inline const std::vector<PixelAcceleration> & get_pixel_accelerations() const {
    return _pixel_accelerations;
  }

  //////////////////////////////////////////////////////////////////////////////

  inline cv::Point get_centroid() const {
    return _offset + geometry_utils::barycenter(_simplified_contour);
  }

protected:
  //////////////////////////////////////////////////////////////////////////////

  static void simplify_contour(std::vector<cv::Point> & contour) {
    for (int pt_idx = 0; pt_idx < (int) contour.size() - 2; ++pt_idx) {
      bool need_delete = false;
      // minimum distance
      double dist = geometry_utils::distance_points_squared(contour[pt_idx], contour[pt_idx + 1]);
      if (dist < 15 * 15)
        need_delete = true;

      if (!need_delete){
        // open angle
        double angle = geometry_utils::absolute_angle_between_three_points
                       (contour[pt_idx], contour[pt_idx + 1], contour[pt_idx + 2]);
        if (fabs(angle - M_PI) < M_PI / 6)
          need_delete = true;
      }

      //printf("pt %i=(%i, %i), pt %i=(%i, %i), dist:%g => delete:%i\n",
      //       pt_idx, contour[pt_idx].x, contour[pt_idx].y,
      //       pt_idx+1, contour[pt_idx+1].x, contour[pt_idx+1].y, dist, need_delete);
      if (need_delete) { // remove pt_idx + 1
        contour.erase(contour.begin() + pt_idx + 1);
        pt_idx--; // rewind
      }
    } // end loop pt
  } // end simplify_contour();

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  cv::Point _offset;
  std::vector<std::vector<cv::Point> > _contours;
  std::vector<cv::Point> _concat_contour;
  std::vector<cv::Point> _simplified_contour;
  std::vector<cv::Point> _prev_simplified_contour;
  cv::Mat1b _user_mask_extended;
  std::vector<PixelAcceleration> _pixel_accelerations;
}; // end class MaskAcceleration


#endif // MASK_ACCELERATION_H

