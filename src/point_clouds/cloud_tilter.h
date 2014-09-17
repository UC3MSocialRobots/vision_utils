/*!
  \file        cloud_tilter.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/11/27

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

#ifndef CLOUD_TILTER_H
#define CLOUD_TILTER_H

#include <opencv2/core/core.hpp>
#include <src/stats/stats_utils.h>

class CloudTilter {
public:
  CloudTilter() {}

  static inline double closest_angle_to_zero(const double & a1, const double & a2) {
    double best_dist = fabs(a1), ans = a1;
    for (int pi_mod = -1; pi_mod <= 1; ++pi_mod) {
      double a = a1 + pi_mod * M_PI, dist = fabs(a);
      if (dist < best_dist) {
        best_dist = dist;
        ans = a;
      }
      a = a2 + pi_mod * M_PI;
      dist = fabs(a);
      if (dist < best_dist) {
        best_dist = dist;
        ans = a;
      }
    } // end loop pi_mod
    return ans;
  }

  template<class T>
  static inline bool average_angle(const std::vector<T> & X, const std::vector<T> & Y,
                                   double & ans) {
    cv::Point2f center, e1, e2;
    if (!gaussian_pdf_ellipse(X, Y, center, e1, e2)) {
      printf("average_angle(): Error in gaussian_pdf_ellipse()\n");
      return false;
    }
    double angle1 = atan2(e1.y - center.y, e1.x - center.x);
    double angle2 = atan2(e2.y - center.y, e2.x - center.x);
    ans = closest_angle_to_zero(angle1, angle2);
    //printf("average_angle(): angle1:%g, angle2:%g => angle:%g\n", angle1, angle2, ans);
    return true;
  }

  /*!
   * A yaw is a counterclockwise rotation of $ \alpha$ about the $ z$-axis.
   * A pitch is a counterclockwise rotation of $ \beta$ about the $ y$-axis.
   * A roll is a counterclockwise rotation of $ \gamma$ about the $ x$-axis.
   */
  template<class Pt3>
  void tilt(std::vector<Pt3> & cloud,
            const double yaw_a, const double pitch_b, const double roll_g) {
    // cf http://planning.cs.uiuc.edu/node102.html
    double ca = cos(yaw_a), cb = cos(pitch_b), cg = cos(roll_g),
         sa = sin(yaw_a), sb = sin(pitch_b), sg = sin(roll_g);
    double a11 = ca * cb,
        a12 = ca * sb * sg - sa * cg,
        a13 = ca * sb * cg + sa * sg,
        a21 = sa * cb,
        a22 = sa * sb * sg + ca * cg,
        a23 = sa * sb * cg - ca * sg,
        a31 = -sb,
        a32 = cb * sg,
        a33 = cb * cg;
    unsigned int npts = cloud.size();
    for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
      Pt3* pt = &(cloud[pt_idx]);
      double nx = a11 * pt->x + a12 * pt->y + a13 * pt->z,
          ny = a21 * pt->x + a22 * pt->y + a23 * pt->z,
          nz = a31 * pt->x + a32 * pt->y + a33 * pt->z;
      pt->x = nx;
      pt->y = ny;
      pt->z = nz;
    } // end loop pt_idx
  } // end tilt()

  //////////////////////////////////////////////////////////////////////////////

  template<class Pt3>
  bool learn_angles(const std::vector<Pt3> & cloud,
                    double max_tilt = .4) {
    // transform into vectors x, y, z
    unsigned int npts = cloud.size();
    X.resize(npts);
    Y.resize(npts);
    Z.resize(npts);
    for (unsigned int pt_idx = 0; pt_idx < npts; ++pt_idx) {
      const Pt3* pt = &(cloud[pt_idx]);
      X[pt_idx] = pt->x;
      Y[pt_idx] = pt->y;
      Z[pt_idx] = pt->z;
    }
    // find average yaw, pitch, roll
    if (!average_angle(X, Y, _yaw)
        || !average_angle(X, Z, _pitch)
        || !average_angle(Y, Z, _roll)) {
      _yaw = 0; _pitch = 0; _roll = 0;
      return false;
    }
    // do not apply big rotations
    //printf("yaw:%g, pitch:%g, roll:%g\n", yaw, pitch, roll);
    if (fabs(_yaw) > max_tilt)
      _yaw = 0;
    if (fabs(_pitch) > max_tilt)
      _pitch = 0;
    if (fabs(_roll) > max_tilt)
      _roll = 0;
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  template<class Pt3>
  void apply_learnt_angles(std::vector<Pt3> & cloud) {
    tilt(cloud, -_yaw, -_pitch, -_roll);
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * Apply a small rotation in each axis such as the cloud
   * is parallel to these axis.
   * We first fit an ellipse to the projection of the cloud on each XY, XZ, YZ plane,
   * then find its tilt angle.
   * \param cloud
   *    modified on place
   * \param max_tilt
   *    the max angle that we can tilt, in radians
   * \return true if success, false otherwise (the cloud is then unchanged)
   */
  template<class Pt3>
  bool straighten(std::vector<Pt3> & cloud,
                  double max_tilt = .4) {
    if (!learn_angles(cloud, max_tilt))
      return false;
    //tilt(cloud, -_yaw, 0, 0);
    //tilt(cloud, 0, -_pitch, 0);
    //tilt(cloud, 0, 0, -_roll);
    tilt(cloud, -_yaw, -_pitch, -_roll);
    return true;
  } // end straighten()

private:
  std::vector<double> X, Y, Z;
  double _yaw, _pitch, _roll;
}; // end class CloudTilter

#endif // CLOUD_TILTER_H
