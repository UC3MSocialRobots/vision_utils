#ifndef COSTMAP_UTILS_H
#define COSTMAP_UTILS_H

/*!
 * In general, the 4 corners of a cell are (x +/- cell_width / 2, y +/- cell_height / 2)
 *
 * For instance,
 * if cell_width = cell_height = 0.1 ,
 * the cell with point (x: 0.05, y: 0.05, z: 0.0) and
 * has corners (0, 0), (0, 0.1), (0.1, 0), (0.1, 0.1)
 */

#include <nav_msgs/GridCells.h>
#include "vision_utils/odom_utils.h"
#include "vision_utils/hausdorff_distances.h"

namespace vision_utils {
//cut:point_factory
//! a simple factory to make a geometry_msgs::Point with one function
inline geometry_msgs::Point point_factory(const double & x = 0,
                                          const double & y = 0,
                                          const double & z = 0) {
  geometry_msgs::Point ans;
  ans.x = x;
  ans.y = y;
  ans.z = z;
  return ans;
} // end point_factory()

////////////////////////////////////////////////////////////////////////////////
//cut:costmap_to_string
std::string costmap_to_string(const nav_msgs::GridCells & costmap) {
  std::ostringstream ans;
  ans << "w:" << costmap.cell_width << ", h:" << costmap.cell_height << ", ";
  ans << costmap.cells.size() << " cells: ";
  for (unsigned int cell_idx = 0; cell_idx < costmap.cells.size(); ++cell_idx)
    ans << "(" << costmap.cells[cell_idx].x
        << ", " << costmap.cells[cell_idx].y << "); ";
  return ans.str();
} // end costmap_to_string()

////////////////////////////////////////////////////////////////////////////////
//cut:is_point_in_costmap
/*!
 * \brief is_point_in_costmap
 * \return true fif \arg pt is in one of the cells of \arg costmap
 */
template<class _Pt2>
inline bool is_point_in_costmap(const _Pt2 & pt,
                                const nav_msgs::GridCells & costmap) {
  double halfwidth = costmap.cell_width / 2;
  double halfheight = costmap.cell_height / 2;
  for (unsigned int cell_idx = 0; cell_idx < costmap.cells.size(); ++cell_idx) {
    //const geometry_msgs::Point & curr_cell = costmap.cells[cell_idx];
    if (fabs(pt.x - costmap.cells[cell_idx].x ) <= halfwidth
        && fabs(pt.y - costmap.cells[cell_idx].y ) <= halfheight) {
      //      //printf("Pt (%g, %g) is within the cell of index %i:(%g, %g)+/-(%g, %g)",
      //               pt.x, pt.y, cell_idx,
      //               costmap.cells[cell_idx].x, costmap.cells[cell_idx].y,
      //               costmap.cell_width / 2, costmap.cell_height/ 2);
      return true;
    }
  } // end loop cell_idx
  return false;
} // end if is_point_in_costmap()

////////////////////////////////////////////////////////////////////////////////
//cut:closest_center_with_costmap_dims
/*!
 * Gives the center of a cell that would be the closest from a query point
 * (even if this cell is actually not part of the map).
 * \param pt
 * \param map
 */
template<class _Pt2>
inline _Pt2 closest_center_with_costmap_dims(const _Pt2 & pt,
                                             const nav_msgs::GridCells & map) {
  _Pt2 ans;
  ans.x = map.cell_width * floor(pt.x / map.cell_width) + map.cell_width / 2;
  ans.y = map.cell_height * floor(pt.y / map.cell_height) + map.cell_height / 2;
  return ans;
} // end closest_center_with_costmap_dims()

////////////////////////////////////////////////////////////////////////////////
//cut:costmap_to_polygon_list
/*!
 * \brief costmap_to_polygon_list
 * \param costmap
 * \param out
 *    points in this order:
 *  y ^
 *    |  3   2
 *    |    x
 *    |  0   1
 *   0+---------> x
 */
template<class _Pt3>
void costmap_to_polygon_list(const nav_msgs::GridCells & costmap,
                             std::vector<_Pt3> & out) {
  double halfwidth = costmap.cell_width / 2;
  double halfheight = costmap.cell_height / 2;
  out.clear();
  out.reserve(costmap.cells.size() * 4);
  _Pt3 corner;
  for (unsigned int cell_idx = 0; cell_idx < costmap.cells.size(); ++cell_idx) {
    // A B
    // C D
    // add C
    corner.x = costmap.cells[cell_idx].x - halfwidth;
    corner.y = costmap.cells[cell_idx].y - halfheight;
    corner.z = costmap.cells[cell_idx].z;
    out.push_back(corner);
    // add D
    corner.x = costmap.cells[cell_idx].x + halfwidth;
    out.push_back(corner);
    // add B
    corner.y = costmap.cells[cell_idx].y + halfheight;
    out.push_back(corner);
    // add A
    corner.x = costmap.cells[cell_idx].x - halfwidth;
    out.push_back(corner);
  } // end loop cell_idx
} // end costmap_to_string()

////////////////////////////////////////////////////////////////////////////////
//cut:add_point_to_costmap
/*!
 * Add a point in a costmap, only if it is not included in it.
 * Do not modify the costmap otherwise.
 * \param pt
 *        a query point
 * \param costmap
 *         the costmap that needs to be modified
 * \return true if a new cell was effectively added,
 *         false if pt was actually already included in the costmap
 */
template<class _Pt2>
inline bool add_point_to_costmap(const _Pt2 & pt,
                                 nav_msgs::GridCells & costmap) {
  if (is_point_in_costmap(pt, costmap))
    return false;
  costmap.cells.push_back(closest_center_with_costmap_dims
                          (point_factory(pt.x, pt.y), costmap));
  return true;
} // end if add_point_to_costmap()

////////////////////////////////////////////////////////////////////////////////
//cut:remove_point_from_costmap
/*!
 * remove a point in a costmap, only if it is not included in it.
 * Do not modify the costmap otherwise.
 * \param pt
 *        a query point
 * \param costmap
 *         the costmap that needs to be modified
 * \return true if a new cell was effectively removed,
 *         false if pt was not in the costmapz
 */
template<class _Pt2>
inline bool remove_point_from_costmap(const _Pt2 & pt,
                                      nav_msgs::GridCells & costmap) {
  geometry_msgs::Point center = closest_center_with_costmap_dims
                                (point_factory(pt.x, pt.y), costmap);
  bool was_pt_erased = false;
  for (unsigned int pt_idx = 0; pt_idx < costmap.cells.size(); ++pt_idx) {

    if (center.x == costmap.cells[pt_idx].x &&
        center.y == costmap.cells[pt_idx].y) {
      costmap.cells.erase(costmap.cells.begin() + pt_idx);
      --pt_idx;
      was_pt_erased = true;
    }
  }
  return was_pt_erased;
} // end if remove_point_from_costmap()

////////////////////////////////////////////////////////////////////////////////
//cut:toggle_point_in_costmap
/*!
 * Add a point if it is not in the costmap, remove it otherwise
 * \param pt
 *        a query point
 * \param costmap
 *         the costmap that needs to be modified
 * \return true if \arg pt is in the map after modification
 *         (ie it was not in the map before),
 *         false otherwise
 */
template<class _Pt2>
inline bool toggle_point_in_costmap(const _Pt2 & pt,
                                    nav_msgs::GridCells & costmap) {

  if (is_point_in_costmap(pt, costmap)) {
    remove_point_from_costmap(pt, costmap);
    return false;
  }
  add_point_to_costmap(pt, costmap);
  return true;
} // end if add_point_to_costmap()



////////////////////////////////////////////////////////////////////////////////
//cut:is_trajectory_collision_free
template<class _Pt2>
inline bool is_trajectory_collision_free(const _Pt2 & start_pos, const float & start_yaw,
                                         const nav_msgs::GridCells & costmap,
                                         const float & time_end, const float & dt,
                                         const float & vel_lin, const float & vel_ang,
                                         std::vector<_Pt2> & traj_buffer)
{
  make_trajectory(vel_lin, vel_ang, traj_buffer, time_end, dt,
                              start_pos.x, start_pos.y, start_yaw);
  for (unsigned int traj_pt_idx = 0; traj_pt_idx < traj_buffer.size(); ++traj_pt_idx) {
    if (is_point_in_costmap<_Pt2>(traj_buffer[traj_pt_idx], costmap))
      return false;
  } // end loop traj_pt_idx
  return true;
} // end is_trajectory_collision_free()

////////////////////////////////////////////////////////////////////////////////
//cut:trajectory_mark
/*!
 \param start_pos
 \param start_yaw
 \param goal
 \param costmap
 \param time_end
 \param dt
 \param vel_lin
 \param vel_ang
 \param traj_buffer
 \return float
    infinity if collision,
    otherwise the L2 distance in meters between the trajectory and the goal
*/
template<class _Pt2>
inline float trajectory_mark(const _Pt2 & start_pos, const float & start_yaw,
                             const _Pt2 & goal,
                             const nav_msgs::GridCells & costmap,
                             const float & time_end, const float & dt,
                             const float & vel_lin, const float & vel_ang,
                             std::vector<_Pt2> & traj_buffer) {
  // return infinity if there is a collision
  if (!is_trajectory_collision_free(start_pos, start_yaw, costmap,
                                    time_end, dt, vel_lin, vel_ang, traj_buffer))
    return std::numeric_limits<float>::infinity();

  // evaluate distance to the goal - best point from the set
  //  return dist_pt_set
  //      (goal, traj_buffer, dist_L2);

  // evaluate distance to the goal - last point
  return dist_L2(goal, traj_buffer.back());
} // end trajectory_mark()
//cut:

} // end namespace vision_utils

#endif // COSTMAP_UTILS_H
