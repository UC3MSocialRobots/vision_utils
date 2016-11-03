/*!
  \file        match.h
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

#ifndef MATCH_H
#define MATCH_H
// std includes
#include <vector>

namespace vision_utils {

typedef double Cost;

struct Match {
  int first;
  int second;
  Cost cost;
};
inline bool operator == (const Match& A, const Match & B) {
  return (A.first == B.first) && (A.second == B.second) && (A.cost == B.cost);
}
inline bool operator < (const Match& A, const Match& B) {
  return A.first < B.first;
}

typedef std::vector<Match> MatchList;
static const int UNASSIGNED = -1;

} // end namespace vision_utils

#endif // MATCH_H
