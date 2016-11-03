/*!
  \file        median.h
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

#ifndef MEDIAN_H
#define MEDIAN_H
// std includes
#include <stdio.h> // for printf(), etc

namespace vision_utils {

/*! from http://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation/1719155#1719155
  Return the median of a sequence of numbers defined by the random
  access iterators begin and end.  The sequence must not be empty
  median is undefined for an empty set).
  The numbers must be convertible to double. */
template<class RandAccessIter>
double median(RandAccessIter begin, RandAccessIter end) {
  if(begin == end){
    printf("median():empty list!\n");
    return -1;
  }
  std::size_t size = end - begin;
  std::size_t middleIdx = size/2;
  RandAccessIter target = begin + middleIdx;
  std::nth_element(begin, target, end);

  if(size % 2 != 0){ //Odd number of elements
    return *target;
  }
  //Even number of elements
  double a = *target;
  RandAccessIter targetNeighbor= target-1;
  std::nth_element(begin, targetNeighbor, end);
  return (a+*targetNeighbor)/2.0;
}

} // end namespace vision_utils

#endif // MEDIAN_H
