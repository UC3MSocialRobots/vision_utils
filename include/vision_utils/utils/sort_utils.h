/*!
  \file        sort_utils.h
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

\todo Description of the file

 */

#ifndef SORT_UTILS_H
#define SORT_UTILS_H

#include <vector>
#include <sstream>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////

/*! the element #i contains the future position of the element at position #i
 *  from http://stackoverflow.com/questions/236172/how-do-i-sort-a-stdvector-by-the-values-of-a-different-stdvector
 */
class Order : public std::vector<size_t> {
public:
  //////////////////////////////////////////////////////////////////////////////

  template<class _T>
  void from_vec(std::vector<_T> & v,
                bool sort_v_after = false,
                bool ascending_order = true) {
    typedef typename std::vector<_T>::const_iterator myiter;
    std::vector<std::pair<size_t, myiter> > map;
    _size = v.size();
    map.resize(_size);
    size_t n = 0;
    for (myiter it = v.begin(); it != v.end(); ++it, ++n)
      map[n] = make_pair(n, it);
    // now sort them
    if (ascending_order)
      std::sort(map.begin(), map.end(), typename Order::ascending_ordering<_T>());
    else
      std::sort(map.begin(), map.end(), typename Order::descending_ordering<_T>());

    // copy to (*this)
    this->resize(_size);
    for (unsigned int i = 0; i < _size; ++i)
      //(*this)[i] = map[i].first;
      (*this)[map[i].first] = i;
    if (sort_v_after)
      apply_on_vec(v);
  }

  //////////////////////////////////////////////////////////////////////////////

  std::string to_string() const {
    if (this->empty())
      return "{}";
    std::ostringstream out;
    for (unsigned int i = 0; i < _size; ++i) {
      out << (*this)[i];
      if (i < _size-1)
        out << "; ";
    } // end loop i
    return out.str();
  } // end to_string()

  //////////////////////////////////////////////////////////////////////////////

  //! from http://stackoverflow.com/questions/838384/reorder-vector-using-a-vector-of-indices
  template<class Vec>
  bool apply_on_vec(Vec & v) {
    if (v.size() != size()) {
      printf("Order:apply_on_vec(): incorrect sizes:%i != %i!\n", v.size(), size());
      return false;
    }
    reorder(this->begin(), this->end(), v.begin());
    //reorder(v, *this);
    return true;
  }

private:
  //! from http://stackoverflow.com/questions/26151/template-typedefs-whats-your-work-around
  template<class _T>
  struct ascending_ordering {
    typedef typename std::vector<_T>::const_iterator myiter;
    bool operator ()(std::pair<size_t, myiter> const& a,
                     std::pair<size_t, myiter> const& b) {
      return *(a.second) < *(b.second);
    }
  };
  template<class _T>
  struct descending_ordering {
    typedef typename std::vector<_T>::const_iterator myiter;
    bool operator ()(std::pair<size_t, myiter> const& a,
                     std::pair<size_t, myiter> const& b) {
      return *(a.second) > *(b.second);
    }
  };

  //! from http://stackoverflow.com/questions/838384/reorder-vector-using-a-vector-of-indices
  template< typename order_iterator, typename value_iterator >
  void reorder( order_iterator order_begin, order_iterator order_end, value_iterator v )  {
    typedef typename std::iterator_traits< value_iterator >::value_type value_t;
    typedef typename std::iterator_traits< order_iterator >::value_type index_t;
    typedef typename std::iterator_traits< order_iterator >::difference_type diff_t;

    diff_t remaining = order_end - 1 - order_begin;
    for ( index_t s = index_t(), d; remaining > 0; ++ s ) {
      for ( d = order_begin[s]; d > s; d = order_begin[d] ) ;
      if ( d == s ) {
        -- remaining;
        value_t temp = v[s];
        while ( d = order_begin[d], d != s ) {
          std::swap( temp, v[d] );
          -- remaining;
        }
        v[s] = temp;
      }
    }
  }

  template< class T >
  void reorder(std::vector<T> &v, std::vector<size_t> const &order )  {
    for ( unsigned int s = 1, d; s < order.size(); ++ s ) {
      for ( d = order[s]; d < s; d = order[d] ) ;
      if ( d == s ) while ( d = order[d], d != s ) std::swap( v[s], v[d] );
    }
  }

  unsigned int _size;
}; // end class Order


#endif // SORT_UTILS_H
