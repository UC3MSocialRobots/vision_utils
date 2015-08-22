/*!
  \file        cmatrix.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11

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

from
  http://www.eskimo.com/~scs/cclass/int/sx9.html
  http://www.eskimo.com/~scs/cclass/int/sx9b.html
A C-like templated matrix.
For instance, CMatrix<double>.ptr_to_data() is of type double**

As such, the matrix elements can be accesed thanks to
mat[i][j].

However, memory allocation and unallocation is automatically handled.
No need to use "new", "free()" or "delete" at any point.

*/

#ifndef CMATRIX_FOO_H
#define CMATRIX_FOO_H

#define NEW_IMPL // based on http://www.eskimo.com/~scs/cclass/int/sx9b.html

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iomanip>
#include "vision_utils/utils/string_casts.h"
#include <vector>

template<class Elem>
class CMatrix {
public:
  //! ctor with no given size
  CMatrix()
    : _data(NULL),
    #ifndef NEW_IMPL
      _data_arr(NULL),
    #endif // NEW_IMPL
      _dim1(0), _dim2(0) {
    // printf("ctor()\n");
  }

  //////////////////////////////////////////////////////////////////////////////

  //! ctor with given size
  CMatrix(const unsigned int dim1, const unsigned int dim2)
    : _data(NULL)
  #ifndef NEW_IMPL
    , _data_arr(NULL)
  #endif // NEW_IMPL
  {
    // printf("CMatrix empty ctor(%i x %i)\n", dim1, dim2);
    resize(dim1, dim2);
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! (deep) copy ctor
   * culture time: cf http://en.wikipedia.org/wiki/Rule_of_three_(C%2B%2B_programming) */
  CMatrix(const CMatrix<Elem>& other) : _data(NULL)
#ifndef NEW_IMPL
  , _data_arr(NULL)
#endif // NEW_IMPL
  {
    // printf("CMatrix deep copy constructor(%i x %i)\n", other.get_dim1(), other.get_dim2());
#ifdef NEW_IMPL
    // do the same as in = operator
    resize(other.get_dim1(), other.get_dim2());
    for (unsigned int dim1_idx = 0; dim1_idx < _dim1; ++dim1_idx)
      memcpy((*this)[dim1_idx], other[dim1_idx], _dim2 * sizeof(Elem) );
#else // not NEW_IMPL
    if (other._data == NULL) { // other empty -> nothing to do
      // printf("CMatrix copy ctor(empty mat)\n");
      return;
    }
    resize(other._dim1, other._dim2);
    std::copy(other._data_arr, other._data_arr + _dim1 * _dim2, _data_arr);
#endif // NEW_IMPL
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! = operator -> deep copy from matrix "other" to current matrix */
  CMatrix& operator=(const CMatrix<Elem>& other) {
    // printf("CMatrix = operator(%i x %i)\n", other.get_dim1(), other.get_dim2());
    resize(other.get_dim1(), other.get_dim2());
#ifdef NEW_IMPL
    for (unsigned int dim1_idx = 0; dim1_idx < _dim1; ++dim1_idx) {
      // loop version
      //      for (unsigned int dim2_idx = 0; dim2_idx < _dim2; ++dim2_idx)
      //        this[dim1_idx][dim2_idx] = other[dim1_idx][dim2_idx];

      // memcpy is faster - cf
      // http://stackoverflow.com/questions/4729046/memcpy-vs-for-loop-whats-the-proper-way-to-copy-an-array-from-a-pointer
      memcpy((*this)[dim1_idx], other[dim1_idx], _dim2 * sizeof(Elem) );
    }

#else // not NEW_IMPL
    free_data();
    if (other._data == NULL) { // other empty -> nothing to do
      // printf("CMatrix copy ctor(empty mat)\n");
      return *this;
    }
    std::copy(other._data_arr, other._data_arr + _dim1 * _dim2, _data_arr);
#endif // NEW_IMPL
    return *this;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! dtor - clean the mess
  virtual ~CMatrix() {
    // printf("CMatrix dtor(%i x %i)\n", _dim1, _dim2);
    free_data();
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! [] operator: access a row (or column, depending what convention was chosen),
   * in the matrix.
   * As such, an element in the matrix can be accessed using:
   * mat[i][j]
   */
  const Elem* operator[](const unsigned int dim1) const { return _data[dim1]; }
  Elem* operator[](const unsigned int dim1)       { return _data[dim1]; }

  //////////////////////////////////////////////////////////////////////////////

  //! resize the inner data
  bool resize(const unsigned int newdim1, const unsigned int newdim2) {
    // printf("resize(%i x %i)\n", newdim1, newdim2);
    // if one of the dims is 0, well both are
    if ((newdim1 == 0 && newdim2 != 0) ||(newdim2 == 0 && newdim1 != 0))
      return resize(0, 0);
    // if was already allocated, delete previous size if different
    if (_data != NULL && (_dim1 != newdim1 || _dim2 != newdim2))
      free_data();

    // make allocation
#ifdef NEW_IMPL
    _data = (Elem**) malloc(newdim1 * sizeof(Elem*));
    if(_data == NULL) {
      fprintf(stderr, "out of memory\n");
      return false;
    }
    for(unsigned int i = 0; i < newdim1; i++) {
      _data[i] = (Elem*) malloc(newdim2 * sizeof(Elem));
      if(_data[i] == NULL) {
        fprintf(stderr, "out of memory\n");
        return false;
      }
    } // end for i
#else // not NEW_IMPL
    _data_arr = (Elem*) malloc(newdim1 * newdim2 * sizeof(Elem));
    _data = (Elem**) malloc(newdim1 * sizeof(*_data));
    for (unsigned int i = 0; i < newdim1; ++i)
      _data[i] = &_data_arr[i * newdim2];
#endif // NEW_IMPL

    // store new dimensions
    _dim1 = newdim1;
    _dim2 = newdim2;
    return true;
  } // end resize()

  //////////////////////////////////////////////////////////////////////////////

  //! return a string version of the matrix
  std::string to_string(const int cell_width = 10) const {
    // printf("to_string()\n");
    std::ostringstream out;
    out << "(" << string_utils::cast_type_to_string<Elem>()
        << ", " << _dim1 << "x" << _dim2 << "):" << std::endl;
    for (unsigned int i1 = 0; i1 < _dim1; ++i1) {
      for (unsigned int i2 = 0; i2 < _dim2; ++i2) {
        out << std::setprecision(3) << std::setw(cell_width);
        out << _data[i1][i2];
      } // end loop i2
      if (i1 < _dim1 - 1)
        out << std::endl;
    } // end loop i1
    return out.str();
  } // end to_string()

  //////////////////////////////////////////////////////////////////////////////

  //! get pointers to inner data, as a 2-dimensional array
  Elem** ptr_to_data()       { return _data; }
  const Elem** ptr_to_data() const { return (const Elem**) _data; }

#ifndef NEW_IMPL
  //! get pointers to inner data, as a 1-dimensional array
  Elem* ptr_to_data_arr()       { return _data_arr; }
  const Elem* ptr_to_data_arr() const { return _data_arr; }
#endif // NEW_IMPL

  //////////////////////////////////////////////////////////////////////////////

  //! a fast way to set the matrix to zero
  void set_to_zero() {
#ifdef NEW_IMPL
    for(unsigned int i = 0; i < _dim1; i++)
      memset(_data[i], 0, _dim2 * sizeof(Elem));
#else // not NEW_IMPL
    memset(_data_arr, 0, _dim1 * _dim2 * sizeof(Elem));
#endif // NEW_IMPL
  }

  //////////////////////////////////////////////////////////////////////////////

  //! get the dimensions of the matrix
  inline unsigned int get_dim1() const { return _dim1; }
  inline unsigned int get_dim2() const { return _dim2; }

  //////////////////////////////////////////////////////////////////////////////

  template<class _T>
  bool from_vector(const std::vector<_T> & v,
                   const unsigned int dim1, const unsigned int dim2) {
    if (v.size() != dim1 * dim2) {
      printf("CMatrix::from_vector(): uncorrect size of the input vec, "
             "should be %i elements, is %i", dim1 * dim2, v.size());
      return false;
    }
    resize(dim1, dim2);
    int data_counter = 0;
    for (unsigned int i1 = 0; i1 < _dim1; ++i1)
      for (unsigned int i2 = 0; i2 < _dim2; ++i2)
        (*this)[i1][i2] = v[data_counter++];
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * transpose current matrix to another one
   * \param dst
   *  will be equal to the current matrix, transposed
   */
  inline void transpose_to(CMatrix<Elem> & dst) const {
    dst.resize(_dim2, _dim1);
    for (unsigned int i1 = 0; i1 < _dim1; ++i1)
      for (unsigned int i2 = 0; i2 < _dim2; ++i2)
        dst[i2][i1] = (*this)[i1][i2];
  }

protected:
  //////////////////////////////////////////////////////////////////////////////

  //! clean the inner data
  void free_data() {
    // printf("free_data()\n");
    if (_data == NULL)
      return;
    // printf("free_data(): really free %i, %i\n", _dim1, _dim2);
#ifdef NEW_IMPL
    for(unsigned int i = 0; i < _dim1; i++)
      free(_data[i]);
#else // not NEW_IMPL
    free(_data_arr);
    _data_arr = NULL;
#endif // NEW_IMPL
    free(_data);
    _data = NULL;
  } // end free_data();

  //////////////////////////////////////////////////////////////////////////////

  //! the real storage space, will be of size [dim1][dim2]
  Elem** _data;
#ifndef NEW_IMPL
  Elem *_data_arr;
#endif // NEW_IMPL
  unsigned int _dim1, _dim2;
};

#endif // CMATRIX_FOO_H
