#ifndef DISJOINTSETS2_H
#define DISJOINTSETS2_H

/***************************************************************************//**
 * \class DisjointSets
 *
 * \brief Disjoint Set Data Structure
 * Implementaton is as described in http://en.wikipedia.org/wiki/Disjoint-set_data_structure
 *
 * cf "Data structures and algorithms for disjoint set union problems"
 * of Zvi Galil and Giuseppe F. Italiano
 *
 * \author Arnaud Ramey ( arnaud.ramey@m4x.org )
 *
 * \date May 2009
 *******************************************************************************/

#include <iostream>
#include <iomanip>      // std::setw
#include <opencv2/core/core.hpp>
#include <vision_utils/comp_labeller_interface.h>

namespace vision_utils {

class DisjointSets2 : public CompLabellerInterface {
public:
  typedef int CompIndex;
  typedef uchar CompRank;
  typedef int CompSize;

  static const CompIndex NO_NODE = -1;

  DisjointSets2(const CompIndex & new_count = 0) {
    //printf("DisjointSets2 ctor");
    roots = NULL;
    ranks = NULL;
    sizes = NULL;
    count = 0;
    if (new_count != 0)
      resize(new_count);
  }
  DisjointSets2(cv::Mat1b & img){
    //printf("DisjointSets2 ctor");
    roots = NULL;
    ranks = NULL;
    sizes = NULL;
    count = 0;
    process_image(img);
  }
  ~DisjointSets2(){
    //printf("DisjointSets2 dtor");
    if (roots != NULL)
      delete[] roots;
    if (ranks != NULL)
      delete[] ranks;
    if (sizes != NULL)
      delete[] sizes;
  }

  //! \see CompLabellerInterface::process_image()
  void process_image(cv::Mat1b & img) {
    //printf("process_image()");

    if (img.empty()) {
      resize(0);
      return;
    }

    if (img.isContinuous() == false)
      throw std::invalid_argument("process_image() only works with continuous images");

    // resizem but with no initial filling
    resize(img.cols * img.rows);

#ifdef TIMER_ON
    timer.reset();
#endif // TIMER_ON

    // make the disjoint sets loop
    nb_comp = 0;
    //bool is_left_non_null, is_up_non_null;
    CompIndex idx_current = 0, idx_left = -1, idx_up = -img.cols;
    CompIndex* roots_ptr = roots;

    for (CompIndex row = 0; row < img.rows; ++row) {
      // get the address of row
      uchar* data = img.ptr<uchar>(row);
      for (CompIndex col = 0; col < img.cols; ++col) {
        if (*data++ != 0) { // pixel non black
          // init roots
          //*roots_ptr = img.cols * y + x;
          *roots_ptr = idx_current;
          ++nb_comp;

          // check neighbours
          //                bool is_left_non_null = (x > 0 && roots[idx_left] != NO_NODE);
          //                bool is_up_non_null = (y > 0 && roots[idx_up] != NO_NODE);

          if (col > 0 && roots[idx_left] != NO_NODE) { // left non null
            if (row > 0 && roots[idx_up] != NO_NODE) { // up non-null
              //printf("%i: case 1", idx_current);
              //idx_current_father =
              Union(FindSet(idx_left),
                    //idx_current_father
                    Union(FindSet(idx_up), idx_current));
            } // end up non-null

            else { // up null
              //printf("%i: case 2", idx_current);
              //idx_current_father =
              Union(FindSet(idx_left), idx_current);
            } // end up null
          }

          else { // left null
            if (row > 0 && roots[idx_up] != NO_NODE) { // up non-null
              //printf("%i: case 3", idx_current);
              //idx_current_father =
              Union(FindSet(idx_up), idx_current);
            } // end up non-null

            else { // up null
              //printf("%i: case 4", idx_current);
              //idx_current_father = idx_current;
            } // end up null
          } // end left null

          //display(img.cols);
        } // end pixel non black

        else { // pixel black
          *roots_ptr = NO_NODE;
        } // end pixel non black

        //++img_it;
        ++idx_left;
        ++idx_up;
        ++idx_current;
        ++roots_ptr;
      } // end loop col
    } // end loop row

#ifdef TIMER_ON
    timer.printTime("process_image()");
#endif // TIMER_ON


    //display(img.cols);
  }

  //! \see CompLabellerInterface::get_connected_components()
  void get_connected_components(const int cols,
                                std::vector< Comp > & components_pts,
                                std::vector<cv::Rect> & boundingBoxes) {
    //printf("get_connected_components()");

#ifdef TIMER_ON
    timer.reset();
#endif // TIMER_ON

    // resize the answers to the number of components
    components_pts.clear();
    boundingBoxes.clear();
    if (cols == 0)
      return;
    components_pts.reserve( nb_comp );
    boundingBoxes.resize( nb_comp );

    // here we maintain the pos of each component into the vector
    short pos_map[count];
    bzero(pos_map, count * sizeof(short));

    // here we maintain where we are in the writing of each component
    //CompSize position_in_component[nb_comp];
    //bzero(position_in_component, count * sizeof(CompSize));

    // iterate on the roots to collect the answers
    CompIndex* roots_ptr = roots;
    cv::Rect* father_bbox;
    bool father_first_time = false;
    int father_vector_pos;
    int old;
    CompIndex father_idx;

#ifdef TIMER_ON
    timer.printTime("get_connected_components() init");
    timer.reset();
#endif // TIMER_ON

    for(CompIndex y = 0 ; y < count / cols ; ++y) {
      for(CompIndex x = 0 ; x < cols ; ++x) {
        if (*roots_ptr == NO_NODE) { // empty node
          ++roots_ptr;
          continue;
        }

        father_idx = FindSet(*roots_ptr);
        father_first_time = (pos_map[father_idx] == 0);

        if (father_first_time) { // first time we see this father
          // it will go at the end of the vector
          father_vector_pos = components_pts.size();
          // add his position into the map
          pos_map[father_idx] = father_vector_pos + 1;
          // add the new vector
          components_pts.push_back( Comp() );
          // make the size allocation
          components_pts.back().reserve( sizes[father_idx] );
        }
        else
          father_vector_pos = pos_map[father_idx] - 1;
        // add the point into the good pos in the vector
        components_pts[ father_vector_pos ].push_back( cv::Point(x, y) );

        // maintain the bbox
        father_bbox = &boundingBoxes[ father_vector_pos ];

        if (father_first_time) {
          // init the bounding rect
          father_bbox->x = x;
          father_bbox->y = y;
          father_bbox->width = 1;
          father_bbox->height = 1;
        }
        else {
          // update the bbox
          old = father_bbox->x;
          if (x < father_bbox->x)
            father_bbox->x = x;
          father_bbox->width = std::max(old + father_bbox->width, x + 1)
              - father_bbox->x;

          old = father_bbox->y;
          if (y < father_bbox->y)
            father_bbox->y = y;
          father_bbox->height = std::max(old + father_bbox->height, y + 1)
              - father_bbox->y;
        }

        ++roots_ptr;
      } // end loop x
    } // end loop y

#ifdef TIMER_ON
    timer.printTime("get_connected_components()");
#endif // TIMER_ON

    //display(cols);
  }

  /*!
     * \brief   computes the biggest conencted component
     *
     * \param   img the monochrome image
     * \param   rep the vector which will contain the results
     */
  void biggestComponent_vector(const int cols,
                               Comp & rep) {
    rep.clear();

    // if biggest comp is empty, do not iterate !
    if (biggest_comp_size == 0)
      return;

    // iterate on the roots to collect the answers
    CompIndex* roots_ptr = roots;
    rep.reserve( biggest_comp_size );

    for(CompIndex y = 0 ; y < count / cols ; ++y) {
      for(CompIndex x = 0 ; x < cols ; ++x) {
        if ( *roots_ptr != NO_NODE
             && FindSet(*roots_ptr) == biggest_comp_idx )
          rep.push_back( cv::Point(x, y) );

        ++roots_ptr;
      } // end loop x
    } // end loop y
  }

  /*!
     * \brief   computes the biggest conencted component
     *
     * \param   img the monochrome image
     * \param   imgRep a monochrome image, of the same size or greater than img
     */
  void biggestComponent_image(const int cols, cv::Mat1b & imgRep) {
    imgRep = 0;

    // if biggest comp is empty, do not iterate !
    if (biggest_comp_size == 0)
      return;

    // iterate on the roots to collect the answers
    CompIndex* roots_ptr = roots;
    for(CompIndex row = 0 ; row < count / cols ; ++row) {
      // get the address of row
      uchar* data = imgRep.ptr<uchar>(row);
      for(CompIndex col = 0 ; col < cols ; ++col) {
        if ( *roots_ptr != NO_NODE
             && FindSet(*roots_ptr) == biggest_comp_idx )
          *data = 255;
        ++data;
        ++roots_ptr;
      } // end loop col
    } // end loop row
  }

  /*!
     * \brief   computess the center of the biggest connected component of the image
     *
     * \param   img the monochrome image
     * \return  the cv::Point corresponding to the center
     */
  cv::Point centroidOfMonochromeImage(const int cols) {
    // iterate on the roots to collect the answers
    CompIndex* roots_ptr = roots;
    double x_tot = 0, y_tot = 0;

    for(CompIndex row = 0 ; row < count / cols ; ++row) {
      for(CompIndex col = 0 ; col < cols ; ++col) {
        if ( *roots_ptr != NO_NODE
             && FindSet(*roots_ptr) == biggest_comp_idx ) {
          x_tot += col;
          y_tot += row;
        }
        ++roots_ptr;
      } // end loop row
    } // end loop col

    return cv::Point(x_tot / biggest_comp_size, y_tot / biggest_comp_size);
  }

  //! for display
  void display(CompIndex width = 10) {

    // display nodes
    CompIndex* nodes_ptr = roots;
    std::cout << "[0]\t";
    for(CompIndex roots_idx = 0 ; roots_idx < count ; ++roots_idx) {
      if (*nodes_ptr == NO_NODE)
        std::cout << "   .";
      else
        std::cout << std::setw(4) << *nodes_ptr;
      ++nodes_ptr;
      if ((roots_idx + 1) % width == 0)
        std::cout << std::endl << "[" << roots_idx + 1 << "]\t";
    } // end loop roots_idx
    std::cout << std::endl << std::endl;

    // display ranks
    std::cout << "Nb comp:" << nb_comp << std::endl;
    CompRank* ranks_ptr = ranks;
    CompIndex* roots_ptr = roots;
    CompSize* sizes_ptr = sizes;
    for(CompRank rank_idx = 0 ; rank_idx < count ;) {
      //printf("rank_idx:%i, ranks_ptr:%i", rank_idx, *ranks_ptr);
      if (rank_idx == *roots_ptr) { // root of its comp
        std::cout << "Comp " << (int) rank_idx
                  << " : rank=" << (int) *ranks_ptr
                  << ", sizes:" << *sizes_ptr
                  << std::endl;
      }
      ++ranks_ptr;
      ++roots_ptr;
      ++sizes_ptr;
      ++rank_idx;
    } // end loop rank_idx

    // display biggest_comp
    if (biggest_comp_size > 0) {
      std::cout << "Biggest comp:" << biggest_comp_idx
                << " (size:" << biggest_comp_size << ")" << std::endl;
    }
  }

  //! \return the nb of components
  inline CompIndex nb_components() const { return nb_comp; }

  //! the different nodes (their pointer to their root)
  CompIndex* roots;

  /*!
     \fn FindSet
     \param element find the root correpsonding to this element
     \return NodeIndex
    */
  inline CompIndex FindSet(const CompIndex & element) {
    //printf("FindSet(%i)", element);

    if (roots[element] == element)
      return element;
    else {
      // The second improvement, called path compression, is a way of
      // flattening the structure of the tree whenever Find is used on it.
      // The idea is that each node visited on the way to a root node may
      // as well be attached directly to the root node;
      // they all share the same representative.
      // To effect this, as Find recursively traverses up the tree,
      // it changes each node's parent reference to point to the root that it found.
      // The resulting tree is much flatter, speeding up future operations
      // not only on these elements but on those referencing them,
      // directly or indirectly.
      roots[element] = FindSet( roots[element] );
      return roots[element];
    }
  }

private:
  //! the ranks of each tree
  CompRank* ranks;
  //! the size of each comp
  CompSize* sizes;
  //! the nb of components
  CompIndex nb_comp;
  //! the total size of the disjoint set
  CompIndex count;

  //! the biggest component index
  CompIndex biggest_comp_idx;
  //! the biggest component index
  CompSize biggest_comp_size;

  /*!
     * \brief   Combine two sets into one.
     * All elements in those two sets will share the same set id that can be gotten using FindSet.
     */
  inline CompIndex Union(const CompIndex & set_x, const CompIndex & set_y) {
    //printf("Union(%i, %i)", set_x, set_y);

    CompIndex root_x = FindSet(set_x);
    CompIndex root_y = FindSet(set_y);
    if (root_x == root_y)
      return root_x;

    --nb_comp;

    // The first way, called union by rank, is to always attach
    // the smaller tree to the root of the larger tree, rather than vice versa.
    // Since it is the depth of the tree that affects the running time,
    // the tree with smaller depth gets added under the root of the deeper tree,
    // which only increases the depth if the depths were equal.
    // In the context of this algorithm, the term rank is used instead of depth
    // since it stops being equal to the depth if path compression
    // (described below) is also used.
    // One-element trees are defined to have a rank of zero,
    // and whenever two trees of the same rank r are united,
    // the rank of the result is r+1.
    // Just applying this technique alone yields an amortized running-time
    // of O(logn) per MakeSet, Union, or Find operation.
    if (ranks[root_x] < ranks[root_y]) {
      roots[root_x] = root_y;

      // update biggest_comp_idx
      if ( (sizes[root_y] += sizes[root_x]) > biggest_comp_size) {
        biggest_comp_idx = root_y;
        biggest_comp_size = sizes[root_y];
      }
      return root_y;
    }
    else {//if (ranks[root_x] >= ranks[root_y]) {
      roots[root_y] = root_x;

      // update biggest_comp_idx
      if ( (sizes[root_x] += sizes[root_y]) > biggest_comp_size) {
        biggest_comp_idx = root_x;
        biggest_comp_size = sizes[root_x];
      }

      if (ranks[root_x] == ranks[root_y])
        ++ranks[root_x];
      return root_x;
    }
  }

  void resize(const CompIndex & new_count, bool reset_roots = true){
    //printf("resize(%i)", new_count);

#ifdef TIMER_ON
    timer.reset();
#endif // TIMER_ON

    // delete the old arrays
    if (count != new_count) {
      count = new_count;

      if (roots != NULL)
        delete[] roots;
      if (ranks != NULL)
        delete[] ranks;
      if (sizes != NULL)
        delete[] sizes;


      // make the allocation for the new stuff
      roots = new CompIndex[count];
      ranks = new CompRank[count];
      sizes = new CompSize[count];
      //        roots = (CompIndex*) malloc( count * sizeof(CompIndex) );
      //        ranks = (CompRank*) malloc( count * sizeof(CompRank) );
      //        sizes = (CompSize*) malloc( count * sizeof(CompSize) );
    }

    biggest_comp_size = 0;
    biggest_comp_idx = 0;
    nb_comp = count;

    // reset ranks
    bzero(ranks, count * sizeof(CompRank));
    //std::fill_n( ranks, count, 0 );

    // reset sizes
    //    CompSize* sizes_ptr = sizes;
    //    for(CompIndex root_idx = 0 ; root_idx < count ; ++root_idx)
    //        *sizes_ptr++ = 1;
    std::fill_n( sizes, count, 1 );

    // reset roots
    if (reset_roots) {
      CompIndex* roots_ptr = roots;
      for(CompIndex root_idx = 0 ; root_idx < count ; ++root_idx)
        *roots_ptr++ = root_idx;
    } // end want_refill

#ifdef TIMER_ON
    timer.printTime("resize()");
#endif // TIMER_ON

  }
};

} // end namespace vision_utils

#endif


