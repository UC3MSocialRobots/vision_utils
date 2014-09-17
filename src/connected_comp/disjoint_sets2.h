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

// the interface
#include "comp_labeller_interface.h"

class DisjointSets2 : public CompLabellerInterface {
public:
  typedef int CompIndex;
  typedef uchar CompRank;
  typedef int CompSize;

  static const CompIndex NO_NODE = -1;

  DisjointSets2(const CompIndex & new_count = 0);
  DisjointSets2(cv::Mat1b & img);
  ~DisjointSets2();

  //! \see CompLabellerInterface::process_image()
  void process_image(cv::Mat1b & img);

  //! \see CompLabellerInterface::get_connected_components()
  void get_connected_components(const int cols,
                                std::vector< Comp > & components_pts,
                                std::vector<cv::Rect> & boundingBoxes);

  /*!
     * \brief   computes the biggest conencted component
     *
     * \param   img the monochrome image
     * \param   rep the vector which will contain the results
     */
  void biggestComponent_vector(const int cols,
                               Comp & rep);

  /*!
     * \brief   computes the biggest conencted component
     *
     * \param   img the monochrome image
     * \param   imgRep a monochrome image, of the same size or greater than img
     */
  void biggestComponent_image(const int cols, cv::Mat1b & imgRep);

  /*!
     * \brief   computess the center of the biggest connected component of the image
     *
     * \param   img the monochrome image
     * \return  the cv::Point corresponding to the center
     */
  cv::Point centroidOfMonochromeImage(const int cols);

  //! for display
  void display(CompIndex width = 10);

  //! \return the nb of components
  inline CompIndex nb_components() const { return nb_comp; }

  //! the different nodes (their pointer to their root)
  CompIndex* roots;

  /*!
     \fn FindSet
     \param element find the root correpsonding to this element
     \return NodeIndex
    */
  inline CompIndex FindSet(const CompIndex & element);

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
  inline CompIndex Union(const CompIndex & set1, const CompIndex & set2);

  void resize(const CompIndex & new_count, bool reset_roots = true);

};

#endif


