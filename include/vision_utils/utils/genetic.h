#ifndef GENETIC_H
#define GENETIC_H

#include <vector>
#include <algorithm>
#include <limits>
#include <sstream>

#include "vision_utils/utils/debug_utils.h"

//! a simple barycenter implementation
inline double barycenter(const double & A, const double & B, const double & weightA) {
  return weightA * A + (1 - weightA) * B;
}

/* A generic implementation for genetic algorithms.

  */
template<class _Element>
class GeneticSolver {
public:
  static const unsigned int DEFAULT_POPULATION_SIZE = 20;
  static const unsigned int DEFAUT_NB_GENERATIONS = 20;
  /*! the RATE of the current population that will be used for
      crossover and mutation */
  static const double POPULATION_KEPT_RATE = 0.3;
  /*! the rate of mutation in the new population.
      The leftover is obtained by crossover. */
  static const double POPULATION_MUTATION_RATE = 0.5;

  //! a simple structure for keeping elements along with their grades
  struct GradedElement {
    //! a copy of the element
    _Element element;
    //! grade: the higher the better
    double grade;
  };

  //////////////////////////////////////////////////////////////////////////////

  //! grade an element - the higher the better
  virtual double fitness(const _Element & to_grade) = 0;

  //////////////////////////////////////////////////////////////////////////////

  //! mix two elements
  virtual void crossover(const _Element & parent1,
                         const _Element & parent2,
                         _Element & crossed_son) = 0;

  //////////////////////////////////////////////////////////////////////////////

  /*!
    The mutation function.
    \arg mutation_rate
          between 0 (no randomness) and 1 (completely random)
   */
  virtual void mutation(const _Element & parent,
                        const double & mutation_rate,
                        _Element & mutated_son) = 0;

  //////////////////////////////////////////////////////////////////////////////

  //! return a copy of the best element
  GradedElement get_best_element() {
    return _best_element;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*! the proper genetic algorithm.
      You can get the result of it with get_best_element()
  */
  void run_algorithm(const unsigned int population_size = DEFAULT_POPULATION_SIZE,
                     const unsigned int nb_generations = DEFAUT_NB_GENERATIONS) {
    // init the buffers size
    _pop_buffer_1.clear();
    _pop_buffer_2.clear();
    _pop_buffer_1.resize(population_size);
    _pop_buffer_2.resize(population_size);

    // set the pointers
    _current_generation = &_pop_buffer_1;
    _next_generation = &_pop_buffer_2;

    // set the best element grade to infinity
    _best_element.grade = -std::numeric_limits<double>::infinity();

    // fill the current with totally random content
    for (unsigned int elt_idx = 0; elt_idx < population_size; ++elt_idx)
      mutation((*_current_generation)[elt_idx].element, 1.,
               (*_current_generation)[elt_idx].element);

    // compute the shares of mutation and crossover
    unsigned int last_kept_elt_index = POPULATION_KEPT_RATE * population_size;
    unsigned int mutation_elts_nb = POPULATION_MUTATION_RATE * population_size;

    // run on each generation
    for (unsigned int generation_idx = 0;
         generation_idx < nb_generations;
         ++generation_idx) {
      //maggieDebug2("Generation %i", generation_idx);

      // compute all grades of the current generation
      for (unsigned int elt_idx = 0; elt_idx < population_size; ++elt_idx)
        (*_current_generation)[elt_idx].grade
          = fitness((*_current_generation)[elt_idx].element);

      // sort the vector according to the grade
      std::sort(_current_generation->begin(),
                _current_generation->end(),
                compare_graded_elements);
      //      std::ostringstream grades;
      //      for (unsigned int elt_idx = 0; elt_idx < population_size; ++elt_idx)
      //        grades << (*_current_generation)[elt_idx].grade << ", ";
      //      maggieDebug2("grades:'%s'", grades.str().c_str());

      // keep the best solution if it is the best
      if (_current_generation->front().grade
          > _best_element.grade) {
        _best_element = _current_generation->front();
        maggieDebug3("New best element at generation %i, grade:%g",
                     generation_idx, _best_element.grade);
      }

      // build the new generation:
      // the first part is obtained by mutation of the best elements
      // a mutation rates that starts at 1 and ends at 0
      double curr_mutation_rate = 1 - generation_idx / nb_generations / 2;
      for (unsigned int elt_idx = 0; elt_idx < mutation_elts_nb; ++elt_idx)
        mutation((*_current_generation)[rand() % last_kept_elt_index].element,
                 curr_mutation_rate,
                 (*_next_generation)[elt_idx].element);

      // build the new generation:
      // the second part by crossover
      for (unsigned int elt_idx = mutation_elts_nb;
           elt_idx < population_size;
           ++elt_idx)
        crossover((*_current_generation)[rand() % last_kept_elt_index].element,
                  (*_current_generation)[rand() % last_kept_elt_index].element,
                  (*_next_generation)[elt_idx].element);

      // swap the buffers
      std::swap(_current_generation, _next_generation);

    } // end loop generation_idx
  } // end run_algorithm();

  //////////////////////////////////////////////////////////////////////////////

  /*! the proper genetic algorithm */
  void run_algorithm_brute_force(const unsigned int population_size = DEFAULT_POPULATION_SIZE,
                                 const unsigned int nb_generations = DEFAUT_NB_GENERATIONS) {
    _pop_buffer_1.clear();
    unsigned int ntries = population_size * nb_generations;
    _pop_buffer_1.resize(ntries);
    // set the best element grade to infinity
    _best_element.grade = -std::numeric_limits<double>::infinity();

    for (unsigned int elt_idx = 0; elt_idx < ntries; ++elt_idx) {
      // set random elements
      mutation(_pop_buffer_1[elt_idx].element, 1, _pop_buffer_1[elt_idx].element);
      // evaluate them
      _pop_buffer_1[elt_idx].grade = fitness(_pop_buffer_1[elt_idx].element);
      // get the best
      if (_best_element.grade < _pop_buffer_1[elt_idx].grade)
        _best_element = _pop_buffer_1[elt_idx];
    }
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  /*! a comparator function for sorting std::vectors of graded elements.
    Returns the highest elements first */
  static bool compare_graded_elements(const GradedElement & e1,
                                      const GradedElement & e2) {
    return e1.grade > e2.grade;
  }

  typedef std::vector<GradedElement> Pop;

  Pop* _current_generation;
  Pop* _next_generation;

  GradedElement _best_element;

  Pop _pop_buffer_1;
  Pop _pop_buffer_2;
};

#endif // GENETIC_H
