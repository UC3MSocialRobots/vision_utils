#include "ukf.h"
#include <combinatorics/combinatorics_utils.h>
#include <gnuplot-cpp/gnuplot_i.hpp>

using namespace ukf::state;

/*************************************************************************************/
/*            Definition of the evolution and observation functions                  */
/*************************************************************************************/

// Evolution function
void evo_function(gsl_vector * params, gsl_vector * xk_1, gsl_vector * xk)
{
  double x = gsl_vector_get(xk_1,0);
  double y = gsl_vector_get(xk_1,1);
  double z = gsl_vector_get(xk_1,2);
  double orien = gsl_vector_get(xk_1,3);
  double speed = gsl_vector_get(xk_1,4);

  double dt = gsl_vector_get(params, 0);
  double brownian_noise_amp = .1;

  gsl_vector_set(xk, 0, x + dt * speed * cos(orien));
  gsl_vector_set(xk, 1, y + dt * speed * sin(orien));
  gsl_vector_set(xk, 2, z);
  gsl_vector_set(xk, 3, orien
                 + dt * brownian_noise_amp * vision_utils::rand_gaussian());
  gsl_vector_set(xk, 4, speed
                 + dt * brownian_noise_amp * vision_utils::rand_gaussian());
}

// Observation function
void obs_function(gsl_vector * xk , gsl_vector * yk)
{
  double observation_noise_amp = .1;
  for(unsigned int i = 0 ; i < yk->size ; ++i)
    gsl_vector_set(yk, i, gsl_vector_get(xk,i)
                   + observation_noise_amp * vision_utils::rand_gaussian());
}

/*****************************************************/
/*                    main                           */
/*****************************************************/

int main(int argc, char** argv) {
  printf("main()\n");

  srand(time(NULL));

  // Definition of the parameters and state variables
  ukf_param p;
  ukf_state s;
  // The parameters for the evolution equation
  s.params = gsl_vector_alloc(1);
  double DT = .1;
  s.params->data[0] = DT;

  // Initialization of the parameters
  p.n = 5;
  p.no = 3;
  p.kpa = 0.0;
  p.alpha = 0.9;
  p.beta = 2.0;

  //EvolutionNoise * evolution_noise = new EvolutionAnneal(1e-1, 1.0, 1e-2);
  //EvolutionNoise * evolution_noise = new EvolutionRLS(1e-5, 0.9995);
  //EvolutionNoise * evolution_noise = new EvolutionRobbinsMonro(1e-4, 1e-3);
  EvolutionNoise * evolution_noise = new EvolutionRLS(1e-5, 0.9995);
  p.evolution_noise = evolution_noise;

  p.measurement_noise = .5;
  p.prior_x= 1.0;

  // Initialization of the state and parameters
  ukf_init(p,s);

  // Allocate the input/output vectors
  gsl_vector * yi = gsl_vector_alloc(p.no);
  gsl_vector_set_zero(yi);

  int epoch = 0;
  std::vector<double> x_hist, y_hist, z_hist;
  std::vector<double> x_computed_hist, y_computed_hist, z_computed_hist;

  while( epoch < 200 ) {
    double x = 0 + cos(epoch / 25.f) + drand48() * p.measurement_noise;
    double y = 1 + sin(epoch / 25.f) + drand48() * p.measurement_noise;
    double z = 1.8 + drand48() * p.measurement_noise;

    gsl_vector_set(yi, 0, x);
    gsl_vector_set(yi, 1, y);
    gsl_vector_set(yi, 2, z);

    // Provide the observation and iterate
    // ukf_param &p, ukf_state &s, FunctProcess f, FunctObservation h, gsl_vector* yi
    ukf_iterate(p, s, evo_function, obs_function, yi);

    // compute error
    double error =
        fabs(x - gsl_vector_get(s.xi, 0)) +
        fabs(y - gsl_vector_get(s.xi, 1)) +
        fabs(z - gsl_vector_get(s.xi, 2));

    std::ostringstream xi_str;
    for (int i = 0; i < p.n; ++i)
      xi_str << gsl_vector_get(s.xi, i) << ", ";
    printf("error:%g, xi:%s\n", error, xi_str.str().c_str());

    x_hist.push_back(x);
    y_hist.push_back(y);
    z_hist.push_back(z);
    x_computed_hist.push_back(gsl_vector_get(s.xi, 0));
    y_computed_hist.push_back(gsl_vector_get(s.xi, 1));
    z_computed_hist.push_back(gsl_vector_get(s.xi, 2));
    ++epoch;
  }

  Gnuplot plotter;
  plotter.set_style("lines").plot_xyz(x_hist, y_hist, z_hist);
  plotter.set_style("lines").plot_xyz(x_computed_hist, y_computed_hist, z_computed_hist);

  std::cout << std::endl << "Press ENTER to continue..." << std::endl;
  std::cin.clear();
  std::cin.ignore(std::cin.rdbuf()->in_avail());
  std::cin.get();


  /***********************************************/
  /****            Free the memory            ****/
  /***********************************************/
  ukf_free(p,s);

  return 0;
}
