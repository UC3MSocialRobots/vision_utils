/*   This file is part of ukflib
 *
 *   Copyright (C) 2010, 2011
 *
 *   Author : Jeremy Fix
 *
 *   Contributor :
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *   Contact : Jeremy.Fix@gmail.com
 *
 */

/**
 * \example example-001.cc
 * \example example-002.cc
 * \example example-003.cc
 * \example example-004.cc
 * \example example-005.cc
 * \example example-006.cc
 * \example example-007.cc
 * \example example-009.cc
 */

/*! \mainpage Unscented Kalman Filter
 *
 * \section intro_sec Introduction
 *
 * Two algorithms are implemented and all of them taken from the PhD of Van Der Merwe "Sigma-Point Kalman Filters for Probabilistic Inference in Dynamic State-Space Models" :<BR>
 * - UKF for parameter estimation, Algorithm 6, p. 93<BR>
 * - UKF for state or joint estimation, additive noise case, Algorithm 8,  p. 108 <BR>
 *
 * \subsection usage Usage
 *
 * To use the library, you simply need to :
 * - define a parameter and state structure, which depends on the algorithm you use : ukf_*_param, ukf_*_state
 * - call the proper function initializing the state : ukf_*_init()
 * - iterate with the proper function by providing a sample \f$ (x_i, y_i) \f$  : ukf_*_iterate()
 * - free the memory : ukf_*_free()
 *  To use these functions, you simply need to define you evolution/observation functions, provided to the ukf_*_iterate functions as well as the samples.<BR>
 *
 *  <b>Warning :</b> Be sure to always use gsl_vector_get and gsl_vector_set in your evolution/observation functions, never access the fiels of the vectors with the data array of the gsl_vectors.<BR>
 *
 * \subsection param_estimation UKF for parameter estimation
 *
 * For UKF for parameter estimation, two versions are implemented : in case of a scalar output or vectorial output. The vectorial version works also in the scalar case but is more expensive (memory and time) than the scalar version when the output is scalar.<BR>
 * For the <b>scalar</b> version :
 * - the structures to define are ukf::parameter::ukf_param and ukf::parameter::ukf_scalar_state
 * - the methods to initialize, iterate, evaluate and free are : ukf::parameter::ukf_scalar_init , ukf::parameter::ukf_scalar_iterate , ukf::parameter::ukf_scalar_evaluate , ukf::parameter::ukf_scalar_free<BR>
 *
 * Examples using the scalar version : <BR>
 *  - Training a simple MPL on the XOR problem : example-001.cc
 *  - Training a MLP on the extended XOR : example-002.cc
 *  - Training a RBF for fitting a sinc function : example-003.cc
 *  - Finding the minimum of the Rosenbrock banana function : example-006.cc
 *
 * For the <b>vectorial</b> version :
 * - the structures to define are ukf::parameter::ukf_param and ukf::parameter::ukf_state
 * - the methods to initialize, iterate, evaluate and free are : ukf::parameter::ukf_init , ukf::parameter::ukf_iterate , ukf::parameter::ukf_evaluate , ukf::parameter::ukf_free<BR>
 *
 * Examples using the vectorial version : <BR>
 *  - Training a 2-2-3 MLP to learn the OR, AND, XOR functions : example-004.cc
 *  - Training a 2-12-2 MLP to learn the Mackay Robot arm data : example-005.cc
 *
 * \subsection joint_ukf Joint UKF
 *
 * The Joint UKF tries to estimate both the state and the parameters of a system. The structures/methods related to Joint UKF are :
 * - ukf::state::ukf_param and ukf::state::ukf_state for the parameters and the state representations
 * - ukf::state::ukf_init, ukf::state::ukf_free, ukf::state::ukf_iterate, ukf::state::ukf_evaluate for respectively initializing the structures, freeing the memory, iterating on one sample and evaluating the observation from the sigma points.<BR>
 * To see how to use Joint UKF, have a look to the example example-007.cc where we seek the parameters and state of a Lorentz attractor.
 *
 * \section install_sec Installation and running
 *
 * \subsection tools_subsec Requirements:
 * In addition to a g++ compiler with the standard libraries, you also need to install :
 *          - GSL (Gnu Scientific Library), available here : <a href="http://www.gnu.org/software/gsl/">http://www.gnu.org/software/gsl/</a>
 *
 * \subsection compilation Compilation, Installation
 * The installation follows the standard :
 * ./configure --prefix=PREFIX_TO_INSTALL
 * make
 * make install
 *
 * It will compile the library and the examples.
 *
 * \section parallel_implementation Parallel Implementation
 *
 * Here are some links on how the library may be written/executed in parallel. The basic operations we need are <br>
 * - matrix multiplication<br>
 * - matrix inversion <br>
 * - cholesky decomposition<br>
 *
 * We need also, for large outputs, to be able to share the representation of matrices.<br>
 * The GSL/C++ library does not provide an interface with MPI. There are basically two questions:<br>
 *
 * - which libraries provide the operations we need, and interfaced with MPI ?
 * - which language should be used ? Should we stick to C++ or to fortran for example which I think is used for intensive computations<br>
 *
 * There are a lot of libraries for linear algebra summarized here : http://www.netlib.org/utk/people/JackDongarra/la-sw.html . <br>
 * BLAS (Basic Linear Algebra) provides routines for vector-vector, matrix-vector and matrix-matrix operations. There is a parallel implementation of BLAS which is PBLAS.<br>
 * LAPACK provides routines for solving linear systems. It provides for example routines for performing standard decomposition (LU, Cholesky, QR, SVD). There is a parallel
 * implementation of LAPACK which is ScaLapack (http://www.netlib.org/scalapack/scalapack_home.html). ScaLapack also uses BLACS (Basic Linear Algebra Communication Subprograms) which
 * is the communication interface<br>
 * There exists a way to install ScaLapack with support of OpenMPI, an open source implementation of MPI : (http://www.open-mpi.org/faq/?category=mpi-apps#scalapack)<br>
 * In addition, on the fedora, there are already packages for installing OpenMpi and ScaLapack with OpenMPI support : <br>
 * yum install scalapack-openmpi-devel openmpi-devel.i686<br>
 *
 * There is also a PLapack (http://www.cs.utexas.edu/users/plapack/) package which seems to be a concurent to ScaLapack.<br>
 *
 * It looks like there is a project providing C++ bindings for Scalapack : http://cppscalapack.sourceforge.net/ . In fact there is no need for an extra library providing
 * bindings in C++ since the routines of Scalapack can be directly called from C. ScaLapack also has the nice property of distributing the representation of matrices.<br>
 *
 * Finally, and this is probably the easiest way to introduce a parallel implementation on a multi-CPU computer, there exists PLASMA (http://icl.cs.utk.edu/plasma/index.html).<br>
 *
 * \section complexity Complexity
 *
 * \subsection matrix_operations Matrix operations
 *
 * Whether for UKF or EKF we need some basic matrix or operations such as multiplication, cholesky decomposition, inverse. We make use of the BLAS library. Here is how the BLAS performs on these operations :
 * - Matrix multiplication : the multiplication of matrices of size \f$(n,p)\f$ \f$(p,m)\f$ is in \f$O(npm)\f$
 * - Matrix inverse : the inverse of a matrix of size \f$(n,n)\f$ is in \f$O(n^3)\f$
 * - Cholesky factorization : the cholesky factorization of a matrix of size \f$(n,n)\f$  is in \f$O(n^3/6)\f$
 *
 * To give an idea of the relative complexities, more accurate than just specifying a \f$O(..)\f$, on one machine, the precise complexities were :
 * - Matrix multiplication of matrices \f$(n,p)\f$ \f$(p,n)\f$ is in \f$6.3.1e^{-9} n^1.75 p\f$
 * - Matrix inverse of a matrix of size \f$(n,n)\f$ is in \f$5.2.1e^{-12} n^3.89\f$
 * - Cholesky factorization of a matrix of size \f$(n,n)\f$ is in \f$3.87 1e^{-10} n^{2.9}\f$
 *
 * \subsection ukf_parameter UKF for parameter estimation
 *
 * Without counting the matrix-vector products, UKF for parameter estimation requires :
 * - One cholesky decomposition of a matrix \f$(n,n)\f$
 * - 2n+1 evaluation of the observation function
 * - One inverse of size \f$(no,no)\f$
 * - 3 matrix products of sizes \f$(n,no)-(no,no)\f$, \f$(n,no)-(no,no)\f$, \f$(no,no)-(no,n)\f$
 * This leads to an overall complexity in \f$n^3 + no^3 + 3nno^2\f$ plus the 2n+1 evaluations of the observation function
 *
 * \subsection ukf_state UKF for state estimation
 *
 * \subsection ekf_state EKF for state estimation
 *
 * EKF for state estimation requires :
 * - One evaluation of the evolution function
 * - One evaluation of the Jacobian of the evolution function
 * - One evaluation of the observation function
 * - One evaluation of the Jacobian of the observation function
 * - 3 products of matrices of size \f$(n, no)\f$ \f$(no, n)\f$ or \f$(n,n)\f$ \f$(n,no)\f$
 * - 2 products of matrices of size \f$(n, no)\f$ \f$(no,no)\f$ or \f$(no, n)\f$ \f$(n,no)\f$
 * - 3 product of matrices of size \f$(n,n)\f$ \f$(n,n)\f$
 * - 1 inverse of a matrix of size \f$(no,no)\f$
 * - Some matrix copies ?
 * If the jacobian of the observations is a diagonal matrix, the complexity can be reduced. The four products of equation 2.40 (2 of complexities \f$ n^2no\f$, 2 of complexities \f$n.n0^2\f$)
 * can be reduced to three products of complexity \f$n.no\f$ and one of complexity \f$n^2no\f$. The complexity of equation 2.42 can also be reduced from \f$n^2.n0\f$ down to \f$n.n0\f$. It can be even better
 * when the product is of the form H P H^T with H diagonal.<br>
 * So overall, the complexity of EKF is in :
 * - If the gradient of the observations is not diagonal : \f$O(3 n^2no + 2 n no^2 + 3 n^3 + no^3)\f$ + one evaluation of the evolution and observation functions and one evaluation of their Jacobians
 * - If the gradient of the observations is diagonal : \f$O(n^2no + no^2 + 2n.no + 3 n^3 + no^3)\f$ + one evaluation of the evolution and observation functions and one evaluation of their Jacobians
 * In particular, if the size of the state and of the observation is the same, we decrease the complexity from \f$ 9 N^3\f$ down to \f$ 5N^3 + 3N^2\f$.
 *
 * \section example Example outputs
 *
 * \subsection example1 Example 1 : Learning XOR with a 2-2-1 MLP
 *
 * Running (maybe several times if falling on a local minima) example-001-xor, you should get the following classification :
 *
 * \image html "example-001.png" "XOR classification"
 *
 * An example set of learned parameters is :
 *
 * x[0] -- (9.89157) --> y[0] <BR>
 * x[1] -- (4.18644) --> y[0]<BR>
 * Bias y[0] : 8.22042<BR>
 *
 * x[0] -- (10.7715) --> y[1]<BR>
 * x[1] -- (4.18047) --> y[1]<BR>
 * Bias y[1] : -8.70185<BR>
 *
 * y[0] -- (6.9837) --> z<BR>
 * y[1] -- (-6.83324) --> z<BR>
 * Bias z : -3.89682<BR>
 *  
 * The transfer function is a sigmoid : \f$ f(x) = \frac{2}{1 + exp(-x)}-1\f$
 *
 * \subsection example2 Example 2 : Learning the extended XOR with a 2-12-1 MLP and a parametrized transfer function
 *
 * Here we use a 2-12-1 MLP, with a sigmoidal transfer function, to learn the extended XOR problem. The transfer function has the shape : \f$f(x) = \frac{1}{1.0 + exp(-x)}\f$
 *
 * The classification should look like this :
 *
 * \image html "example-002.png" "Extended XOR classification"
 *
 * \subsection example3 Example 3 : Approximating the sinc function with a Radial Basis Function network
 *
 * In this example, we use a RBF network with 10 kernels to approximate the sinc function on [-5.0,5.0]
 * To make the life easier for the algorithm, we evenly spread the centers of the gaussians on [-5.0, 5.0].
 *
 * The results are saved in 'example-003.data', the first column contains the x-position, the second column the result given by the trained RBF and the last column the value of sinc(x)
 *
 * \image html "example-003.png" "RBF learning the sinc function"
 *
 * \subsection example4 Example 4 : Using a 2-2-3 MLP to learn three boolean functions : XOR, AND, OR
 *
 * \subsection example5 Example 5 : Using a 2-12-2 MLP to learn the Mackay-robot arm problem
 *
 * In this example, we learn the two outputs (x,y) from the inputs (theta, phi) of the Mackay-robot arm dataset. For this
 * we train a 2-12-2 MLP with a parametrized sigmoidal transfer function.
 *
 * \image html "example-005-x.png" "Learning the x-component" \image html "example-005-y.png" "Learning the y-component"
 *
 * \subsection example6 Example 6 : Finding the minimum of the Rosenbrock banana function
 *
 * We use here UKF for parameter estimation to find the minimum of the Rosenbrock banana function : \f$ f(x,y) = (1 - x)^2 + 100 ( y - x^2)^2 \f$<BR>
 *
 * \image html "example-006.png" "Minimisation of the Rosenbrock banana function"
 *
 * \subsection example7 Example 7 : Finding the parameters of a Lorentz attractor
 *
 * In this example, we try to find the parameters (initial condition, evolution parameters) of a noisy lorentz attractor. The dynamic of the lorentz attractor is defined by the three equations :
 *
 * \f$ \frac{dx}{dt} = \sigma ( y - x ) \f$ <BR>
 * \f$ \frac{dy}{dt} = x (\rho - z) - y \f$ <BR>
 * \f$ \frac{dz}{dt} = xy  - \beta z \f$ <BR>
 * While observing a noisy trajectory of such a Lorentz attractor, the algorithm tries to find the current state
 * and the evolution parameters \f$ (\sigma, \rho, \beta)\f$. The samples we provide are \f$ (t_i, {x(t_i), y(t_i), z(t_i)})\f$.
 *
 * To clearly see how UKF catches the true state, we initialized the estimated state of UKF to -15, -15 , -15<BR>
 *
 *  \image html "example-007-rms.png" "Learning RMS" \image html "example-007.png" "Estimated state with the true state and its noisy observation"
 *
 * <BR><BR>
 *
 */


#ifndef UKF_H
#define UKF_H

#include "ukf_math.h"
#include "ukf_types.h"
#include "ukf_samples.h"

#include "ukf_parameter_scalar.h"
#include "ukf_parameter_ndim.h"

#include "ukf_state_ndim.h"
#include "ukf_sr_state_ndim.h"

#include "ekf.h"
#include "ekf_types.h"

/**
  * @brief In this section we implement the Unscented Kalman Filter for parameter estimation and Joint UKF
  * involving the Scaled Unscented Transform detailed in Van der Merwe PhD Thesis
  *
*/
namespace ukf
{

}




#endif  // UKF_H
