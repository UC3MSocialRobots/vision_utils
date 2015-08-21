/************************************************************************
*
*  lap.h
   version 1.0 - 21 june 1996
   author  Roy Jonker, MagicLogic Optimization Inc.

   header file for LAP
*
**************************************************************************/

/*************** CONSTANTS  *******************/

  #define BIG 1E8

/*************** TYPES      *******************/

  typedef int row;
  typedef int col;
  typedef double cost;

/*************** FUNCTIONS  *******************/

/*! input:
// dim        - problem size
// assigncost - cost matrix

// output:
// rowsol     - column assigned to row in solution
// colsol     - row assigned to column in solution
// u          - dual variables, row reduction numbers
// v          - dual variables, column reduction numbers
*/
extern double lap(int dim, double **assigncost,
               int *rowsol, int *colsol, double *u, double *v);

extern void checklap(int dim, double **assigncost,
                     int *rowsol, int *colsol, double *u, double *v);

