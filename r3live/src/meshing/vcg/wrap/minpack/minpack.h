#ifndef LM_DIFF
#define LM_DIFF

/****************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004                                                \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *   
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#include <cminpack.h>
#include <stdlib.h>
/*
LMDiff is a class that performs non linear minimization:

min Sum_{i=0}^{M} ( F(x0,..,xN)_i ) ^2 )
Where:
F: R^N->R^M is a user defined function

****** basic example use ****

void evaluate (void *data,  int n_par, int m_dat, double* par, double* fvec, int iflag )
{
for(int i = 0 ; i < m_dat;++i)
fvec[i] = i * (par)[0] * (par)[0] + (i/2) * (par)[1] * (par)[1];
}


int main(){
LMDiff nlm;
double par[2]={4.0,-4.0};
nlm.InitControl();
nlm.Run(3,2,par,evaluate,0);
}

*/
/** Class LMDiff.
    This is a class for wrapping  the lmdif part of cminpack (see http://devernay.free.fr/hacks/cminpack.html).
 */
class LMDiff{
public:


// parameters for calling the high-level interface Run
// ( Run.c provides lm_control_default which sets default values ):
typedef struct {
	double ftol; 		// relative error desired in the sum of squares.
	double xtol; 		// relative error between last two approximations.
	double gtol; 		// orthogonality desired between fvec and its derivs.
	double epsilon; 	// step used to calculate the jacobian.
	double stepbound; 	// initial bound to steps in the outer loop.
	double fnorm; 		// norm of the residue vector fvec.
	int maxcall; 		// maximum number of iterations.
	int nfev; 		// actual number of iterations.
	int nprint; 		// desired frequency of print outs.
	int info; 		// status of minimization.
} lm_control_type;


// through the following parameters, lm_lmdif communicates with evaluate:
typedef struct {
	int nfev; 	// actual number of iterations.
	int nprint; 	// desired frequency of print outs.
	int stop; 	// if set to a nonzero value, minimization will stop.
} lm_status_type;


// ***** the following messages are referenced by the variable info.
static char * Message(const int & i){
static char *lm_infmsg[] = {
"improper input parameters",
"the relative error in the sum of squares is at most tol",
"the relative error between x and the solution is at most tol",
"both errors are at most tol",
"fvec is orthogonal to the columns of the jacobian to machine precision",
"number of calls to fcn has reached or exceeded 200*(n+1)",
"tol is too small. no further reduction in the sum of squares is possible",
"tol too small. no further improvement in approximate solution x possible",
"not enough memory"
};
return lm_infmsg[i];
}
lm_control_type control; // control of this object

/// Initialize the control: termination condition, steps size..
void InitControl(){lm_initialize_control();}

/// the subroutine that calculates fvec:
typedef int (lm_evaluate_type) (
	void *user_data, // user data (the same passed to Run
	int m, // the dimension of the co-domain of your F
	int n,
	const double* n_var, // the n parameters to compute your F
	double* fvec, // the values computed by F
	int iflag // status
);

void Run (
	int m, // size of the codomain of F
	int n , // size of the domain of F
	double* par, // starting values of the parameters
	lm_evaluate_type *evaluate, // yuor function F
	void *data, // user data (will be passed to F as they are
	lm_control_type *control // control
);


// compact high-level interface:

void Run( int m_dat,int n_par, double* par, lm_evaluate_type *evaluate,
void *data );

private:
void lm_initialize_control( );
};



/* *********************** high-level interface **************************** */


void LMDiff::lm_initialize_control( )
{
	control.maxcall = 10000;
	control.epsilon = 1.e-14;
	control.stepbound = 100.;
	control.ftol = 1.e-14;
	control.xtol = 1.e-14;
	control.gtol = 1.e-14;
	control.nprint = 0;
}

void LMDiff::Run( int m_dat,int n_par, double* par, lm_evaluate_type *evaluate,
void *data ){Run( m_dat, n_par, par, evaluate, data, &control);}


void LMDiff::Run( int m_dat,int n_par, double* par, lm_evaluate_type *evaluate,
void *data, lm_control_type *control )
{

// *** allocate work space.

double *fvec, *diag, *fjac, *qtf, *wa1, *wa2, *wa3, *wa4;
int *ipvt;

int n = n_par;
int m = m_dat;

if 	(!(fvec = (double*) malloc( m*sizeof(double))) ||
	!(diag = (double*) malloc(n* sizeof(double))) ||
	!(qtf = (double*) malloc(n* sizeof(double))) ||
	!(fjac = (double*) malloc(n*m*sizeof(double))) ||
	!(wa1 = (double*) malloc(n* sizeof(double))) ||
	!(wa2 = (double*) malloc(n* sizeof(double))) ||
	!(wa3 = (double*) malloc(n* sizeof(double))) ||
	!(wa4 = (double*) malloc( m*sizeof(double))) ||
	!(ipvt = (int*) malloc(n* sizeof(int)))) {
	control->info = 9;
	return;
}

// *** perform fit.

control->info = 0;
control->nfev = 0;
// this goes through the modified legacy interface:
control->info = 
lmdif(
evaluate,
data,
m,
n,
par,
fvec,
control->ftol,
control->xtol,
control->gtol,
control->maxcall*(n+1),
control->epsilon,
diag,
1,
control->stepbound,
control->nprint,
&(control->nfev),
fjac,
m,
ipvt,
qtf,
wa1,
wa2,
wa3,
wa4
);

if (control->info >= 8) control->info = 4;

// *** clean up.

free(fvec);
free(diag);
free(qtf);
free(fjac);
free(wa1);
free(wa2);
free(wa3 );
free(wa4);
free(ipvt);
}

#endif 