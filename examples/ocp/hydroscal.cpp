/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



 /**
  *    \file examples/ocp/hydroscal.cpp
  *    \author Boris Houska, Hans Joachim Ferreau, David Ariens
  *    \date 2009-2010
  *
  *  Distillation model
  *  82 diff + 122 alg var
  *  Zoltan Nagy, Moritz Diehl, 2000
  *
  *  For a description see e.g. Chapter 7 in the PhD thesis
  *  "Real-Time Optimization for Large Scale Nonlinear Processes" (2001) by Moritz Diehl
  *  Download at: http://www.ub.uni-heidelberg.de/archiv/1659/
  *
  *  Optimization problem is to steer the column
  *  from a disturbed state back into the nominal
  *  operating point, minimizing an integrated
  *  least-squares deviation of two temperatures
  *  and the controls.
  *
  *  EXPRESSIONS:
  *  u = (  L_{vol} , Q                                           )^T
  *  x = (  X_0, ..., X_{41}, n_1, ..., n_{40}                    )^T
  *  z = (  L_1, ..., L_{40}, V_1, ..., V_{40}, T_0, ..., T_{41}  )^T.
  */


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include "../integrator/hydroscal_model.hpp"     // MODEL FILE

#include <time.h>

USING_NAMESPACE_ACADO

void ffcn_model( double *x, double *f, void *user_data  ){

    int i;
    double *xd = new double[NXD];
    double *xa = new double[NXA];
    double *u  = new double[ NU];
    double *p  = new double[ NP];

    for( i = 0; i < NXD; i++ ) xd[i] = x[            1+i ];
    for( i = 0; i < NXA; i++ ) xa[i] = x[        NXD+1+i ];
    for( i = 0; i <  NU; i++ )  u[i] = x[    NXA+NXD+1+i ];
    for( i = 0; i <  NP; i++ )  p[i] = x[ NXA+NXD+NU+1+i ];

    ffcn( &x[0], xd, xa, u, p, f );
    gfcn( &x[0], xd, xa, u, p, &(f[NXD]) );

    delete[] xd;
    delete[] xa;
    delete[]  u;
    delete[]  p;

}



int main( ){

	double clock1 = clock();

	double t_start    		=    0.0;
	double t_end      		=  600.0;
	int    intervals  		=    5  ;
	int    i;

	TIME t;
	DifferentialState 		x("", NXD, 1);
	AlgebraicState 			z("", NXA, 1);
	Control 				u("", NU, 1);
	Parameter 				p("", NP, 1);
	IntermediateState 		is(1+NXD+NXA+NU+NP);

	                        is(0)              = t;
	for (i=0; i < NXD; ++i) is(1+i)            = x(i);
	for (i=0; i < NXA; ++i) is(1+NXD+i)        = z(i);
	for (i=0; i < NU;  ++i) is(1+NXD+NXA+i)    = u(i);
	for (i=0; i < NP;  ++i) is(1+NXD+NXA+NU+i) = p(i);

	CFunction hydroscalModel( NXD+NXA, ffcn_model );


	// Define a Right-Hand-Side:
	// -------------------------
	DifferentialEquation f;
	f << hydroscalModel(is);


	// DEFINE INITIAL VALUES:
	// ----------------------
	double xd[NXD] = {  2.1936116177990631E-01,   // X_0
						3.3363028623863722E-01,   // X_1
						3.7313133250625952E-01,   // X_2
						3.9896472354654333E-01,   // X_3
						4.1533719381260475E-01,   // X_4
						4.2548399372287182E-01,   // X_5

						4.3168379354213621E-01,   // X_6
						4.3543569751236455E-01,   // X_7
						4.3768918647214428E-01,   // X_8
						4.3903262905928286E-01,   // X_9
						4.3982597315656735E-01,   // X_10
						4.4028774979047969E-01,   // X_11
						4.4055002518902953E-01,   // X_12
						4.4069238917008052E-01,   // X_13
						4.4076272408112094E-01,   // X_14
						4.4078980543461005E-01,   // X_15
						4.4079091412311144E-01,   // X_16
						4.4077642312834125E-01,   // X_17
						4.4075255679998443E-01,   // X_18
						4.4072304911231042E-01,   // X_19
						4.4069013958173919E-01,   // X_20
						6.7041926189645151E-01,   // X_21
						7.3517997375758948E-01,   // X_22
						7.8975978943631409E-01,   // X_23
						8.3481725159539033E-01,   // X_24
						8.7125377077380739E-01,   // X_25
						9.0027275078767721E-01,   // X_26
						9.2312464536394301E-01,   // X_27
						9.4096954980798608E-01,   // X_28
						9.5481731262797742E-01,   // X_29
						9.6551271145368878E-01,   // X_30
						9.7374401773010488E-01,   // X_31
						9.8006186072166701E-01,   // X_32
						9.8490109485675337E-01,   // X_33
						9.8860194771099286E-01,   // X_34
						9.9142879342008328E-01,   // X_35
						9.9358602331847468E-01,   // X_36
						9.9523105632238640E-01,   // X_37
						9.9648478785701988E-01,   // X_38
						9.9743986301741971E-01,   // X_39
						9.9816716097314861E-01,   // X_40
						9.9872084014280071E-01,   // X_41
						3.8633811956730968E+00,   // n_1
						3.9322260498028840E+00,   // n_2
						3.9771965626392531E+00,   // n_3
						4.0063070333869728E+00,   // n_4
						4.0246026844143410E+00,   // n_5
						4.0358888958821835E+00,   // n_6
						4.0427690398786789E+00,   // n_7
						4.0469300433477020E+00,   // n_8
						4.0494314648020326E+00,   // n_9
						4.0509267560029381E+00,   // n_10
						4.0518145583397631E+00,   // n_11
						4.0523364846379799E+00,   // n_12
						4.0526383977460299E+00,   // n_13
						4.0528081437632766E+00,   // n_14
						4.0528985491134542E+00,   // n_15
						4.0529413510270169E+00,   // n_16
						4.0529556049324462E+00,   // n_17
						4.0529527471448805E+00,   // n_18
						4.0529396392278008E+00,   // n_19
						4.0529203970496912E+00,   // n_20
						3.6071164950918582E+00,   // n_21
						3.7583754503438387E+00,   // n_22
						3.8917148481441974E+00,   // n_23
						4.0094300698741563E+00,   // n_24
						4.1102216725798293E+00,   // n_25
						4.1944038520620675E+00,   // n_26
						4.2633275166560596E+00,   // n_27
						4.3188755452109175E+00,   // n_28
						4.3630947909857642E+00,   // n_29
						4.3979622247841386E+00,   // n_30
						4.4252580012497740E+00,   // n_31
						4.4465128947193868E+00,   // n_32
						4.4630018314791968E+00,   // n_33
						4.4757626150015568E+00,   // n_34
						4.4856260094946823E+00,   // n_35
						4.4932488551808500E+00,   // n_36
						4.4991456959629330E+00,   // n_37
						4.5037168116896273E+00,   // n_38
						4.5072719605639726E+00,   // n_39
						4.5100498969782414E+00    // n_40
					 };

	double ud[ NU] = {  4.1833910982822058E+00,   // L_vol
						2.4899344742988991E+00    // Q
					 };

	double pd[ NP] = {  1.5458567140000001E-01,   // n^{ref}_{tray}  = 0.155 l
						1.7499999999999999E-01,   // (not in use?)
						3.4717208398678062E-01,   // \alpha_{rect}   = 35 %
						6.1895708603484367E-01,   // \alpha_{strip}  = 62 %
						1.6593025789999999E-01,   // W_{tray}        = 0.166 l^{-.5} s^{.1}
						5.0695122527590109E-01,   // Q_{loss}        = 0.51 kW
						8.5000000000000000E+00,   // n^v_0           = 8.5 l
						1.7000000000000001E-01,   // n^v_{N+1}       = 0.17 l
						9.3885430857029321E+04,   // P_{top}         = 939 h Pa
						2.5000000000000000E+02,   // \Delta P_{strip}= 2.5 h Pa and \Delta P_{rect} = 1.9 h Pa
						1.4026000000000000E+01,   // F_{vol}         = 14.0 l h^{-1}
						3.2000000000000001E-01,   // X_F             = 0.32
						7.1054000000000002E+01,   // T_F             = 71 oC
						4.7163089489100003E+01,   // T_C             = 47.2 oC
						4.1833910753991770E+00,   // (not in use?)
						2.4899344810136301E+00,   // (not in use?)
						1.8760537088149468E+02    // (not in use?)
		             };

	DVector x0(NXD, xd);
	DVector p0(NP,  pd);


	// DEFINE AN OPTIMAL CONTROL PROBLEM:
	// ----------------------------------
	OCP ocp( t_start, t_end, intervals );

	// LSQ Term on temperature deviations and controls
	Function h;


//     for( i = 0; i < NXD; i++ )
// 		h << 0.001*x(i);


	h << 0.1 * ( z(94)  - 88.0    );   // Temperature tray 14
	h << 0.1 * ( z(108) - 70.0    );   // Temperature tray 28
	h << 0.01 * ( u(0)   - ud[0]   );   // L_vol
	h << 0.01 * ( u(1)   - ud[1]   );   // Q
	ocp.minimizeLSQ( h );

	// W.r.t. differential equation
	ocp.subjectTo( f );

	// Fix states
	ocp.subjectTo( AT_START, x == x0 );

	// Fix parameters
	ocp.subjectTo( p == p0 );

	// Path constraint on controls
	ocp.subjectTo( ud[0] - 2.0 <=  u(0)  <= ud[0] + 2.0 );
	ocp.subjectTo( ud[1] - 2.0 <=  u(1)  <= ud[1] + 2.0 );



	// DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
	// ---------------------------------------------------
	OptimizationAlgorithm algorithm(ocp);

	algorithm.initializeAlgebraicStates("hydroscal_algebraic_states.txt");

	algorithm.set( INTEGRATOR_TYPE, 		 INT_BDF 			);
	algorithm.set( MAX_NUM_ITERATIONS, 		 5	 				);
	algorithm.set( KKT_TOLERANCE, 			 1e-3 				);
	algorithm.set( INTEGRATOR_TOLERANCE, 	 1e-4 				);
	algorithm.set( ABSOLUTE_TOLERANCE  , 	 1e-6 				);
	algorithm.set( PRINT_SCP_METHOD_PROFILE, YES 				);
	algorithm.set( LINEAR_ALGEBRA_SOLVER, 	 SPARSE_LU 			);
	algorithm.set( DISCRETIZATION_TYPE, 	 MULTIPLE_SHOOTING 	);

    //algorithm.set( LEVENBERG_MARQUARDT, 1e-3 );

    algorithm.set( DYNAMIC_SENSITIVITY,  FORWARD_SENSITIVITY_LIFTED );
	//algorithm.set( DYNAMIC_SENSITIVITY,  FORWARD_SENSITIVITY );
	//algorithm.set( CONSTRAINT_SENSITIVITY,  FORWARD_SENSITIVITY );
	//algorithm.set( ALGEBRAIC_RELAXATION,ART_EXPONENTIAL );    //results in an extra step but steps are quicker


	algorithm.solve();

	double clock2 = clock();
	printf("total computation time = %.16e \n", (clock2-clock1)/CLOCKS_PER_SEC  );


	// PLOT RESULTS:
	// ---------------------------------------------------
	VariablesGrid out_states;
	algorithm.getDifferentialStates( out_states );
	out_states.print( "OUT_states.m","STATES",PS_MATLAB );

	VariablesGrid out_controls;
	algorithm.getControls( out_controls );
	out_controls.print( "OUT_controls.m","CONTROLS",PS_MATLAB );

	VariablesGrid out_algstates;
	algorithm.getAlgebraicStates( out_algstates );
	out_algstates.print( "OUT_algstates.m","ALGSTATES",PS_MATLAB );

	GnuplotWindow window;
	window.addSubplot( out_algstates(94),  "Temperature tray 14" );
	window.addSubplot( out_algstates(108), "Temperature tray 28" );
	window.addSubplot( out_controls(0),    "L_vol"               );
	window.addSubplot( out_controls(1),    "Q"                   );
	window.plot( );

    return 0;
}



