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
 *    \file examples/integrator/chemical_rate-equations.cpp
 *    \author Boris Houska, Julian Bonilla, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>


/**
*
* The problem is from chemical
*   kinetics, and consists of the following three rate equations:
*      dy1/dt = -p1*y1 + p2*y2*y3
*      dy2/dt =  p1*y1 - p2*y2*y3 - p3*(y2)^2
*      dy3/dt =  p3*(y2)^2
*   on the interval from t = 0.0 to t = 1.e10, with initial
*   conditions y1 = 1.0, y2 = y3 = 0. The reaction rates are: p1=0.04,
*   p2=1e4, and p3=3e7. The problem is stiff.
*
*/


int main( ){

    USING_NAMESPACE_ACADO


    // DEFINE THE RIGHT-HAND-SIDE
    // OF A DOUBLE PENDULUM:
    // ---------------------------

    DifferentialState             y1;
    DifferentialState             y2;
    DifferentialState             y3;

    DifferentialStateDerivative  dy1;
    DifferentialStateDerivative  dy2;
    DifferentialStateDerivative  dy3;

    Parameter                     p1;
    Parameter                     p2;
    Parameter                     p3;


    DifferentialEquation f;

    f << dy1 + p1*y1 - p2*y2*y3                 ;
    f << dy2 - p1*y1 + p2*y2*y3 + p3*y2*y2      ;
    f << dy3 - p3*y2*y2  ;


    // DEFINE AN INTEGRATOR:
    // ---------------------

    IntegratorBDF integrator(f);
	integrator.set(INTEGRATOR_PRINTLEVEL, MEDIUM );

    integrator.set(INTEGRATOR_TOLERANCE, 1e-5 );
    integrator.set(ABSOLUTE_TOLERANCE, 1e-6 );

    integrator.set(MAX_NUM_INTEGRATOR_STEPS, 2000  );
   // integrator.set(MAX_INTEGRATOR_STEPSIZE, 1e-3  );

//    integrator.set( LINEAR_ALGEBRA_SOLVER , SPARSE_LU );


    // DEFINE INITIAL VALUES:
    // ----------------------

    double x0[3] = { 1.0, 0.0, 0.0 };
    double pp[3] = { 0.04, 1e+4, 3e+7 };

    double t0   = 0.0 ;
    double tend = 4e9;


    // START THE INTEGRATION:
    // ----------------------

//     double a = acadoGetTime();

    //integrator.freezeAll();
	integrator.integrate( t0, tend, x0, 0, pp );

//     printf("%.16e \n",  acadoGetTime() - a );


    // GET THE RESULTS
    // ---------------

	VariablesGrid differentialStates;
	integrator.getX( differentialStates );
	
	differentialStates.print( "x" );


    return 0;
}



