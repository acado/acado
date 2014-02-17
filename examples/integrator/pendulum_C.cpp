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
 *    \file examples/integrator/pendulum_C.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>


USING_NAMESPACE_ACADO


void ffcn_model( double *x, double *f, void *user_data ){

//  double       time =  x[ 0];    // the time
    double        phi =  x[ 1];    // the angle phi
    double       dphi =  x[ 2];    // the first derivative of phi w.r.t time
    double          F =  x[ 3];    // a force acting on the pendulum
    double          l =  x[ 4];    // the length of the pendulum

    const double m     = 1.0  ;    // the mass of the pendulum
    const double g     = 9.81 ;    // the gravitational constant
    const double alpha = 2.0  ;    // frictional constant

    f[0] = dphi;
    f[1] = -(m*g/l)*sin(phi) - alpha*dphi + F/(m*l);
}


int main( ){

    DifferentialState phi, dphi;

    Control u;
    Parameter p;
    TIME t;

    IntermediateState x(5);

    x(0) = t   ;
    x(1) = phi ;
    x(2) = dphi;
    x(3) = u   ;
    x(4) = p   ;

    CFunction pendulumModel( 2, ffcn_model );

    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialEquation f;

    f << pendulumModel(x);


    // DEFINE AN INTEGRATOR:
    // ---------------------

	IntegratorRK45 integrator( f );

	integrator.set(INTEGRATOR_PRINTLEVEL, HIGH );
	

    // DEFINE INITIAL VALUES:
    // ----------------------

    double x_start[2] = { 0.0, 0.0 };
    double u_     [1] = { 1.0      };
    double p_     [1] = { 1.0      };

    double t_start    =  0.0;
    double t_end      =  1.0;


    // START THE INTEGRATION:
    // ----------------------

    integrator.freezeAll();
    integrator.integrate( t_start, t_end, x_start, 0, p_, u_ );


    // DEFINE A SEED MATRIX:
    // ---------------------
    DVector seed1(2);
    DVector seed2(2);

    seed1(0) = 1.0;
    seed1(1) = 0.0;

    seed2(0) = 1.0;
    seed2(1) = 0.0;

    // COMPUTE FIRST ORDER DERIVATIVES:
    // --------------------------------
    integrator.setForwardSeed(1,seed1);
    integrator.integrateSensitivities();

    // COMPUTE SECOND ORDER DERIVATIVES:
    // ---------------------------------
    integrator.setForwardSeed(2,seed2);
    integrator.integrateSensitivities();


    // GET THE RESULTS
    // ---------------

	VariablesGrid differentialStates;
	integrator.getX( differentialStates );
	
	DVector Dx( 2 ), DDx( 2 );
	integrator.getForwardSensitivities( Dx,1 );
	integrator.getForwardSensitivities( DDx,2 );
	
	differentialStates.print( "x" );
	Dx.print( "Dx" );
	DDx.print( "DDx" );


    return 0;
}



