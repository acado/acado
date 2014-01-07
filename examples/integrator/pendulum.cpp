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
 *    \file examples/integrator/pendulum.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>



int main( ){

    USING_NAMESPACE_ACADO


    // Define a Right-Hand-Side:
    // -------------------------
    DifferentialState      phi;    // the angle phi
    DifferentialState     dphi;    // the first derivative of phi w.r.t time
    Control                  F;    // a force acting on the pendulum
    Parameter                l;    // the length of the pendulum

    const double m     = 1.0  ;    // the mass of the pendulum
    const double g     = 9.81 ;    // the gravitational constant
    const double alpha = 2.0  ;    // frictional constant

    IntermediateState    z;
    DifferentialEquation f;

    z = sin(phi);

    f << dot(phi ) == dphi;
    f << dot(dphi) == -(m*g/l)*z - alpha*dphi + F/(m*l);


    // DEFINE AN INTEGRATOR:
    // ---------------------

    IntegratorRK45 integrator( f );

	integrator.set( INTEGRATOR_PRINTLEVEL, HIGH );
	integrator.set( INTEGRATOR_TOLERANCE, 1.0e-6 );

    // DEFINE INITIAL VALUES:
    // ----------------------

    double x_start[2] = { 1.0, 0.0 };
    double u      [1] = { 0.0      };
    double p      [1] = { 1.0      };

    double t_start    =  0.0        ;
    double t_end      =  2.0        ;


    // START THE INTEGRATION:
    // ----------------------

	//integrator.freezeAll();
	integrator.integrate( t_start, t_end, x_start, 0, p, u );


    // GET THE RESULTS
    // ---------------

	VariablesGrid differentialStates;
	integrator.getX( differentialStates );
	
	differentialStates.print( "x" );


    return 0;
}



