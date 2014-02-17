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
 *    \file examples/integrator/simple_dae.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE A RIGHT-HAND-SIDE:
    // -------------------------
    DifferentialState         x;
    AlgebraicState            z;
    Parameter               p,q;

    DifferentialEquation f;

    f << dot(x) == -p*x*x*z  ;
    f <<     0  ==  q*q - z*z;


    // DEFINE AN INTEGRATOR:
    // ---------------------

	IntegratorBDF integrator(f);

	integrator.set( INTEGRATOR_PRINTLEVEL, HIGH );

	
    // DEFINE INITIAL VALUES:
    // ----------------------

    double x0   =  1.0;
    double z0   =  1.000000;

    double pp[2] = { 1.0, 1.0 };

    double t0   = 0.0 ;
    double tend = 0.2 ;


    // START THE INTEGRATION:
    // ----------------------

	//integrator.freezeAll();
    integrator.integrate( t0, tend, &x0, &z0, pp );


    // GET THE RESULTS
    // ---------------

    VariablesGrid differentialStates;
    VariablesGrid algebraicStates   ;

    integrator.getX ( differentialStates );
    integrator.getXA( algebraicStates    );

	differentialStates.print( "x" );
	algebraicStates.print( "z" );


    return 0;
}
/* <<< end tutorial code <<< */


