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
 *    \file examples/integrator/simple_dae_c.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, David Ariens
 *    \date 2010
 */


#include <acado_integrators.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */

void ffcn_model( double *xx, double *f, void *user_data ){

    double        x =  xx[ 0];
    double        z =  xx[ 1];
    double        p =  xx[ 2];
    double        q =  xx[ 3];

    f[0] = -p*x*x*z;
    f[1] = q*q - z*z + 0.1*x;


//    f << dot(x) == -p*x*x*z  ;
//    f <<     0  ==  q*q - z*z;

}




int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE A RIGHT-HAND-SIDE:
    // -------------------------
    DifferentialState         x;
    AlgebraicState            z;
    Parameter               p,q;


    IntermediateState is(4);
     is(0) = x;
     is(1) = z;
     is(2) = p;
     is(3) = q;

    CFunction simpledaeModel( 2, ffcn_model );

    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialEquation f;

    f << simpledaeModel(is);



    // DEFINE AN INTEGRATOR:
    // ---------------------
    IntegratorBDF integrator(f);


    // DEFINE INITIAL VALUES:
    // ----------------------
    double x0   =  1.0;
    double z0   =  1.000000;

    double pp[2] = { 1.0, 1.0 };

    Grid interval( 0.0, 1.0, 100 );


    // START THE INTEGRATION:
    // ----------------------
    integrator.integrate( interval, &x0, &z0, pp );

    VariablesGrid differentialStates;
    VariablesGrid algebraicStates   ;
    VariablesGrid intermediateStates;

    integrator.getX ( differentialStates );
    integrator.getXA( algebraicStates    );
    integrator.getI ( intermediateStates );

    GnuplotWindow window;
        window.addSubplot( differentialStates(0) );
        window.addSubplot( algebraicStates   (0) );

    window.plot();


    return 0;
}
/* <<< end tutorial code <<< */


