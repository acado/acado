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
 *    \file   examples/getting_started/simple_parameter_estimation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */


#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>


int main( ){

    USING_NAMESPACE_ACADO


    DifferentialState          phi, omega;    // the states of the pendulum
    Parameter                  l, alpha  ;    // its length and the friction
    const double               g = 9.81  ;    // the gravitational constant
    DifferentialEquation       f         ;    // the model equations
    Function                   h         ;    // the measurement function

    VariablesGrid measurements;                 // read the measurements
    measurements = readFromFile( "data.txt" );  // from a file.

//  --------------------------------------
    OCP ocp(measurements.getTimePoints());    // construct an OCP
    h << phi                             ;    // the state phi is measured
    ocp.minimizeLSQ( h, measurements )   ;    // fit h to the data

    f << dot(phi  ) == omega             ;    // a symbolic implementation
    f << dot(omega) == -(g/l) *sin(phi )      // of the model
                       - alpha*omega     ;    // equations

    ocp.subjectTo( f                    );    // solve OCP s.t. the model,
    ocp.subjectTo( 0.0 <= alpha <= 4.0  );    // the bounds on alpha
    ocp.subjectTo( 0.0 <=   l   <= 2.0  );    // and the bounds on l.
//  --------------------------------------

    GnuplotWindow window;
        window.addSubplot( phi  , "The angle phi", "time [s]", "angle [rad]" );
        window.addSubplot( omega, "The angular velocity dphi"                );
        window.addData( 0, measurements(0) );

    ParameterEstimationAlgorithm algorithm(ocp); // the parameter estimation
    algorithm << window;
    algorithm.solve();                           // algorithm solves the problem.


    return 0;
}



