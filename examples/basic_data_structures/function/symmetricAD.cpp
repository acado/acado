/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file examples/function/symmetricAD.cpp
 *    \author Boris Houska, Rien Quirynen
 *    \date 2014
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE VALRIABLES:
    // ---------------------------
//     DifferentialState x(2);
 //   Function          f;

    // DEFINE A TEST FUNCTION:
    // -----------------------
    
//     f << (sin (x)).ADsymmetric( x,1.,1.);
//     f << (cos (x)).ADsymmetric( x,1.,1.);
//     f << (tan (x)).ADsymmetric( x,1.,1.);
//     f << (asin(x)).ADsymmetric( x,1.,1.);
//     f << (acos(x)).ADsymmetric( x,1.,1.);
//     f << (atan(x)).ADsymmetric( x,1.,1.);
//     f << (exp (x)).ADsymmetric( x,1.,1.);
//     f << (log (x)).ADsymmetric( x,1.,1.);

//     f << (pow (x,2)).ADsymmetric( x,1.,1.);
//     f << (pow (x,3)).ADsymmetric( x,1.,1.);
    
//     f << (sin(x)+cos(x)).ADsymmetric( x,1.,1.);
//     f << (sin(x)*cos(x)).ADsymmetric( x,1.,1.);
//     f << (sin(x)-cos(x)).ADsymmetric( x,1.,1.);
//     f << (sin(x)/cos(x)).ADsymmetric( x,1.,1.);
//     f << (pow(sin(x),cos(x))).ADsymmetric( x,1.,1.);
    
//     IntermediateState a = cos(x(0));
//     IntermediateState b = log(x(1));
    
//     Matrix S = eye(2);    //   S^T*(l*f'')*S
//     Vector l(2);
//     l(0) = 1.0;
//     l(1) = 5.0;
//     
//     IntermediateState c(2);
//     
//     c(0) = a*b;
//     c(1) = sin(a);
//     
//     f << c.ADsymmetric( x, S, l );
    
    
    DifferentialState   xT;     // the trolley position
    DifferentialState   vT;     // the trolley velocity
    IntermediateState   aT;     // the trolley acceleration
    DifferentialState   xL;     // the cable length
    DifferentialState   vL;     // the cable velocity
    IntermediateState   aL;     // the cable acceleration
    DifferentialState   phi;    // the excitation angle
    DifferentialState   omega;  // the angular velocity
        
    DifferentialState   uT;     // trolley velocity control
    DifferentialState   uL;     // cable velocity control

    Control             duT;
    Control             duL;

	//
    // DEFINE THE PARAMETERS:
    //
    const double      tau1 = 0.012790605943772;
    const double      a1   = 0.047418203070092;
    const double      tau2 = 0.024695192379264;
    const double      a2   = 0.034087337273386;
    const double      g = 9.81;       		
    const double      c = 0.0;        		
    const double      m = 1318.0;     		

    //
    // DEFINE THE MODEL EQUATIONS:
    //
    DifferentialEquation   f;
    aT = -1.0 / tau1 * vT + a1 / tau1 * uT;
    aL = -1.0 / tau2 * vL + a2 / tau2 * uL;

    Expression arg;
    
    arg << xT;
    arg << vT;
    arg << xL;
    arg << phi;
    arg << omega;
    arg << duT;
    
    IntermediateState a = - 1.0/xL*(-g*sin(phi)-aT*cos(phi) 
						-2*vL*omega-c*omega/(m*xL)) + duT;
    
    f << backwardDerivative( a, arg );
    
    
    
    FILE *file = fopen("symmetricAD_output.txt", "w" );
    file << f;
    fclose(file);

    return 0;
}
/* <<< end tutorial code <<< */

