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
 *    \file examples/integrator/harmonic_oscillator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado_integrators.hpp>
#include <acado_gnuplot.hpp>


const double k10 =  1.287e12;
const double k20 =  1.287e12;
const double k30 =  9.043e09;
const double E1  =  -9758.3;
const double E2  =  -9758.3;
const double E3  =  -8560.0;
const double H1  =      4.2;
const double H2  =    -11.0;
const double H3  =    -41.85;
const double rho =      0.9342;
const double Cp  =      3.01;
const double kw  =   4032.0;
const double AR  =      0.215;
const double VR  =     10.0;
const double mK  =      5.0;
const double CPK =      2.0;

const double cA0    =    5.1;
const double theta0 =  104.9;

const double FFs    =    14.19;  /* Feed Flow normed by VR: (dV/dt  / VR)*/
const double QdotKs = -1113.50;

const double cAs     =  2.1402105301746182e00;
const double cBs     =  1.0903043613077321e00;
const double thetas  =  1.1419108442079495e02;
const double thetaKs =  1.1290659291045561e02;


const double TIMEUNITS_PER_HOUR = 3600.0;


const double R_OMEGA = 90.0;


USING_NAMESPACE_ACADO


IntermediateState cstrModel( const DifferentialState &x,
                             const Control           &u  ){

    IntermediateState rhs(4);

    IntermediateState cA     = x(0);
    IntermediateState cB     = x(1);
    IntermediateState theta  = x(2);
    IntermediateState thetaK = x(3);

    IntermediateState k1, k2, k3;

    k1 = k10*exp(E1/(273.15 +theta));
    k2 = k20*exp(E2/(273.15 +theta));
    k3 = k30*exp(E3/(273.15 +theta));

    rhs(0) = (1/TIMEUNITS_PER_HOUR)*(u(0)*(cA0-cA) - k1*cA - k3*cA*cA);
    rhs(1) = (1/TIMEUNITS_PER_HOUR)* (- u(0)*cB + k1*cA - k2*cB);
    rhs(2) = (1/TIMEUNITS_PER_HOUR)*(u(0)*(theta0-theta) - (1/(rho*Cp)) *(k1*cA*H1 + k2*cB*H2 + k3*cA*cA*H3)+(kw*AR/(rho*Cp*VR))*(thetaK -theta)); 
    rhs(3) = (1/TIMEUNITS_PER_HOUR)*((1/(mK*CPK))*(u(1) + kw*AR*(theta-thetaK)));

    return rhs;
}



int main( ){


    // Define a Right-Hand-Side:
    // -------------------------
    DifferentialState x("", 4, 1), P("", 4, 4);
    Control           u("", 2, 1);

    IntermediateState rhs = cstrModel( x, u );

    DMatrix Q = zeros<double>(4,4);

    Q(0,0) = 0.2;
    Q(1,1) = 1.0;
    Q(2,2) = 0.5;
    Q(3,3) = 0.2;


    DMatrix R = zeros<double>(2,2);

    R(0,0) = 0.5;
    R(1,1) = 5e-7;

    DifferentialEquation f;
    f << dot(x) == rhs;
    f << dot(P) == getRiccatiODE( rhs, x, u, P, Q, R );



    // Define an integrator:
    // ---------------------

    IntegratorRK45 integrator( f );
    integrator.set( INTEGRATOR_PRINTLEVEL, MEDIUM );
	integrator.set( PRINT_INTEGRATOR_PROFILE, YES );
	
    // Define an initial value:
    // ------------------------
    //double x_ss[4] = { 2.14, 1.09, 114.2, 112.9 };
	double x_start[20] = { 1.0, 0.5, 100.0, 100.0, 1.0, 0.0, 0.0, 0.0,
                                                   0.0, 1.0, 0.0, 0.0,
                                                   0.0, 0.0, 1.0, 0.0,
                                                   0.0, 0.0, 0.0, 1.0 };

	double u_start[2] = { 14.19, -1113.5 };
//	double u_start[2] = { 10.00, -7000.0 };

	Grid timeInterval( 0.0, 5000.0, 100 );

    integrator.freezeAll();
    integrator.integrate( timeInterval, x_start, 0 ,0, u_start );


    // GET THE RESULTS
    // ---------------

    VariablesGrid differentialStates;
    integrator.getX( differentialStates );

	DVector PP = differentialStates.getLastVector();
	DMatrix PPP(4,4);
	for( int i=0; i<4; ++i )
		for( int j=0; j<4; ++j )
			PPP(i,j) = PP(4+i*4+j);
	PPP.print( "P1.txt","",PS_PLAIN );
//	PPP.printToFile( "P2.txt","",PS_PLAIN );

    GnuplotWindow window;
        window.addSubplot( differentialStates(0), "cA [mol/l]" );
        window.addSubplot( differentialStates(1), "cB [mol/l]" );
        window.addSubplot( differentialStates(2), "theta [C]" );
        window.addSubplot( differentialStates(3), "thetaK [C]" );

        window.addSubplot( differentialStates(4 ), "P11" );
        window.addSubplot( differentialStates(9 ), "P22" );
        window.addSubplot( differentialStates(14), "P33" );
        window.addSubplot( differentialStates(19), "P44" );
		
    window.plot();

    return 0;
}



