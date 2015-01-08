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
  *    \file   examples/ocp/bioreactor.cpp
  *    \author Boris Houska, Filip Logist, Rien Quirynen
  *    \date   2014
  */

#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState     X,S,P;
    Control               Sf   ;
    IntermediateState     mu   ;
    DifferentialEquation  f    ;
    
    const double D       = 0.15;
    const double Ki      = 22.0;
    const double Km      = 1.2 ; 
    const double Pm      = 50.0;
    const double Yxs     = 0.4 ;
    const double alpha   = 2.2 ;
    const double beta    = 0.2 ;
    const double mum     = 0.48;
    // const double Sfbar   = 32.9;
    const double Sfmin   = 28.7;
    const double Sfmax   = 40.0;
    // const double Xbarmax = 5.8 ;

    const double t_start =  0.0;
    const double t_end   = 48.0;

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------

    mu = mum*(1.-P/Pm)*S/(Km+S+pow(S,2)/Ki);
    
    f << dot(X) == -D*X+mu*X;
    f << dot(S) == D*(Sf-S)-(mu/Yxs)*X;
    f << dot(P) == -D*P+(alpha*mu+beta)*X;


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 20 );
    ocp.minimizeLagrangeTerm( D*(Sf-P) );  // weight this with the physical cost!!!
    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, X ==  6.5 );
    ocp.subjectTo( AT_START, S == 12.0 );
    ocp.subjectTo( AT_START, P == 22.0 );
    
//     ocp.subjectTo( 0.0, X , -X, 0.0 );
//     ocp.subjectTo( 0.0, S , -S, 0.0 );
//     ocp.subjectTo( 0.0, P , -P, 0.0 );

    ocp.subjectTo( Sfmin <= Sf <= Sfmax );


    // DEFINE A PLOT WINDOW:
    // ---------------------
    GnuplotWindow window;
        window.addSubplot( X ,"X"  );
        window.addSubplot( S ,"S"  );
        window.addSubplot( P ,"P"  );
        window.addSubplot( Sf,"Sf" );


    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    
    algorithm.initializeDifferentialStates("s2.txt");
    algorithm.initializeControls          ("c2.txt");
    
    algorithm.set( MAX_NUM_ITERATIONS, 20 );
    algorithm.set( KKT_TOLERANCE, 1e-8 );
    algorithm << window;
    
    algorithm.solve();

    VariablesGrid s3,c3;
    algorithm.getDifferentialStates(s3);
    algorithm.getControls          (c3);
    
    std::ofstream stream1( "s3.txt" );
	
    s3.print(stream1, 0," "," ", 16, 16, " ", "\n" );
//     s3.print(stream, " ", PS_PLAIN );
    stream1.close();
    
    return 0;
}
/* <<< end tutorial code <<< */

