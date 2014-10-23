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


#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>

/* >>> start tutorial code >>> */


void myAcadoDifferentialEquation1( double *xx, double *f, void *user_data ){

    //double t = xx[0];
    double x = xx[1];
    //double l = xx[2];
    double z = xx[3];
    double u = xx[4];

    f[0] = -x + 0.5*x*x + u + 0.5*z;
    f[1] =  x*x + 3.0*u*u          ;
    f[2] =  z + exp(z) - 1.0 + x;
}




int main( ){

    USING_NAMESPACE_ACADO

    TIME autotime;
    DifferentialState x("", 2, 1);
    AlgebraicState z;
    Control u;
    DifferentialEquation f1;
    IntermediateState setc_is_1(5);
    setc_is_1(0) = autotime;
    setc_is_1(1) = x(0);
    setc_is_1(2) = x(1);
    setc_is_1(3) = z;
    setc_is_1(4) = u;


    CFunction cLinkModel_1( 3, myAcadoDifferentialEquation1 );
    f1 << cLinkModel_1(setc_is_1);


    double dconstant1 = 0.0;
    double dconstant2 = 5.0;
    int dconstant3 = 10;
    OCP ocp1(dconstant1, dconstant2, dconstant3);
    ocp1.minimizeMayerTerm(x(1));
    ocp1.subjectTo(f1);
    ocp1.subjectTo(AT_START, x(0) == 1.0 );
    ocp1.subjectTo(AT_START, x(1) == 0.0 );


    GnuplotWindow window;
        window.addSubplot(x(0),"DIFFERENTIAL STATE  x");
        window.addSubplot(z,"ALGEBRAIC STATE  z"   );
        window.addSubplot(u,"CONTROL u"            );


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-05 );
    algo1.set( RELAXATION_PARAMETER, 1.500000E+00 );

 // algo1.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
 // 
    algo1 << window;

    algo1.solve();

    return 0;
}
/* <<< end tutorial code <<< */


