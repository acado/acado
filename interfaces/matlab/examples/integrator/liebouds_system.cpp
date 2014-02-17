/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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
 *    \file interfaces/matlab/models/liebouds_system.cpp
 *    \author Lieboud Van den Broeck, Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


void liebouds_system( DifferentialEquation *f ){

    DifferentialState x("x",10,1);

    Control         u1, u2;

    DMatrix A0(10,10);
    A0.setZero();

    DMatrix A1(10,10);
    A1.setZero();

    DMatrix A2(10,10);
    A2.setZero();

    DMatrix A3(10,10);
    A3.setZero();

    DMatrix A4(10,10);
    A4.setZero();

    DVector b1(10), b2(10);
    b1.setZero();
    b2.setZero();

    // DEFINE THE MATRICES AND VECTORS:
    // --------------------------------

    A0(0,1) =  1.0;
    A0(1,1) = -1.0;

    A0(2,3) =  6.3;
    A0(3,3) = -71.4;
    A0(4,3) =  62.6;
    A0(5,3) = -91.1;
    A0(6,3) =  54.4;
    A0(7,3) = -126.5;
    A0(8,3) =  82.6;
    A0(9,3) = -228.4;

    A0(4,4) = -2000.0;
    A0(5,4) =  2000.0;

    A0(2,5) =  6.3;
    A0(3,5) =  0.0   ;
    A0(4,5) = -1882.6;
    A0(5,5) =  1662.3;
    A0(6,5) =  54.4;
    A0(7,5) = -126.5;
    A0(8,5) =  82.6;
    A0(9,5) = -228.4;

    A0(6,6) = -2000.0;
    A0(7,6) =  2000.0;

    A0(2,7) =  6.3;
    A0(3,7) =  0.0   ;
    A0(4,7) =  0.0   ;
    A0(5,7) =  0.0   ;
    A0(6,7) = -1888.9;
    A0(7,7) =  1589.4;
    A0(8,7) =  82.6;
    A0(9,7) = -228.4;

    A0(8,8) = -2000.0;
    A0(9,8) =  2000.0;

    A0(2,7) =  6.3;
    A0(3,7) =  0.0   ;
    A0(4,7) =  0.0   ;
    A0(5,7) =  0.0   ;
    A0(6,7) =  0.0   ;
    A0(7,7) =  0.0   ;
    A0(8,7) = -1834.4;
    A0(9,7) =  1303.4;

    A1(4,3) = -1.4597;
    A1(5,3) =  2.2015;
    A1(6,3) = -0.0238;
    A1(7,3) =  0.0306;
    A1(8,3) = -0.0072;
    A1(9,3) =  0.0106;

    A1(4,5) = -2.9068;
    A1(5,5) =  5.7267;
    A1(6,5) = -0.0238;
    A1(7,5) =  0.0306;
    A1(8,5) = -0.0072;
    A1(9,5) =  0.0106;

    A1(8,7) = -0.0072;
    A1(9,7) =  0.0106;


    A2(4,3) =  0.0129;
    A2(5,3) = -0.0206;
    A2(6,3) =  0.0002;
    A2(7,3) = -0.0003;
    A2(8,3) =  0.0001;
    A2(9,3) = -0.0001;

    A2(4,5) =  0.0284;
    A2(5,5) = -0.0448;
    A2(6,5) =  0.0002;
    A2(7,5) = -0.0003;
    A2(8,5) =  0.0001;
    A2(9,5) = -0.0001;

    A2(8,7) =  0.0001;
    A2(9,7) = -0.0001;


    A3(4,3) = -0.0497e-03;
    A3(5,3) =  0.0835e-03;
    A3(6,3) = -0.0009e-03;
    A3(7,3) =  0.0012e-03;
    A3(8,3) = -0.0003e-03;
    A3(9,3) =  0.0004e-03;

    A3(4,5) = -0.1225e-03;
    A3(5,5) =  0.1688e-03;
    A3(6,5) = -0.0009e-03;
    A3(7,5) =  0.0012e-03;
    A3(8,5) = -0.0003e-03;
    A3(9,5) =  0.0004e-03;

    A3(8,7) = -0.0003e-03;
    A3(9,7) =  0.0004e-03;


    A4(4,3) =  0.0691e-06;
    A4(5,3) = -0.1238e-06;
    A4(6,3) =  0.0014e-06;
    A4(7,3) = -0.0019e-06;
    A4(8,3) =  0.0004e-06;
    A4(9,3) = -0.0006e-06;

    A4(4,5) =  0.1936e-06;
    A4(5,5) = -0.2445e-06;
    A4(6,5) =  0.0014e-06;
    A4(7,5) = -0.0019e-06;
    A4(8,5) =  0.0004e-06;
    A4(9,5) = -0.0006e-06;

    A4(8,7) =  0.0004e-06;
    A4(9,7) = -0.0006e-06;



    b1(1) = 10.0  ;
    b2(3) = 2000.0;

    IntermediateState rho  = x(0)    ;
    IntermediateState rho2 = rho*rho ;
    IntermediateState rho3 = rho2*rho;
    IntermediateState rho4 = rho3*rho;


    // Define the Right-Hand-Side:
    // ---------------------------

    *f << dot(x) == ( A0 + A1*rho + A2*rho2 + A3*rho3 + A4*rho4 )*x - b1*u1 - b2*u2;
}



