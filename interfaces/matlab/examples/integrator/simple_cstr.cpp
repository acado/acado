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
 *    \file interfaces/matlab/integrator/models/simple_cstr.cpp
 *    \author Niels Haverbeke, Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 *
 */

void simple_cstr( DifferentialEquation *f ){

    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialState   PA;      // partial pressure of A
    DifferentialState   PB;      // partial pressure of B

    Control              u;

    Disturbance		w0;
    Disturbance		w1;

    Parameter            k;      // reaction rate constant

    IntermediateState z;

    z = k*pow(PA,2);

    *f <<  dot(PA) ==  -2*z + u + w0;
    *f <<  dot(PB) ==  z + w1;
}


