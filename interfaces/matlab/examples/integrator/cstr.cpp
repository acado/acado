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
 *    \file interfaces/matlab/integrator/models/cstr.cpp
 *    \author Niels Haverbeke, Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 *
 */

void cstr( DifferentialEquation *f ){

    // Define a Right-Hand-Side:
    // -------------------------

    const double Ca0     = 10     ;

    DifferentialState   Ca;      // partial pressure of A
    DifferentialState   Cb;      // partial pressure of B

    Disturbance		w0;
    Disturbance		w1;

    Control              u;

    Parameter            k1;      // reaction rate constant for reaction 1
    Parameter            k2;      // reaction rate constant for reaction 2
    Parameter            k3;      // reaction rate constant for reaction 3

    *f << dot(Ca) ==  u*(Ca0 - Ca) - k1 * Ca - k3 * Ca * Ca + w0;
    *f << dot(Cb) ==  k1 * Ca - k2*Cb - u * Cb + w1;

}


