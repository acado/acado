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
 *    \file examples/integrator/models/glycemia.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 *
 */


void glycemia( DifferentialEquation *f ){

    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialState   G ;		// blood glucose concentration
    DifferentialState   I ;		// blood insulin concentration
    DifferentialState   X ;		// effect of insulin on net glucose disappearance
    DifferentialState   I2;		// effect of endogenous insulin

    Control             FI;		// exogenous insulin flow

    Disturbance		w0;
    Disturbance		w1;
    Disturbance		w2;
    Disturbance		w3;

    Parameter           FG;		// carbohydrate calories flow
    Parameter           P1;
    Parameter           P2;
    Parameter           P3;
    Parameter           C1;
    Parameter           C2;
    Parameter           C3;
    Parameter           C4;
    Parameter           C5;
    Parameter            n;
    Parameter           alpha;
    Parameter           gamma;

    const double s     = 0.1     ;

    IntermediateState z;

    z = exp(I2/s);

    *f <<  dot(G) == (-P1 - X) * G   +   P1 * C1  +  FG/C2 + w0;
    *f <<  dot(I) == alpha * s *log(1 + z) - n*(I-C4)  +  FI/C3 + w1;
    *f <<  dot(X) == -P2 * X + P3 * 0.001 * (I-C4) + w2;
    *f <<  dot(I2) == gamma * (G - C5) - n * (I2) + w3;
}

