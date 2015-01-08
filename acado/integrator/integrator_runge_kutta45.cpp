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
 *    \file src/integrator/integrator_runge_kutta45.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2008-2010
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/differential_equation.hpp>
#include <acado/integrator/integrator.hpp>
#include <acado/integrator/integrator_runge_kutta.hpp>
#include <acado/integrator/integrator_runge_kutta45.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorRK45::IntegratorRK45( )
               :IntegratorRK(7,0.25){

    if( A != 0 ) initializeButcherTableau();
}

IntegratorRK45::IntegratorRK45( const DifferentialEquation &rhs_ )
               :IntegratorRK(rhs_,7,0.25){

    if( A != 0 ) initializeButcherTableau();
}

IntegratorRK45::IntegratorRK45( const IntegratorRK45& arg )
               :IntegratorRK(arg){ }

IntegratorRK45::~IntegratorRK45( ){ }

IntegratorRK45& IntegratorRK45::operator=( const IntegratorRK45& arg ){

    if( this != &arg ){
        IntegratorRK::operator=(arg);
    }
    return *this;
}

Integrator* IntegratorRK45::clone() const{

    return new IntegratorRK45(*this);
}


void IntegratorRK45::initializeButcherTableau(){

    A[0][0] = 0.0;
    A[0][1] = 0.0;
    A[0][2] = 0.0;
    A[0][3] = 0.0;
    A[0][4] = 0.0;
    A[0][5] = 0.0;
    A[0][6] = 0.0;

    A[1][0] = 1.0/5.0;
    A[1][1] = 0.0;
    A[1][2] = 0.0;
    A[1][3] = 0.0;
    A[1][4] = 0.0;
    A[1][5] = 0.0;
    A[1][6] = 0.0;

    A[2][0] = 3.0/40.0;
    A[2][1] = 9.0/40.0;
    A[2][2] = 0.0;
    A[2][3] = 0.0;
    A[2][4] = 0.0;
    A[2][5] = 0.0;
    A[2][6] = 0.0;

    A[3][0] = 44.0/45.0;
    A[3][1] = -56.0/15.0;
    A[3][2] = 32.0/9.0;
    A[3][3] = 0.0;
    A[3][4] = 0.0;
    A[3][5] = 0.0;
    A[3][6] = 0.0;

    A[4][0] = 19372.0/6561.0;
    A[4][1] = -25360.0/2187.0;
    A[4][2] = 64448.0/6561.0;
    A[4][3] = -212.0/729.0;
    A[4][4] = 0.0;
    A[4][5] = 0.0;
    A[4][6] = 0.0;

    A[5][0] = 9017.0/3168.0;
    A[5][1] = -355.0/33.0;
    A[5][2] = 46732.0/5247.0;
    A[5][3] = 49.0/176.0;
    A[5][4] = -5103.0/18656.0;
    A[5][5] = 0.0;
    A[5][6] = 0.0;

    A[6][0] = 35.0/384.0;
    A[6][1] = 0.0;
    A[6][2] = 500.0/1113.0;
    A[6][3] = 125.0/192.0;
    A[6][4] = -2187.0/6784.0;
    A[6][5] = 11.0/84.0;
    A[6][6] = 0.0;

    b4[0] = 5179.0/57600.0;
    b4[1] = 0.0;
    b4[2] = 7571.0/16695.0;
    b4[3] = 393.0/640.0;
    b4[4] = -92097.0/339200.0;
    b4[5] = 187.0/2100.0;
    b4[6] = 1.0/40.0;

    b5[0] = 35.0/384.0;
    b5[1] = 0.0;
    b5[2] = 500.0/1113.0;
    b5[3] = 125.0/192.0;
    b5[4] = -2187.0/6784.0;
    b5[5] = 11.0/84.0;
    b5[6] = 0.0;

    c[0] = 0.0;
    c[1] = 0.2;
    c[2] = 0.3;
    c[3] = 0.8;
    c[4] = 8.0/9.0;
    c[5] = 1.0;
    c[6] = 1.0;
}


CLOSE_NAMESPACE_ACADO

// end of file.
