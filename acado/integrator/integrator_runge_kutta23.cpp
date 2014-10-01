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
 *    \file src/integrator/integrator_runge_kutta23.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 31.12.2008
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/differential_equation.hpp>
#include <acado/integrator/integrator.hpp>
#include <acado/integrator/integrator_runge_kutta.hpp>
#include <acado/integrator/integrator_runge_kutta23.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorRK23::IntegratorRK23( )
               :IntegratorRK(3,0.5){

    if( A != 0 ) initializeButcherTableau();
}

IntegratorRK23::IntegratorRK23( const DifferentialEquation &rhs_ )
               :IntegratorRK(rhs_,3,0.5){

    if( A != 0 ) initializeButcherTableau();
}

IntegratorRK23::IntegratorRK23( const IntegratorRK23& arg )
               :IntegratorRK(arg){ }

IntegratorRK23::~IntegratorRK23( ){ }

IntegratorRK23& IntegratorRK23::operator=( const IntegratorRK23& arg ){

    if( this != &arg ){
        IntegratorRK::operator=(arg);
    }
    return *this;
}

Integrator* IntegratorRK23::clone() const{

    return new IntegratorRK23(*this);
}

void IntegratorRK23::initializeButcherTableau(){

    A[0][0] = 0.0;
    A[0][1] = 0.0;
    A[0][2] = 0.0;

    A[1][0] = 1.0/2.0;
    A[1][1] = 0.0;
    A[1][2] = 0.0;

    A[2][0] = -1.0;
    A[2][1] =  2.0;
    A[2][2] =  0.0;

    b4[0] = 0.0;
    b4[1] = 1.0;
    b4[2] = 0.0;

    b5[0] = 1.0/6.0;
    b5[1] = 4.0/6.0;
    b5[2] = 1.0/6.0;

    c[0] = 0.0;
    c[1] = 1.0/2.0;
    c[2] = 1.0;

    TOL = 0.01;
}


CLOSE_NAMESPACE_ACADO

// end of file.
