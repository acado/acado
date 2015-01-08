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
 *    \file examples/integrator/linking_c_functions2.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>



void my_function( double *x_, double *f, void *userData ){

    double          t =  x_[ 0];    // the time
    double          x =  x_[ 1];    // the differential state

    f[0] = x*x + t;
    f[1] = t;
}


int main( ){

    USING_NAMESPACE_ACADO

    CFunction map( 2, my_function );

    TIME t;
    DifferentialState y;

    IntermediateState x(2);

    x(0) = t;
    x(1) = 2*y+1;

    Function f;

    f << map(2*x)*t;
    f << t;
    f << map(x);

    IntermediateState z;

    z = euclidean_norm( map(x) );

    f << z + z;

    EvaluationPoint zz(f);

    DVector xx(1);
    double tt   ;

    xx(0) = 2.0;
    tt    = 1.0;

    zz.setT( tt );
    zz.setX( xx );


    // EVALUATE f AT THE POINT  (tt,xx):
    // ---------------------------------
    DVector result = f.evaluate( zz );


    // PRINT THE RESULT:
    // -----------------
    result.print();

    return 0;
}



