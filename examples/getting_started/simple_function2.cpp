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
 *    \file   examples/getting_started/simple_function2.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2009
 */

#include <acado_toolkit.hpp>


int main( ){

    USING_NAMESPACE_ACADO

    DMatrix             A(3,3);
    DVector             b(3);
    DifferentialState  x("", 3, 1);
    Function           f;

    A.setZero() ;
    A(0,0) = 1.0;  A(1,1) = 2.0;  A(2,2) = 3.0;
    b(0)   = 1.0;  b(1)   = 1.0;  b(2)   = 1.0;

    f << A*x + b;

	DVector x0(3), dummy(3);
	x0.setAll( 1.0 );
	dummy.setZero( );
	//f.evaluate( 0.0,&x0,&dummy,MEDIUM ); DOES NOT COMPILE

    return 0;
}


