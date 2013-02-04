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
 *    \file examples/basic_data_structures/ode_taylor_expansion.cpp
 *    \author Boris Houska
 *    \date 2013
 */

#include <time.h>

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>


/* >>> start tutorial code >>> */
int main( ){

    USING_NAMESPACE_ACADO

    DifferentialState     x(2);
	TIME                  t;
    
	IntermediateState rhs(2);
	rhs(0) = -t*x(0);
	rhs(1) = -x(1);
	
	DifferentialEquation ode;
	
	ode << dot(x) == rhs;
	
	Function f;
    f << ode.getODEexpansion( 4 );

	TevaluationPoint<double> z(f);
	
    Tmatrix<double> xx(2);
    xx(0) = 1.1;
    xx(1) = 1.0;
    Tmatrix<double> tt(1);
	tt(0) = 0.0;
    z.setT( tt );
    z.setX( xx );

	Tmatrix<double> result = f.evaluate( z );
	
	std::cout << "result = \n" << result << "\n";

    return 0;
}
/* <<< end tutorial code <<< */


