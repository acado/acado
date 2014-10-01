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
 *    \file src/function/discretized_differential_equation.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/differential_equation.hpp>
#include <acado/function/discretized_differential_equation.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

DiscretizedDifferentialEquation::DiscretizedDifferentialEquation( )
                                :DifferentialEquation( )
{
	stepLength = 1.0;
	is_discretized = BT_TRUE;
}


DiscretizedDifferentialEquation::DiscretizedDifferentialEquation( const double &stepLength_ )
                                :DifferentialEquation( )
{
	stepLength = stepLength_;
	is_discretized = BT_TRUE;
}


DiscretizedDifferentialEquation::DiscretizedDifferentialEquation( const DiscretizedDifferentialEquation& arg )
                                :DifferentialEquation( arg ){

}


DiscretizedDifferentialEquation::~DiscretizedDifferentialEquation( ){

}


DiscretizedDifferentialEquation& DiscretizedDifferentialEquation::operator=(
                                 const DiscretizedDifferentialEquation& arg ){

    if ( this != &arg ){

        DifferentialEquation::operator=( arg );
    }
    return *this;
}


DifferentialEquation* DiscretizedDifferentialEquation::clone() const
{
    return new DiscretizedDifferentialEquation(*this);
}



CLOSE_NAMESPACE_ACADO

// end of file.
