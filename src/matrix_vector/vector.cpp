/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
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
 *    \file src/matrix_vector/vector.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2008 - 2013
 */

#include <acado/matrix_vector/matrix_vector.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

Vector::Vector( )
	: VectorspaceElement( )
{}

Vector::Vector( uint _dim )
	: VectorspaceElement( _dim )
{}

Vector::Vector( uint _dim, const double* const _values )
	: VectorspaceElement(_dim, _values)
{}

Vector::Vector( const VectorspaceElement& rhs )
	: VectorspaceElement( rhs )
{}

Vector::~Vector( )
{}

returnValue Vector::append( const Vector& arg )
{
	return VectorspaceElement::append( arg );
}

Vector operator-(const Vector &arg){

    uint i;

    Vector tmp(arg.getDim());

    for( i = 0; i < arg.getDim(); i++ )
        tmp(i) = -arg(i);

    return tmp;
}

CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
