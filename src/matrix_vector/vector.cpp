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
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 31.05.2008
 */


#include <acado/matrix_vector/matrix_vector.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

Vector::Vector( ) : VectorspaceElement( )
{
}


Vector::Vector( uint _dim ) : VectorspaceElement( _dim )
{
}

Vector::Vector( uint _dim, const double* const _values ) : VectorspaceElement( _dim,_values )
{
}

Vector::Vector( FILE *file ) : VectorspaceElement()
{

    operator=(file);
}

//Vector::Vector( const char* filename ) : VectorspaceElement()
//{
//	FILE* file  = fopen(filename,"r");
//	if( !file ) ACADOERRORTEXT(RET_FILE_CAN_NOT_BE_OPENED,filename);
//	operator=(file);
//}


Vector::Vector( const Vector& rhs ) : VectorspaceElement( rhs )
{
}

Vector::Vector( const VectorspaceElement& rhs ) : VectorspaceElement( rhs )
{
}


Vector::~Vector( )
{
}


Vector& Vector::operator=( const Vector& rhs )
{

    if ( this != &rhs )
    {
		VectorspaceElement::operator=( rhs );
    }

    return *this;
}


Vector& Vector::operator=( FILE *rhs ){

    int     m;
    double *x;
    returnValue returnvalue;

    x = 0;
    returnvalue = allocateDoublePointerFromFile(rhs, &x, m);   

    if( returnvalue == SUCCESSFUL_RETURN ){
        VectorspaceElement::init( m, x );
        if( x != 0 ) free(x);
    }
    else{
        if( x != 0 ) free(x);
        ACADOINFO(returnvalue);
    }

    return *this;
}


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


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
