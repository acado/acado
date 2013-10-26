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
 *    \file src/matrix_vector/vectorspace_element.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan vukov
 *    \date 2008 - 2013
 */

#include <acado/matrix_vector/vectorspace_element.hpp>
#include <acado/matrix_vector/vector.hpp>
#include <acado/matrix_vector/acado_mat_file.hpp>

#include <sstream>
#include <iomanip>

BEGIN_NAMESPACE_ACADO

using namespace std;

//
// PUBLIC MEMBER FUNCTIONS:
//

VectorspaceElement::VectorspaceElement( )
{
	dim = 0;
	element = 0;
}


VectorspaceElement::VectorspaceElement( uint _dim )
{
	dim = _dim;
	element = new double[ dim ];
}


VectorspaceElement::VectorspaceElement( uint _dim, const double* const _values )
{
	uint i;

	dim = _dim;
	element = new double[ dim ];

	for( i=0; i<_dim; ++i )
		operator()( i ) = _values[i];
}


VectorspaceElement::VectorspaceElement( const VectorspaceElement& rhs )
{
	uint i;

	dim = rhs.dim;
	element = new double[ dim ];

	for( i=0; i<dim; ++i )
		element[i] = rhs.element[i];
}


VectorspaceElement::VectorspaceElement( const Vector& rhs )
{
	uint i;

	dim = rhs.dim;
	element = new double[ dim ];

	for( i=0; i<dim; ++i )
		element[i] = rhs.element[i];
}


VectorspaceElement::~VectorspaceElement( )
{
	if ( element != 0 )
		delete[] element;
}


VectorspaceElement& VectorspaceElement::operator=( const VectorspaceElement& rhs )
{
	uint i;

    if ( this != &rhs )
    {
		if ( element != 0 )
			delete[] element;

		dim = rhs.dim;
		element = new double[ dim ];

		for( i=0; i<dim; ++i )
			element[i] = rhs.element[i];
    }

    return *this;
}


VectorspaceElement& VectorspaceElement::operator<<( double *rhs ){

    uint i;

    for( i = 0; i < dim; i++ )
        operator()(i) = rhs[i];

    return *this;
}


double* operator<<( double *lhs, VectorspaceElement &rhs ){

    uint i;

    for( i = 0; i < rhs.getDim(); i++ )
        lhs[i] = rhs.operator()(i);

    return lhs;
}


returnValue VectorspaceElement::init( uint _dim, double* _values )
{
	uint i;

	if ( element != 0 )
		delete[] element;

	dim = _dim;
	
	if ( dim > 0 )
		element = new double[ dim ];
	else 
		element = 0;

	if ( _values != 0 )
		for( i=0; i<dim; ++i )
			operator()( i ) = _values[i];

	return SUCCESSFUL_RETURN;
}


returnValue VectorspaceElement::append( const VectorspaceElement& arg )
{
	if ( arg.getDim( ) == 0 )
		return SUCCESSFUL_RETURN;

	VectorspaceElement old = *this;

	if ( init( old.getDim()+arg.getDim() ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNKNOWN_BUG );

	for( uint i=0; i<old.getDim(); ++i )
		operator()( i ) = old( i );
	for( uint i=0; i<arg.getDim(); ++i )
		operator()( old.getDim()+i ) = arg( i );

	return SUCCESSFUL_RETURN;
}


double VectorspaceElement::getNorm( VectorNorm norm ) const{

    VectorspaceElement scale(dim);
    scale.setAll(1.0);
    return getNorm( norm, scale );
}

double VectorspaceElement::getNorm( VectorNorm norm, const VectorspaceElement &scale  ) const{

    double value = 0.0;

    switch( norm ){

        case VN_L1:
             for( uint i=0; i<getDim(); ++i )
                 value += fabs( operator()( i )/scale(i) );
             break;

        case VN_L2:
             for( uint i=0; i<getDim(); ++i )
                 value += pow( operator()( i )/scale(i) , 2.0 );
             value = sqrt( value );
             break;

        case VN_LINF:
            for( uint i=0; i<getDim(); ++i )
                if ( fabs( operator()( i )/scale(i) ) > value )
                    value = fabs( operator()( i )/scale(i) );
            break;

         default:
            value = -1.0;
    }
    return value;
}

double* VectorspaceElement::getDoublePointer( )
{ return element; }

returnValue VectorspaceElement::print(	std::ostream& stream,
										const char* const name,
										const char* const startString,
										const char* const endString,
										uint width,
										uint precision,
										const char* const colSeparator,
										const char* const rowSeparator
										) const
{
	if (strlen( name ) > 0)
		stream << name << " = ";

	if (strlen(startString) > 0)
		stream << startString;

	for (unsigned i = 0; i < dim; ++i)
	{
		if (precision > 0)
			stream << setw( width ) << setprecision( precision ) << operator()( i );
		else
			stream << setw( width ) << width << (int)operator()( i );

		if (i < (dim - 1) && strlen( rowSeparator ) > 0)
			stream << rowSeparator;
	}

	return SUCCESSFUL_RETURN;
}

returnValue VectorspaceElement::print(	const char* const filename,
										const char* const name,
										const char* const startString,
										const char* const endString,
										uint width,
										uint precision,
										const char* const colSeparator,
										const char* const rowSeparator
										) const
{
	ofstream stream( filename );

	if ( stream.is_open() )
		return print(stream, name, startString, endString, width, precision,
				colSeparator, rowSeparator);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return SUCCESSFUL_RETURN;
}

returnValue VectorspaceElement::print(	std::ostream& stream,
										const char* const name,
										PrintScheme printScheme
										) const
{
	MatFile* matFile;

	switch ( printScheme )
	{
	case PS_MATLAB_BINARY:
		matFile = new MatFile;

		matFile->write(stream, *this, name);

		delete matFile;

		return SUCCESSFUL_RETURN;

	default:

		char* startString = 0;
		char* endString = 0;
		uint width = 0;
		uint precision = 0;
		char* colSeparator = 0;
		char* rowSeparator = 0;

		returnValue ret = getGlobalStringDefinitions(printScheme, &startString,
				&endString, width, precision, &colSeparator, &rowSeparator);
		if (ret != SUCCESSFUL_RETURN)
			return ret;

		returnValue status = print(stream, name, startString, endString, width,
				precision, colSeparator, rowSeparator);

		if ( startString != 0 )   delete[] startString;
		if ( endString != 0 )     delete[] endString;
		if ( colSeparator != 0 )  delete[] colSeparator;
		if ( rowSeparator != 0 )  delete[] rowSeparator;

		return status;
	}

	return SUCCESSFUL_RETURN;
}

returnValue VectorspaceElement::print(	const char* const filename,
										const char* const name = DEFAULT_LABEL,
										PrintScheme printScheme = PS_DEFAULT
										) const
{
	ofstream stream( filename );

	if ( stream.is_open() )
		return print(stream, name, printScheme);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return SUCCESSFUL_RETURN;
}

std::ostream& operator<<(std::ostream& stream, const VectorspaceElement& arg)
{
	if (arg.print( stream ) != SUCCESSFUL_RETURN)
		ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Cannot write to output stream.");

	return stream;
}

CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
