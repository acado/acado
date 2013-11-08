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

#include <iostream>
#include <iomanip>

BEGIN_NAMESPACE_ACADO

using namespace std;

//
// PUBLIC MEMBER FUNCTIONS:
//

VectorspaceElement::VectorspaceElement( )
	: data()
{}


VectorspaceElement::VectorspaceElement( uint _dim )
	: data(_dim, 0.0)
{}


VectorspaceElement::VectorspaceElement( uint _dim, const double* const _values )
	: data(_values, _values + _dim)
{}

VectorspaceElement::VectorspaceElement( const Vector& rhs )
	: data( rhs.data )
{}

VectorspaceElement::~VectorspaceElement( )
{}

returnValue VectorspaceElement::init( uint _dim, double* _values )
{
	if ( data.size() )
		data.clear();

	if ( _dim > 0 )
		data.resize( _dim );

	if ( _values != 0 )
		data.assign(_values, _values + _dim);

	return SUCCESSFUL_RETURN;
}

returnValue VectorspaceElement::append( const VectorspaceElement& arg )
{
	data.insert(data.end(), arg.data.begin(), arg.data.end());

	return SUCCESSFUL_RETURN;
}


double VectorspaceElement::getNorm( VectorNorm norm ) const{

    VectorspaceElement scale( getDim() );
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
{
	return data.data();
}

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
	if (name != NULL && strlen( name ) > 0)
		stream << name << " = ";

	if (startString != NULL && strlen(startString) > 0)
		stream << startString;

	if (precision > 0)
		stream << setw( width ) << setprecision( precision ) << scientific;
	else
		stream << setw( width );

	for (unsigned i = 0; i < getDim(); ++i)
	{
		if (precision > 0)
			stream << operator()( i );
		else
			stream << (int)operator()( i );

		if (i < (getDim() - 1) && rowSeparator != NULL && strlen( rowSeparator ) > 0)
			stream << rowSeparator;
	}
	if (endString != NULL && strlen(endString) > 0)
		stream << endString;

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
	MatFile* matFile = 0;

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
	returnValue status;

	if ( stream.is_open() )
		status = print(stream, name, printScheme);
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

std::ostream& operator<<(std::ostream& stream, const VectorspaceElement& arg)
{
	if (arg.print( stream ) != SUCCESSFUL_RETURN)
		ACADOERRORTEXT(RET_INVALID_ARGUMENTS, "Cannot write to output stream.");

	return stream;
}

returnValue VectorspaceElement::read( std::istream& stream )
{
	vector< double > tmp;
	stream >> tmp;

	return init(tmp.size(), tmp.data());
}

returnValue VectorspaceElement::read(	const char* const filename
										)
{
	ifstream stream( filename );
	returnValue status;

	if (stream.is_open())
		status = read( stream );
	else
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	stream.close();

	return status;
}

std::istream& operator>>(std::istream& stream, VectorspaceElement& arg)
{
	if (arg.read( stream ) != SUCCESSFUL_RETURN)
		ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	return stream;
}

CLOSE_NAMESPACE_ACADO

/*
 *	end of file
 */
