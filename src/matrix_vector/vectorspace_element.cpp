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
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 31.05.2008
 */


#include <acado/matrix_vector/vectorspace_element.hpp>
#include <acado/matrix_vector/vector.hpp>



BEGIN_NAMESPACE_ACADO



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



returnValue operator<<( FILE *file, VectorspaceElement& arg ){

    return arg.printToFile(file);
}


returnValue VectorspaceElement::print(	const char* const name,
										const char* const startString,
										const char* const endString,
										uint width,
										uint precision,
										const char* const colSeparator,
										const char* const rowSeparator
										) const
{
	char* string = 0;

	printToString( &string, name,startString,endString,width,precision,colSeparator,rowSeparator );
	acadoPrintf( "%s",string );

	if ( string != 0 )
	  delete[] string;

	return SUCCESSFUL_RETURN;
}


returnValue VectorspaceElement::print(	const char* const name,
										PrintScheme printScheme
										) const
{
	char* string = 0;

	printToString( &string, name,printScheme );
	acadoPrintf( "%s",string );

	if ( string != 0 )
	  delete[] string;

	return SUCCESSFUL_RETURN;
}


returnValue VectorspaceElement::printToFile(	const char* const filename,
												const char* const name,
												const char* const startString,
												const char* const endString,
												uint width,
												uint precision,
												const char* const colSeparator,
												const char* const rowSeparator
												) const
{
	FILE* file = fopen( filename,"w+" );

	if ( file == 0 )
		return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

	printToFile( file, name,startString,endString,width,precision,colSeparator,rowSeparator );
	fclose( file );

	return SUCCESSFUL_RETURN;
}


returnValue VectorspaceElement::printToFile(	FILE* file,
												const char* const name,
												const char* const startString,
												const char* const endString,
												uint width,
												uint precision,
												const char* const colSeparator,
												const char* const rowSeparator
												) const
{
	char* string = 0;

	printToString( &string, name,startString,endString,width,precision,colSeparator,rowSeparator );
	acadoFPrintf( file,"%s",string );

	if ( string != 0 )
	  delete[] string;

	return SUCCESSFUL_RETURN;
}



returnValue VectorspaceElement::printToFile(	const char* const filename,
												const char* const name,
												PrintScheme printScheme
												) const
{
	FILE* file = 0;
	MatFile* matFile = 0;
	
	switch ( printScheme )
	{
		case PS_MATLAB_BINARY:
			matFile = new MatFile;
			
			matFile->open( filename );
			matFile->write( *this,name );
			matFile->close( );
			
			delete matFile;
			return SUCCESSFUL_RETURN;

		default:
			file = fopen( filename,"w+" );

			if ( file == 0 )
				return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

			printToFile( file, name,printScheme );

			fclose( file );
			return SUCCESSFUL_RETURN;
	}
}


returnValue VectorspaceElement::printToFile(	FILE* file,
												const char* const name,
												PrintScheme printScheme
												) const
{
	char* string = 0;

	printToString( &string, name,printScheme );
	acadoFPrintf( file,"%s",string );

	if ( string != 0 )
	  delete[] string;

	return SUCCESSFUL_RETURN;
}


returnValue VectorspaceElement::printToString(	char** string,
												const char* const name,
												const char* const startString,
												const char* const endString,
												uint width,
												uint precision,
												const char* const colSeparator,
												const char* const rowSeparator,
												BooleanType allocateMemory
												) const
{
	uint i;

	/* determine length of single component */
	uint componentLength = width;

	// 0.e-0000
	if ( componentLength < (9 + (uint)precision) )
		componentLength = 9 + precision;

	char* componentString = new char[componentLength];

	/* determine length of whole string */
	if ( allocateMemory == BT_TRUE )
	{
		uint stringLength = determineStringLength( name,startString,endString,
												   width,precision,colSeparator,rowSeparator );

		*string = new char[stringLength];

		for( i=0; i<stringLength; ++i )
			(*string)[i] = '\0';
	}

	if ( getStringLength(name) > 0 )
	{
		strcat( *string,name );
		strcat( *string," = " );
	}

	if ( getStringLength(startString) > 0 )
		strcat( *string,startString );

	int writtenChars;

	for( i=0; i<getDim( ); ++i )
	{
		if ( precision > 0 )
			writtenChars = sprintf( componentString,"%*.*e",width,precision,operator()( i ) );
		else
			writtenChars = sprintf( componentString,"%*.d",width,(int)operator()( i ) );

		if ( ( writtenChars < 0 ) || ( (uint)writtenChars+1 > componentLength ) )
		{
			delete[] componentString;
			return ACADOERROR( RET_UNKNOWN_BUG );
		}

		strcat( *string,componentString );
		
		if ( i < getDim( )-1 )
			if ( getStringLength(rowSeparator) > 0 )
				strcat( *string,rowSeparator );
	}

	if ( getStringLength(endString) > 0 )
		strcat( *string,endString );

	delete[] componentString;

	return SUCCESSFUL_RETURN;
}


returnValue VectorspaceElement::printToString(	char** string,
												const char* const name,
												PrintScheme printScheme,
												BooleanType allocateMemory
												) const
{
	char* startString = 0;
	char* endString = 0;
	uint width = 0;
	uint precision = 0;
	char* colSeparator = 0;
	char* rowSeparator = 0;

	returnValue returnvalue;
	returnvalue = getGlobalStringDefinitions( printScheme,&startString,&endString,
											  width,precision,&colSeparator,&rowSeparator );

	if ( returnvalue == SUCCESSFUL_RETURN )
	{
		returnvalue = printToString( string,name,startString,endString,width,precision,colSeparator,rowSeparator,allocateMemory );
	}

	if ( startString != 0 )   delete[] startString;
	if ( endString != 0 )     delete[] endString;
	if ( colSeparator != 0 )  delete[] colSeparator;
	if ( rowSeparator != 0 )  delete[] rowSeparator;

	return returnvalue;
}


uint VectorspaceElement::determineStringLength(	const char* const name,
												const char* const startString,
												const char* const endString,
												uint width,
												uint precision,
												const char* const colSeparator,
												const char* const rowSeparator
												) const
{
	uint componentLength = width;

	// 0.e-0000
	if ( componentLength < (9 + (uint)precision) )
		componentLength = 9 + precision;
	

	// allocate string of sufficient size (being quite conservative)
	uint stringLength;

	if ( getDim( ) > 0 )
	{
		stringLength =	1
						+ getDim( ) * getStringLength(startString)
						+ getDim( ) * componentLength
						+ ( getDim( )-1 ) * getStringLength(rowSeparator)
						+ getDim( ) * getStringLength(endString);
	}
	else
	{
		stringLength =	1
						+ getStringLength(startString) 
						+ getStringLength(endString);
	}

	if ( getStringLength(name) > 0 )
		stringLength += getStringLength(name)+3;

	return stringLength; 
}



double* VectorspaceElement::getDoublePointer( )
{
	return element;
}



//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
