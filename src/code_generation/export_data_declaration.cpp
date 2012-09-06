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
 *    \file src/code_generation/export_data_declaration.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_data_declaration.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportDataDeclaration::ExportDataDeclaration( ) : ExportStatement( )
{
	data = 0;
}


ExportDataDeclaration::ExportDataDeclaration(	const ExportVariable& _data
												) : ExportStatement( )
{
	data = new ExportVariable( _data );
}


ExportDataDeclaration::ExportDataDeclaration(	const ExportIndex& _data
												) : ExportStatement( )
{
	data = new ExportIndex( _data );
}


ExportDataDeclaration::ExportDataDeclaration(	const ExportDataDeclaration& arg
												) : ExportStatement( arg )
{
	if ( arg.data != 0 )
		data = arg.data->clone( );
	else
		data = 0;
}


ExportDataDeclaration::~ExportDataDeclaration( )
{
	if ( data != 0 )
	{
		delete data;
		data = 0;
	}
}


ExportDataDeclaration& ExportDataDeclaration::operator=(	const ExportDataDeclaration& arg
															)
{
	if( this != &arg )
	{
		if ( data != 0 )
		{
			delete data;
			data = 0;
		}

		ExportStatement::operator= ( arg );

		if ( arg.data != 0 )
			data = arg.data->clone( );
		else
			data = 0;
	}

	return *this;
}



ExportStatement* ExportDataDeclaration::clone( ) const
{
	return new ExportDataDeclaration(*this);
}




returnValue ExportDataDeclaration::exportCode(	FILE* file,
												const String& _realString,
												const String& _intString,
												int _precision
												) const
{
	if ( data == 0 )
		return SUCCESSFUL_RETURN;
	
	return data->exportDataDeclaration( file,_realString,_intString,_precision );
}



//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
