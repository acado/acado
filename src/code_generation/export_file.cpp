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
 *    \file src/code_generation/export_file.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_file.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportFile::ExportFile(	const String& _fileName,
						const String& _commonHeaderName,
						const String& _realString,
						const String& _intString,
						int _precision,
						const String& _commentString
						) : ExportStatementBlock( )
{
	fileName         = _fileName;
	commonHeaderName = _commonHeaderName;
	
	realString    = _realString;
	intString     = _intString;
	precision     = _precision;
	commentString = _commentString;
}


ExportFile::ExportFile(	const ExportFile& arg
						) : ExportStatementBlock( arg )
{
	copy( arg );
}


ExportFile::~ExportFile( )
{
}


ExportFile& ExportFile::operator=(	const ExportFile& arg
									)
{
	if( this != &arg )
	{
		ExportStatementBlock::operator=( arg );
		copy( arg );
	}
	
	return *this;
}



returnValue ExportFile::exportCode( ) const
{
	FILE* file = openFile( );

	if ( file == 0 )
		return ACADOERROR( RET_DOES_DIRECTORY_EXISTS );

	returnValue returnvalue = ExportStatementBlock::exportCode( file,realString,intString,precision );

	if ( file != 0 )
		fclose( file );

	return returnvalue;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue ExportFile::copy(	const ExportFile& arg
								)
{
	fileName         = arg.fileName;
	commonHeaderName = arg.commonHeaderName;

	realString    = arg.realString;
	intString     = arg.intString;
	precision     = arg.precision;
	commentString = arg.commentString;

	return SUCCESSFUL_RETURN;
}



FILE* ExportFile::openFile( ) const
{
	FILE* file = acadoFOpen( fileName.getName(), "w" );
	if ( file == 0 )
		return 0;

	if ( commentString.isEmpty() == BT_TRUE )
		acadoPrintAutoGenerationNotice( file );
	else
		acadoPrintAutoGenerationNotice( file,commentString.getName() );

	if ( commonHeaderName.isEmpty() == BT_FALSE )
		acadoFPrintf( file, "#include \"%s\"\n\n\n",commonHeaderName.getName() );
	
	return file;
}


CLOSE_NAMESPACE_ACADO

// end of file.
