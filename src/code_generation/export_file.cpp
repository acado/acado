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

ExportFile::ExportFile(	const std::string& _fileName,
						const std::string& _commonHeaderName,
						const std::string& _realstd::string,
						const std::string& _intstd::string,
						int _precision,
						const std::string& _commentstd::string
						) : ExportStatementBlock( )
{
	fileName         = _fileName;
	commonHeaderName = _commonHeaderName;
	
	realstd::string    = _realstd::string;
	intstd::string     = _intstd::string;
	precision     = _precision;
	commentstd::string = _commentstd::string;
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

	returnValue returnvalue = ExportStatementBlock::exportCode( file,realstd::string,intstd::string,precision );

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

	realstd::string    = arg.realstd::string;
	intstd::string     = arg.intstd::string;
	precision     = arg.precision;
	commentstd::string = arg.commentstd::string;

	return SUCCESSFUL_RETURN;
}



FILE* ExportFile::openFile( ) const
{
	FILE* file = acadoFOpen( fileName.getName(), "w" );
	if ( file == 0 )
		return 0;

	if ( commentstd::string.isEmpty() == BT_TRUE )
		acadoPrintAutoGenerationNotice( file );
	else
		acadoPrintAutoGenerationNotice( file,commentstd::string.getName() );

	if ( commonHeaderName.isEmpty() == BT_FALSE )
		acadoFPrintf( file, "#include \"%s\"\n\n\n",commonHeaderName.getName() );
	
	return file;
}


CLOSE_NAMESPACE_ACADO

// end of file.
