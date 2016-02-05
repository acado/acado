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
 *    \file src/code_generation/export_file.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_file.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//
ExportFile::ExportFile( ) : ExportStatementBlock( )
{}

ExportFile::ExportFile(	const std::string& _fileName,
						const std::string& _commonHeaderName,
						const std::string& _realString,
						const std::string& _intString,
						int _precision,
						const std::string& _commentString
						) : ExportStatementBlock( )
{
	setup( _fileName,_commonHeaderName,_realString,_intString,_precision,_commentString );
}

ExportFile::~ExportFile( )
{}


returnValue ExportFile::setup(	const std::string& _fileName,
                                const std::string& _commonHeaderName,
                                const std::string& _realString,
                                const std::string& _intString,
                                int _precision,
                                const std::string& _commentString
                                )
{
	fileName         = _fileName;
	commonHeaderName = _commonHeaderName;
	
	realString    = _realString;
	intString     = _intString;
	precision     = _precision;
	commentString = _commentString;
    
    return SUCCESSFUL_RETURN;
}


returnValue ExportFile::exportCode( ) const
{
	ofstream stream( fileName.c_str() );

	if (stream.good() == false)
		return ACADOERROR( RET_DOES_DIRECTORY_EXISTS );

	acadoPrintAutoGenerationNotice(stream, commentString);

	if ( commonHeaderName.size() )
		stream << "#include \"" << commonHeaderName << "\"\n\n\n";

	returnValue returnvalue = ExportStatementBlock::exportCode(stream, realString, intString, precision);

	stream.close();

	return returnvalue;
}

CLOSE_NAMESPACE_ACADO

// end of file.
