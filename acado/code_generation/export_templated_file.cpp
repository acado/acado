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
 *    \file src/code_generation/export_templated_file.cpp
 *    \author Milan Vukov
 *    \date 2012 - 2013
 */

#include <acado/code_generation/export_templated_file.hpp>
#include <acado/code_generation/templates/templates.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO
        
ExportTemplatedFile::ExportTemplatedFile( ) : ExportFile( )
{}


ExportTemplatedFile::ExportTemplatedFile(	const std::string& _templateName,
											const std::string& _fileName,
											const std::string& _commonHeaderName,
											const std::string& _realString,
											const std::string& _intString,
											int _precision,
											const std::string& _commentString
						) : ExportFile( )
{
	setup( _templateName,_fileName,_commonHeaderName,_realString,_intString,_precision,_commentString );
}
                        
                        
returnValue ExportTemplatedFile::setup(	const std::string& _templateName,
										const std::string& _fileName,
										const std::string& _commonHeaderName,
										const std::string& _realString,
										const std::string& _intString,
										int _precision,
										const std::string& _commentString
                                        )
{
    ExportFile::setup( _fileName, _commonHeaderName, _realString, _intString, _precision, _commentString );
    
	folders = TEMPLATE_PATHS;
	templateName = _templateName;
    
    return SUCCESSFUL_RETURN;
}


returnValue ExportTemplatedFile::fillTemplate( )
{
	ifstream inputFile;
	size_t oldPos = 0;
	while( 1 )
	{
		size_t pos;
		string tmp;

		pos = folders.find(";", oldPos);
		tmp = folders.substr(oldPos, pos) + "/" + templateName;

		inputFile.open(tmp.c_str());

		if (inputFile.is_open() == true)
			break;

		if (pos == string::npos)
		{
			LOG( LVL_ERROR ) << "Unable to open the file: " << tmp << endl;
			return ACADOERROR( RET_DOES_DIRECTORY_EXISTS );
		}

		oldPos = pos + 1;
	}

	string str;
	while ( getline(inputFile, str) )
	{
		for (map<string, string>::const_iterator it = dictionary.begin(); it != dictionary.end(); ++it)
		{
			size_t pos = 0;
			while ((pos = str.find(it->first, pos)) != string::npos)
			{
				str.replace(pos, it->first.length(), it->second);
				pos += it->second.length();
			}
		}

		addStatement( str );
		addStatement( "\n" );
	}

	inputFile.close();
	
	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
