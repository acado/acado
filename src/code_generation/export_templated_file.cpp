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
 *    \file src/code_generation/export_templated_file.cpp
 *    \author Milan Vukov
 *    \date 2012 - 2013
 */

#include <acado/code_generation/export_templated_file.hpp>
#include <acado/code_generation/templates/templates.hpp>

#include <iostream>
#include <fstream>

BEGIN_NAMESPACE_ACADO

using namespace std;

ExportTemplatedFile::ExportTemplatedFile(	const String& _templateName,
											const String& _fileName,
											const String& _commonHeaderName,
											const String& _realString,
											const String& _intString,
											int _precision,
											const String& _commentString
						) : ExportFile(_fileName, _commonHeaderName, _realString, _intString, _precision, _commentString)
{
	folders = TEMPLATE_PATHS;
	templateName = _templateName;
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
		tmp = folders.substr(oldPos, pos) + "/" + templateName.getName();

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

		// This is some extremely stupid hack we had to do. Namely, String() class
		// cannot handle long strings, so we have to cut them in smaller pieces.
		if ( str.size() )
		{
			size_t pos = 0;
			while ( 1 )
			{
				if ((str.size() - pos) < 256)
				{
					string tmp = str.substr( pos );

					if ( tmp.size() )
						addStatement( static_cast< String >( tmp.c_str() ) );

					break;
				}
				else
				{
					addStatement( static_cast< String >( str.substr(pos, 256).c_str() ) );

					pos += 256;
				}
			}
		}
		addStatement( (String)"\n" );
	}

	inputFile.close();
	
	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO
