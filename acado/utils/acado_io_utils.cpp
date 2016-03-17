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
 *    \file src/utils/acado_io_utils.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 29.12.2008
 */

#include <acado/utils/acado_types.hpp>
#include <acado/utils/acado_constants.hpp>
#include <acado/utils/acado_io_utils.hpp>
#include <acado/code_generation/templates/templates.hpp>

#if defined( __WIN32__ ) || defined( WIN32 )

#include <windows.h>
#include <direct.h>

#elif defined( LINUX )

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#else

#warning "File I/O is not supported on this platform"

#endif /* defined(__WIN32__) || defined(WIN32) */

using namespace std;

BEGIN_NAMESPACE_ACADO

#define AUTOGEN_NOTICE_LENGTH 15
static const char* autogenerationNotice[ AUTOGEN_NOTICE_LENGTH ] =
{
		"This file was auto-generated using the ACADO Toolkit.\n",
		"\n",
		"While ACADO Toolkit is free software released under the terms of\n",
		"the GNU Lesser General Public License (LGPL), the generated code\n",
		"as such remains the property of the user who used ACADO Toolkit\n",
		"to generate this code. In particular, user dependent data of the code\n",
		"do not inherit the GNU LGPL license. On the other hand, parts of the\n",
		"generated code that are a direct copy of source code from the\n",
		"ACADO Toolkit or the software tools it is based on, remain, as derived\n",
		"work, automatically covered by the LGPL license.\n",
		"\n",
		"ACADO Toolkit is distributed in the hope that it will be useful,\n",
		"but WITHOUT ANY WARRANTY; without even the implied warranty of\n",
		"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n",
		"\n"
};

static uint getStringLength(const char* string)
{
	if (string != 0)
		return strlen(string);

	return 0;
}

returnValue getGlobalStringDefinitions(	PrintScheme _printScheme,
										char** _startString,
										char** _endString,
										uint& _width,
										uint& _precision,
										char** _colSeparator,
										char** _rowSeparator
										)
{
	switch (_printScheme) {
	case PS_DEFAULT:

		*_startString = new char[getStringLength(DEFAULT_START_STRING) + 1];
		strcpy(*_startString, DEFAULT_START_STRING);

		*_endString = new char[getStringLength(DEFAULT_END_STRING) + 1];
		strcpy(*_endString, DEFAULT_END_STRING);

		_width = DEFAULT_WIDTH;
		_precision = DEFAULT_PRECISION;

		*_colSeparator = new char[getStringLength(DEFAULT_COL_SEPARATOR) + 1];
		strcpy(*_colSeparator, DEFAULT_COL_SEPARATOR);

		*_rowSeparator = new char[getStringLength(DEFAULT_ROW_SEPARATOR) + 1];
		strcpy(*_rowSeparator, DEFAULT_ROW_SEPARATOR);

		break;

	case PS_PLAIN:

		*_startString = new char[1];
		(*_startString)[0] = '\0';

		*_endString = new char[2];
		(*_endString)[0] = '\n';
		(*_endString)[1] = '\0';

		_width = DEFAULT_WIDTH;
		_precision = DEFAULT_PRECISION;

		*_colSeparator = new char[2];
		(*_colSeparator)[0] = ' ';
		(*_colSeparator)[1] = '\0';

		*_rowSeparator = new char[2];
		(*_rowSeparator)[0] = '\n';
		(*_rowSeparator)[1] = '\0';

		break;

	case PS_MATLAB:
	case PS_MATLAB_BINARY:

		*_startString = new char[3];
		(*_startString)[0] = '[';
		(*_startString)[1] = ' ';
		(*_startString)[2] = '\0';

		*_endString = new char[5];
		(*_endString)[0] = ' ';
		(*_endString)[1] = ']';
		(*_endString)[2] = ';';
		(*_endString)[3] = '\n';
		(*_endString)[4] = '\0';

		_width = DEFAULT_WIDTH;
		_precision = DEFAULT_PRECISION;

		*_colSeparator = new char[3];
		(*_colSeparator)[0] = ',';
		(*_colSeparator)[1] = ' ';
		(*_colSeparator)[2] = '\0';

		*_rowSeparator = new char[3];
		(*_rowSeparator)[0] = ';';
		(*_rowSeparator)[1] = '\n';
		(*_rowSeparator)[2] = '\0';

		break;

	default:
		return ACADOERROR( RET_UNKNOWN_BUG );
	}

	return SUCCESSFUL_RETURN;
}

returnValue acadoCopyFile(	const std::string& source,
							const std::string& destination,
							const std::string& commentString,
							bool printCodegenNotice
							)
{
	std::ifstream  src( source.c_str() );
	if (src.is_open() == false)
	{
		LOG( LVL_ERROR ) << "Could not open the source file: " << source << std::endl;
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	}

	std::ofstream  dst( destination.c_str() );
	if (dst.is_open() == false)
	{
		LOG( LVL_ERROR ) << "Could not open the destination file: " << destination << std::endl;
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	}

	if (printCodegenNotice == BT_TRUE)
	{
		if (commentString.empty())
		{
			dst << "/*" << endl;
			for (unsigned i = 0; i < AUTOGEN_NOTICE_LENGTH; ++i)
				dst << " *    " << autogenerationNotice[ i ];
			dst << " */" << endl << endl;
		}
		else
		{
			dst << commentString << endl;
			for (unsigned i = 0; i < AUTOGEN_NOTICE_LENGTH; ++i)
				dst << commentString << "    " << autogenerationNotice[ i ];
			dst << endl;
		}
	}

	dst << src.rdbuf();

	src.close();
	dst.close();

	return SUCCESSFUL_RETURN;
}

returnValue acadoCopyTemplateFile(	const std::string& source,
									const std::string& destination,
									const std::string& commentString,
									bool printCodegenNotice
									)
{
	const string folders( TEMPLATE_PATHS );
	ifstream inputFile;
	size_t oldPos = 0;

	while( 1 )
	{
		size_t pos;
		string tmp;

		pos = folders.find(";", oldPos);
		tmp = folders.substr(oldPos, pos) + "/" + source;

		inputFile.open(tmp.c_str());

		if (inputFile.is_open() == true)
			return acadoCopyFile(tmp, destination, commentString, printCodegenNotice);

		if (pos == string::npos)
			break;

		oldPos = pos + 1;
	}

	LOG( LVL_ERROR ) << "Could not open the template file: " << source << std::endl;
	return ACADOERROR( RET_INVALID_ARGUMENTS );
}

returnValue acadoCreateFolder(	const std::string& name
								)
{
#if defined( __WIN32__ ) || defined( WIN32 )

	int status = _mkdir( name.c_str() );
	errno_t err;
	_get_errno( &err );
	if (status && err != EEXIST)
	{
		LOG( LVL_ERROR ) << "Problem creating directory " << name << endl;
		return ACADOERROR( RET_INVALID_ARGUMENTS );
	}

#elif defined( LINUX )

	struct stat st = {0};

	if (stat(name.c_str(), &st) == -1)
	{
	    if (mkdir(name.c_str(), 0700) == -1)
	    {
	    	// TODO give here an error code
	    	LOG( LVL_ERROR ) << "Problem creating directory " << name << endl;
	    	return ACADOERROR( RET_INVALID_ARGUMENTS );
	    }
	}

#else

	return ACADOERRORTEXT(RET_INVALID_OPTION, "Unsupported platform.");

#endif /* defined(__WIN32__) || defined(WIN32) */

	return SUCCESSFUL_RETURN;
}

/*
 *	p r i n t C o p y r i g h t N o t i c e
 */
returnValue acadoPrintCopyrightNotice(	const std::string& subpackage
										)
{
	if (subpackage.empty())
		cout << "\nACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.\n" \
				"Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,\n" \
				"Milan Vukov, Rien Quirynen, KU Leuven.\n" \
				"Developed within the Optimization in Engineering Center (OPTEC)\n" \
				"under supervision of Moritz Diehl. All rights reserved.\n\n" \
				"ACADO Toolkit is distributed under the terms of the GNU Lesser\n" \
				"General Public License 3 in the hope that it will be useful,\n" \
				"but WITHOUT ANY WARRANTY; without even the implied warranty of\n" \
				"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\n" \
				"GNU Lesser General Public License for more details.\n\n";
	else
		cout << "\nACADO Toolkit::" << subpackage << endl
			 << "Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,\n" \
				"Milan Vukov, Rien Quirynen, KU Leuven.\n" \
				"Developed within the Optimization in Engineering Center (OPTEC)\n" \
				"under supervision of Moritz Diehl. All rights reserved.\n\n" \
				"ACADO Toolkit is distributed under the terms of the GNU Lesser\n" \
				"General Public License 3 in the hope that it will be useful,\n" \
				"but WITHOUT ANY WARRANTY; without even the implied warranty of\n" \
				"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\n" \
				"GNU Lesser General Public License for more details.\n\n";

	return SUCCESSFUL_RETURN;
}


returnValue acadoPrintAutoGenerationNotice(	std::ofstream& stream,
											const std::string& commentString
											)
{
	if (stream.is_open() == false)
		return RET_INVALID_ARGUMENTS;

	if (commentString.empty())
	{
		stream << "/*\n";
		for (unsigned i = 0; i < AUTOGEN_NOTICE_LENGTH; ++i)
			stream << " *    " << autogenerationNotice[ i ];
		stream << " */\n";
	}
	else
	{
		stream << commentString << endl;
		for (unsigned i = 0; i < AUTOGEN_NOTICE_LENGTH; ++i)
			stream << commentString << "    " << autogenerationNotice[ i ];
	}

	stream << endl << endl;

    return SUCCESSFUL_RETURN;
}



/*
 *	g e t C P U t i m e
 */
double acadoGetTime( )
{
	double current_time = 0.0;

	#if defined(__WIN32__) || defined(WIN32)
	LARGE_INTEGER counter, frequency;
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&counter);
	current_time = ((double) counter.QuadPart) / ((double) frequency.QuadPart);
	#elif defined(LINUX)
	struct timeval theclock;
	gettimeofday( &theclock,0 );
	current_time = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;
	#endif

	return current_time;
}

CLOSE_NAMESPACE_ACADO

/*
 *    end of file
 */
