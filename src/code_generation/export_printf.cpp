/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 * 	  under supervision of Moritz Diehl. All rights reserved.
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
 *    \file src/code_generation/export_printf.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_printf.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportPrintf::ExportPrintf(	const ExportVariable& _data
							) : ExportStatement( )
{
// 	ASSERT( data.isGiven() == BT_TRUE );

	data = _data;
}


ExportPrintf::ExportPrintf( const ExportPrintf& arg ) : ExportStatement( arg )
{
	data = arg.data;
}


ExportPrintf::~ExportPrintf( )
{
}


ExportPrintf& ExportPrintf::operator=( const ExportPrintf& arg )
{
	if( this != &arg )
	{
		ExportStatement::operator=( arg );
		data = arg.data;
	}

	return *this;
}


ExportStatement* ExportPrintf::clone( ) const
{
	return new ExportPrintf(*this);
}




returnValue ExportPrintf::exportDataDeclaration(	FILE *file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
// 	if ( data.isVector() == BT_TRUE )
// 		acadoFPrintf( file,"%s %s;\n", _intString,"run1" );
// 	else
//		acadoFPrintf( file,"%s %s,%s;\n", _intString.getName(),"run01","run02" );

	return SUCCESSFUL_RETURN;
}


returnValue ExportPrintf::exportCode(	FILE *file,
										const String& _realString,
										const String& _intString,
										int _precision
										) const
{
	acadoFPrintf( file,"printf( \"%s = \\n\" );\n", data.getFullName().getName() );
	
// 	ExportIndex outerLoopVariable( "run01" );
// 	ExportIndex innerLoopVariable( "run02" );
// 	
// 	ExportForLoop outerLoop( outerLoopVariable,0,data.getNumRows() );
// 	ExportForLoop innerLoop( innerLoopVariable,0,data.getNumCols() );
// 	
// 	innerLoop.addStatement( )
// 	outerLoop.addStatement( innerLoop );
	
	acadoFPrintf( file,"%s %s,%s;\n", _intString.getName(),"run01","run02" );

	acadoFPrintf( file,"for( run01=0; run01<%d; ++run01 ){\n",data.getNumRows() );
	acadoFPrintf( file,"  for( run02=0; run02<%d; ++run02 )\n",data.getNumCols() );

//	if ( data.isAccessedTransposed() == BT_FALSE )
		acadoFPrintf( file,"    printf( \"%%e \\t\", %s[run01*%d+run02] );\n", data.getFullName().getName(),data.getNumCols() );
//	else
//		acadoFPrintf( file,"    printf( \"%%e \\t\", %s[run02*%d+run01] );\n", data.getFullName().getName(),data.getNumRows() );

	acadoFPrintf( file,"  printf( \"\\n\" );\n" );
	acadoFPrintf( file,"}\n" );

	return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
