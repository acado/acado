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
 *    \file src/code_generation/export_ode_function.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/export_ode_function.hpp>
#include <acado/function/function_.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportODEfunction::ExportODEfunction( ) : ExportFunction( ), globalVar()
{
	numX = 0;
	numXA = 0;
	numU = 0;
	numDX = 0;
	numP = 0;

	f = new Function();

	globalVar.setup("acado_aux", f->getGlobalExportVariableSize(), 1, REAL, ACADO_WORKSPACE);
	f->setGlobalExportVariableName( globalVar.getFullName() );
}


ExportODEfunction::ExportODEfunction(	const Function& _f,
										const String& _name
										) : ExportFunction( _name )
{
	numX = 0;
	numXA = 0;
	numU = 0;
	numP = 0;
	numDX = 0;
	f = new Function( _f );

	globalVar.setup("acado_aux", f->getGlobalExportVariableSize(), 1, REAL, ACADO_WORKSPACE);
	f->setGlobalExportVariableName( globalVar.getFullName() );
}


ExportODEfunction::ExportODEfunction( const ExportODEfunction& arg ) : ExportFunction( arg )
{
	numX = arg.numX;
	numXA = arg.numXA;
	numU = arg.numU;
	numP = arg.numP;
	numDX = arg.numDX;
	globalVar = arg.globalVar;

	if ( arg.f != 0 )
	{
		f = new Function( *(arg.f) );
	}
}


ExportODEfunction::~ExportODEfunction( )
{
	if ( f )
		delete f;
	f = 0;
}


ExportODEfunction& ExportODEfunction::operator=( const ExportODEfunction& arg )
{
	if( this != &arg )
	{
		ExportFunction::operator=( arg );
		numX = arg.numX;
		numXA = arg.numXA;
		numU = arg.numU;
		globalVar = arg.globalVar;

		if ( f )
		{
			delete f;

			f = 0;
		}

		if ( arg.f != 0 )
		{
			f = new Function( *(arg.f) );
		}
	}

	return *this;
}


ExportStatement* ExportODEfunction::clone( ) const
{
	return new ExportODEfunction(*this);
}


ExportFunction* ExportODEfunction::cloneFunction( ) const
{
	return new ExportODEfunction(*this);
}



returnValue ExportODEfunction::init(	const Function& _f,
										const String& _name,
										const uint _numX,
										const uint _numXA,
										const uint _numU,
										const uint _numP,
										const uint _numDX
										)
{
	numX = _numX;
	numXA = _numXA;
	numU = _numU;
	numP = _numP;
	numDX = _numDX;

	if ( f )
		delete f;
	f = 0;

	f = new Function( _f );
	globalVar.setup("acado_aux", f->getGlobalExportVariableSize(), 1, REAL, ACADO_WORKSPACE);
	f->setGlobalExportVariableName( globalVar.getFullName() );

	return ExportFunction::init( _name );
}



returnValue ExportODEfunction::exportDataDeclaration(	FILE* file,
														const String& _realString,
														const String& _intString,
														int _precision
														) const
{
	if (f->getDim() > 0)
		return f->exportHeader(file, name.getName(), _realString.getName());

	return SUCCESSFUL_RETURN;
}


returnValue ExportODEfunction::exportForwardDeclaration(	FILE* file,
															const String& _realString,
															const String& _intString,
															int _precision
															) const
{
	if (f->getDim() > 0)
		return f->exportForwardDeclarations(file, name.getName(), _realString.getName());

	return SUCCESSFUL_RETURN;
}


returnValue ExportODEfunction::exportCode(	FILE* file,
											const String& _realString,
											const String& _intString,
											int _precision
											) const
{
	if (f->getDim() > 0)
		return f->exportCode(file, name.getName(), _realString.getName(), _precision, numX, numXA, numU, numP, numDX);

	return SUCCESSFUL_RETURN;
}


BooleanType ExportODEfunction::isDefined( ) const
{
	if (f->getDim() > 0)
		return BT_TRUE;

	return BT_FALSE;
}


unsigned ExportODEfunction::getFunctionDim( void )
{
	return f->getDim();
}

ExportVariable ExportODEfunction::getGlobalExportVariable( ) const
{
	return deepcopy( globalVar );
}

returnValue ExportODEfunction::setGlobalExportVariable(const ExportVariable& var)
{
	ASSERT(var.getNumRows() < f->getGlobalExportVariableSize() || var.getNumCols() != 1)

	globalVar = var;
	f->setGlobalExportVariableName( globalVar.getFullName() );

	return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
