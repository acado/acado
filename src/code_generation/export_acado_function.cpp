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
 *    \file src/code_generation/export_ode_function.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010-2013
 */

#include <acado/code_generation/export_acado_function.hpp>
#include <acado/function/function_.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportAcadoFunction::ExportAcadoFunction( ) : ExportFunction( )
{
	numX = 0;
	numXA = 0;
	numU = 0;
	numDX = 0;
	numP = 0;
	f = std::tr1::shared_ptr< Function >(new Function());
}


ExportAcadoFunction::ExportAcadoFunction(	const Function& _f,
										const String& _name
										) : ExportFunction( _name )
{
	init(_f, _name);
}


ExportAcadoFunction::ExportAcadoFunction( const ExportAcadoFunction& arg ) : ExportFunction( arg )
{
	numX = arg.numX;
	numXA = arg.numXA;
	numU = arg.numU;
	numP = arg.numP;
	numDX = arg.numDX;
	globalVar = arg.globalVar;
	f = arg.f;
}


ExportAcadoFunction::~ExportAcadoFunction( )
{}


ExportAcadoFunction& ExportAcadoFunction::operator=( const ExportAcadoFunction& arg )
{
	if( this != &arg )
	{
		ExportFunction::operator=( arg );
		numX = arg.numX;
		numXA = arg.numXA;
		numU = arg.numU;
		globalVar = arg.globalVar;
		f = arg.f;
	}

	return *this;
}


ExportStatement* ExportAcadoFunction::clone( ) const
{
	return new ExportAcadoFunction(*this);
}


ExportFunction* ExportAcadoFunction::cloneFunction( ) const
{
	return new ExportAcadoFunction(*this);
}



returnValue ExportAcadoFunction::init(	const Function& _f,
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

	f = std::tr1::shared_ptr< Function >(new Function( _f ));

	globalVar.setup("acado_aux", f->getGlobalExportVariableSize(), 1, REAL, ACADO_WORKSPACE);
	f->setGlobalExportVariableName( globalVar.getFullName() );

	// Just add two dummy arguments in order to keep addFunctionCall function happy.
	return ExportFunction::init(_name, ExportArgument("input", 1, 1), ExportArgument("output", 1, 1));
}



returnValue ExportAcadoFunction::exportDataDeclaration(	FILE* file,
														const String& _realString,
														const String& _intString,
														int _precision
														) const
{
	return f->exportHeader(file, name.getName(), _realString.getName());
}


returnValue ExportAcadoFunction::exportForwardDeclaration(	FILE* file,
															const String& _realString,
															const String& _intString,
															int _precision
															) const
{
	return f->exportForwardDeclarations(file, name.getName(), _realString.getName());
}


returnValue ExportAcadoFunction::exportCode(	FILE* file,
											const String& _realString,
											const String& _intString,
											int _precision
											) const
{
	return f->exportCode(file, name.getName(), _realString.getName(),
			_precision, numX, numXA, numU, numP, numDX);
}


BooleanType ExportAcadoFunction::isDefined( ) const
{
	if (f->getDim() > 0)
		return BT_TRUE;

	return BT_FALSE;
}


unsigned ExportAcadoFunction::getFunctionDim( void )
{
	return f->getDim();
}

ExportVariable ExportAcadoFunction::getGlobalExportVariable( ) const
{
	return deepcopy( globalVar );
}

returnValue ExportAcadoFunction::setGlobalExportVariable(const ExportVariable& var)
{
	if (getFunctionDim() == 0)
		return SUCCESSFUL_RETURN;

	// TODO This is more hurting that helping. The dev has to take care of the size
	//      outside this function, the whole point of this function is to set names,
	//      not to check sizes.
//	ASSERT(var.getNumRows() >= f->getGlobalExportVariableSize() && var.getNumCols() == 1);

	globalVar = var;
	f->setGlobalExportVariableName( globalVar.getFullName() );

	return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
//



CLOSE_NAMESPACE_ACADO

// end of file.
