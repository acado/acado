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
 *    \file src/code_generation/export_ode_function.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov
 *    \date 2010-2013
 */

#include <acado/code_generation/export_acado_function.hpp>
#include <acado/function/function_.hpp>

using namespace std;
BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExportAcadoFunction::ExportAcadoFunction( ) : ExportFunction( )
{
	numX = 0;
	numXA = 0;
	numU = 0;
	numP = 0;
	numDX = 0;
	numOD = 0;

	f = std::shared_ptr< Function >(new Function( ));

	external = false;
}


ExportAcadoFunction::ExportAcadoFunction(	const Function& _f,
											const std::string& _name
											) : ExportFunction( _name )
{
	init(_f, _name);
}

ExportAcadoFunction::ExportAcadoFunction(	const std::string& _name
											) : ExportFunction( _name )
{
	init(Function(), _name);
	external = true;
}

ExportAcadoFunction::~ExportAcadoFunction( )
{}


ExportStatement* ExportAcadoFunction::clone( ) const
{
	return new ExportAcadoFunction(*this);
}

returnValue ExportAcadoFunction::init(	const Function& _f,
										const std::string& _name,
										const uint _numX,
										const uint _numXA,
										const uint _numU,
										const uint _numP,
										const uint _numDX,
										const uint _numOD
										)
{
	numX = _numX;
	numXA = _numXA;
	numU = _numU;
	numP = _numP;
	numDX = _numDX;
	numOD = _numOD;

	f = std::shared_ptr< Function >(new Function( _f ));

	globalVar.setup( "rhs_aux", f->getGlobalExportVariableSize(), 1, REAL, ACADO_WORKSPACE);
	f->setGlobalExportVariableName( globalVar.getFullName() );

	external = false;

	// Just add two dummy arguments in order to keep addFunctionCall function happy.
	return ExportFunction::init(_name, ExportArgument("input", 1, 1), ExportArgument("output", 1, 1));
}

returnValue ExportAcadoFunction::exportDataDeclaration(	std::ostream& stream,
														const std::string& _realString,
														const std::string& _intString,
														int _precision
														) const
{
	ASSERT( external == false );

	stream	<< _realString << " " << f->getGlobalExportVariableName()
			<< "[ " << f->getGlobalExportVariableSize( ) << " ];" << std::endl;

	return SUCCESSFUL_RETURN;
}


returnValue ExportAcadoFunction::exportForwardDeclaration(	std::ostream& stream,
															const std::string& _realString,
															const std::string& _intString,
															int _precision
															) const
{
	if (flagPrivate == true)
		return SUCCESSFUL_RETURN;

	if (external == true)
	{
		stream << endl;
		stream << "/** An external function for evaluation of symbolic expressions. */" << endl;
		stream << "void " << name << "(const " << _realString << "* in, " << _realString << "* out);" << endl;

		return SUCCESSFUL_RETURN;
	}

	return f->exportForwardDeclarations(stream, name.c_str(), _realString.c_str());
}


returnValue ExportAcadoFunction::exportCode(	std::ostream& stream,
												const std::string& _realString,
												const std::string& _intString,
												int _precision
												) const
{
	if (external == true)
		return SUCCESSFUL_RETURN;

	return f->exportCode(
			stream, name.c_str(), _realString.c_str(), numX, numXA, numU, numP, numDX, numOD,
			// TODO: Here we allocate local memory for the function, this should be extended.
			false, false);
}


bool ExportAcadoFunction::isDefined( ) const
{
	if (f->getDim() > 0 || external == true)
		return true;

	return false;
}


unsigned ExportAcadoFunction::getFunctionDim( void )
{
	ASSERT( external == false );

	return f->getDim();
}

ExportVariable ExportAcadoFunction::getGlobalExportVariable( ) const
{
	return deepcopy( globalVar );
}

returnValue ExportAcadoFunction::setGlobalExportVariable(const ExportVariable& var)
{
	ASSERT( external == false );

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

bool ExportAcadoFunction::isExternal() const
{
	return external;
}

CLOSE_NAMESPACE_ACADO
