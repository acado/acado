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
 *    \file src/code_generation/rk_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/rk_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

RungeKuttaExport::RungeKuttaExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : IntegratorExport( _userInteraction,_commonHeaderName )
{
}


RungeKuttaExport::RungeKuttaExport(	const RungeKuttaExport& arg
									) : IntegratorExport( arg )
{
	copy( arg );
}


RungeKuttaExport::~RungeKuttaExport( )
{
	clear( );
}


RungeKuttaExport& RungeKuttaExport::operator=( const RungeKuttaExport& arg
												)
{
	if( this != &arg )
	{
		clear( );
		IntegratorExport::operator=( arg );
		copy( arg );
	}
    return *this;
}


returnValue RungeKuttaExport::initializeButcherTableau( const Matrix& _AA, const Vector& _bb, const Vector& _cc ) {

	if( _cc.isEmpty() || !_AA.isSquare() || _AA.getNumRows() != _bb.getDim() || _bb.getDim() != _cc.getDim() ) return RET_INVALID_OPTION;

	numStages = _cc.getDim();
	AA = _AA;
	bb = _bb;
	cc = _cc;

	return SUCCESSFUL_RETURN;
}


uint RungeKuttaExport::getNumStages() {
	
	return numStages;
}


returnValue RungeKuttaExport::setNARXmodel( const uint delay, const Matrix& parms ) {

	return RET_INVALID_OPTION;
}



// PROTECTED:


returnValue RungeKuttaExport::copy(	const RungeKuttaExport& arg
									)
{
	numStages = arg.numStages;
	AA = arg.AA;
	bb = arg.bb;
	cc = arg.cc;

	rhs = arg.rhs;
	diffs_rhs = arg.diffs_rhs;
	name_rhs = arg.name_rhs;
	name_diffs_rhs = arg.name_diffs_rhs;
	grid = arg.grid;

	// ExportVariables
	rk_ttt = arg.rk_ttt;
	rk_xxx = arg.rk_xxx;
	rk_kkk = arg.rk_kkk;
	
	// ExportFunctions
	integrate = arg.integrate;
	
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
