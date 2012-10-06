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
 *    \file src/code_generation/integrator_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Rien Quirynen
 *    \date 2010-2011
 */

#include <acado/code_generation/integrators/integrator_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IntegratorExport::IntegratorExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExportAlgorithm( _userInteraction,_commonHeaderName )
{
}


IntegratorExport::IntegratorExport(	const IntegratorExport& arg
									) : ExportAlgorithm( arg )
{
}


IntegratorExport::~IntegratorExport( )
{
	clear( );
}


IntegratorExport& IntegratorExport::operator=(	const IntegratorExport& arg
												)
{
	if( this != &arg && &arg != 0 )
	{
		clear( );
		ExportAlgorithm::operator=( arg );
	}
    return *this;
}


returnValue IntegratorExport::setGrid(	const Grid& _ocpGrid, const uint _numSteps
										)
{
	uint i;
	N = _ocpGrid.getNumIntervals();
	BooleanType equidistant = _ocpGrid.isEquidistant();
	double T = _ocpGrid.getLastTime() - _ocpGrid.getFirstTime();
	double h = T/((double)_numSteps);
	Vector stepsVector( N );

	for( i = 0; i < stepsVector.getDim(); i++ )
	{
		stepsVector(i) = (int) ceil((_ocpGrid.getTime(i+1)-_ocpGrid.getTime(i))/h - 10.0*EPS);
	}

	if( equidistant )
	{
		// Setup fixed integrator grid for equidistant control grid
		grid = Grid( 0.0, ((double) T)/((double) N), (int) ceil((double)_numSteps/((double) N) - 10.0*EPS) + 1 );
	}
	else
	{
		// Setup for non equidistant control grid
		// NOTE: This grid defines only one integration step because the control
		// grid is non equidistant.
		grid = Grid( 0.0, h, 2 );
		numSteps = stepsVector;
	}
	return SUCCESSFUL_RETURN;
}



// PROTECTED:


returnValue IntegratorExport::copy(	const IntegratorExport& arg
									)
{
	grid = arg.grid;
	numSteps = arg.numSteps;

	// ExportFunctions
	integrate = arg.integrate;
	
	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::clear( )
{
	return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getGrid( Grid& grid_ ) const{

    grid_ = grid;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getNumSteps( Vector& _numSteps ) const{

    _numSteps = numSteps;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getOutputExpressions( std::vector<Expression>& outputExpressions_ ) const{

    outputExpressions_ = outputExpressions;
    return SUCCESSFUL_RETURN;
}


returnValue IntegratorExport::getOutputGrids( std::vector<Grid>& outputGrids_ ) const{

    outputGrids_ = outputGrids;
    return SUCCESSFUL_RETURN;
}

BooleanType IntegratorExport::hasEquidistantGrid( ) const{
	
	return numSteps.isEmpty();
}


CLOSE_NAMESPACE_ACADO

// end of file.
