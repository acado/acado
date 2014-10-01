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
 *    \file src/dynamic_system/dynamic_system.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 */


#include <acado/dynamic_system/dynamic_system.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DynamicSystem::DynamicSystem( )
{
	nDiffEqn   = 0;
	nSwitchFcn = 0;

	diffEqn = 0;
	outputFcn = 0;

	switchFcn = 0;
	selectFcn = 0;
}


DynamicSystem::DynamicSystem(	const DifferentialEquation& _diffEqn
								)
{
    OutputFcn _outputFcn;

	nDiffEqn   = 1;
	nSwitchFcn = 0;

	diffEqn    = (DifferentialEquation**) calloc( 1,sizeof(DifferentialEquation*) );
	diffEqn[0] = _diffEqn.clone();

	outputFcn    = (OutputFcn**) calloc( 1,sizeof(OutputFcn*) );
	outputFcn[0] = new OutputFcn( _outputFcn );

	switchFcn = 0;
	selectFcn = 0;
}


DynamicSystem::DynamicSystem(	const DifferentialEquation& _diffEqn,
								const OutputFcn& _outputFcn
								)
{
	nDiffEqn   = 1;
	nSwitchFcn = 0;

	diffEqn    = (DifferentialEquation**) calloc( 1,sizeof(DifferentialEquation*) );
	diffEqn[0] = _diffEqn.clone();

	outputFcn    = (OutputFcn**) calloc( 1,sizeof(OutputFcn*) );
	outputFcn[0] = new OutputFcn( _outputFcn );

	switchFcn = 0;
	selectFcn = 0;
}


DynamicSystem::DynamicSystem( const DynamicSystem& rhs )
{
	nDiffEqn   = rhs.nDiffEqn;
	nSwitchFcn = rhs.nSwitchFcn;

	if ( ( rhs.diffEqn != 0 ) && ( rhs.outputFcn != 0 ) )
	{
		diffEqn = (DifferentialEquation**) calloc( nDiffEqn,sizeof(DifferentialEquation*) );
		for( uint i=0; i<nDiffEqn; ++i )
			diffEqn[i] = (rhs.diffEqn[i])->clone();

		outputFcn = (OutputFcn**) calloc( nDiffEqn,sizeof(OutputFcn*) );
		for( uint i=0; i<nDiffEqn; ++i )
			outputFcn[i] = new OutputFcn( *(rhs.outputFcn[i]) );
	}
	else
	{
		nDiffEqn = 0;

		diffEqn = 0;
		outputFcn = 0;
	}

	if ( rhs.switchFcn != 0 )
	{
		switchFcn = (Function**) calloc( nSwitchFcn,sizeof(Function*) );
		for( uint i=0; i<nSwitchFcn; ++i )
			switchFcn[i] = new Function( *(rhs.switchFcn[i]) );
	}
	else
	{
		switchFcn = 0;
		nSwitchFcn = 0;
	}

	if ( rhs.selectFcn != 0 )
		selectFcn = new Function( *(rhs.selectFcn) );
	else
		selectFcn = 0;
}


DynamicSystem::~DynamicSystem( )
{
	if ( diffEqn != 0 )
	{
		for( uint i=0; i<nDiffEqn; ++i )
			delete diffEqn[i];
		free( diffEqn );
	}

	if ( outputFcn != 0 )
	{
		for( uint i=0; i<nDiffEqn; ++i )
			delete outputFcn[i];
		free( outputFcn );
	}

	if ( switchFcn != 0 )
	{
		for( uint i=0; i<nSwitchFcn; ++i )
			delete switchFcn[i];
		free( switchFcn );
	}

	if ( selectFcn != 0 )
		delete selectFcn;
}


DynamicSystem& DynamicSystem::operator=( const DynamicSystem& rhs )
{
	if ( this != &rhs )
	{
		if ( diffEqn != 0 )
		{
			for( uint i=0; i<nDiffEqn; ++i )
				delete diffEqn[i];
			free( diffEqn );
		}
	
		if ( outputFcn != 0 )
		{
			for( uint i=0; i<nDiffEqn; ++i )
				delete outputFcn[i];
			free( outputFcn );
		}
	
		if ( switchFcn != 0 )
		{
			for( uint i=0; i<nSwitchFcn; ++i )
				delete switchFcn[i];
			free( switchFcn );
		}
	
		if ( selectFcn != 0 )
			delete selectFcn;


		nDiffEqn   = rhs.nDiffEqn;
		nSwitchFcn = rhs.nSwitchFcn;
	
		if ( ( rhs.diffEqn != 0 ) && ( rhs.outputFcn != 0 ) )
		{
			diffEqn = (DifferentialEquation**) calloc( nDiffEqn,sizeof(DifferentialEquation*) );
			for( uint i=0; i<nDiffEqn; ++i )
				diffEqn[i] = (rhs.diffEqn[i])->clone();
	
			outputFcn = (OutputFcn**) calloc( nDiffEqn,sizeof(OutputFcn*) );
			for( uint i=0; i<nDiffEqn; ++i )
				outputFcn[i] = new OutputFcn( *(rhs.outputFcn[i]) );
		}
		else
		{
			nDiffEqn = 0;
	
			diffEqn = 0;
			outputFcn = 0;
		}
	
		if ( rhs.switchFcn != 0 )
		{
			switchFcn = (Function**) calloc( nSwitchFcn,sizeof(Function*) );
			for( uint i=0; i<nSwitchFcn; ++i )
				switchFcn[i] = new Function( *(rhs.switchFcn[i]) );
		}
		else
		{
			switchFcn = 0;
			nSwitchFcn = 0;
		}
	
		if ( rhs.selectFcn != 0 )
			selectFcn = new Function( *(rhs.selectFcn) );
		else
			selectFcn = 0;
	}

	return *this;
}



returnValue DynamicSystem::addSubsystem( const DifferentialEquation& _diffEqn )
{
	OutputFcn emptyOutputFcn;
	return addSubsystem( _diffEqn,emptyOutputFcn );
}


returnValue DynamicSystem::addSubsystem(	const DifferentialEquation& _diffEqn,
											const OutputFcn& _outputFcn
											)
{
	if ( isConsistentDiffEqn( _diffEqn ) == BT_FALSE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( isConsistentOutputFcn( _outputFcn ) == BT_FALSE )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	++nDiffEqn;

	diffEqn = (DifferentialEquation**) realloc( diffEqn,nDiffEqn*sizeof(DifferentialEquation*) );
	diffEqn[ nDiffEqn-1 ] = _diffEqn.clone();

	outputFcn = (OutputFcn**) realloc( outputFcn,nDiffEqn*sizeof(OutputFcn*) );
	outputFcn[ nDiffEqn-1 ] = new OutputFcn( _outputFcn );

	//return SUCCESSFUL_RETURN;
	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}



returnValue DynamicSystem::addSwitchFunction(	const Function& _switchFcn
												)
{
	++nSwitchFcn;

	switchFcn = (Function**) realloc( switchFcn,nSwitchFcn*sizeof(Function*) );
	switchFcn[ nSwitchFcn-1 ] = new Function( _switchFcn );

	//return SUCCESSFUL_RETURN;
	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}



returnValue DynamicSystem::setSelectFunction(	const Function& _selectFcn
												)
{
	if ( selectFcn == 0 )
		selectFcn = new Function( _selectFcn );
	else
		*selectFcn = _selectFcn;

	//return SUCCESSFUL_RETURN;
	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}



//
// PROTECTED MEMBER FUNCTIONS:
//

BooleanType DynamicSystem::isConsistentDiffEqn(	const DifferentialEquation& _diffEqn
												) const
{
	if ( nDiffEqn == 0 )
		return BT_TRUE;

	if ( diffEqn[0]->getNumDynamicEquations( ) != _diffEqn.getNumDynamicEquations( ) )
		return BT_FALSE;

	if ( diffEqn[0]->getNumAlgebraicEquations( ) != _diffEqn.getNumAlgebraicEquations( ) )
		return BT_FALSE;

	if ( diffEqn[0]->isDiscretized( ) != _diffEqn.isDiscretized( ) )
		return BT_FALSE;

	return BT_TRUE;
}


BooleanType DynamicSystem::isConsistentOutputFcn(	const OutputFcn& _outputFcn
													) const
{
	if ( nDiffEqn == 0 )
		return BT_TRUE;

	if ( outputFcn[0]->getDim( ) != _outputFcn.getDim( ) )
		return BT_FALSE;

	if ( ( diffEqn[0]->isODE( ) == BT_TRUE ) && ( _outputFcn.getNXA( ) > 0 ) )
		return BT_FALSE;

	return BT_TRUE;
}



CLOSE_NAMESPACE_ACADO

// end of file.
