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
*    \file src/simulation_environment/simulation_block.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*/


#include <acado/simulation_environment/simulation_block.hpp>



BEGIN_NAMESPACE_ACADO



SimulationBlock::SimulationBlock( ) : UserInteraction( )
{
	setName( BN_DEFAULT );
	setSamplingTime( DEFAULT_SAMPLING_TIME );
}


SimulationBlock::SimulationBlock(	BlockName _name,
									double _samplingTime
									) : UserInteraction( )
{
	setName( _name );
	setSamplingTime( _samplingTime );
}


SimulationBlock::SimulationBlock(	const SimulationBlock &rhs
									) : UserInteraction( rhs )
{
	setName( rhs.name );
	setSamplingTime( rhs.samplingTime );
}


SimulationBlock::~SimulationBlock()
{
}


SimulationBlock& SimulationBlock::operator=(	const SimulationBlock &rhs
												)
{
    if( this != &rhs )
	{
		UserInteraction::operator=( rhs );

		setName( rhs.name );
		setSamplingTime( rhs.samplingTime );
	}

    return *this;
}


CLOSE_NAMESPACE_ACADO


/*
 *	end of file
 */
