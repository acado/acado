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
 *    \file src/code_generation/integrators/discrete_export.cpp
 *    \author Rien Quirynen
 *    \date 2013
 */

#include <acado/code_generation/integrators/discrete_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DiscreteTimeExport::DiscreteTimeExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : IntegratorExport( _userInteraction,_commonHeaderName )
{
}


DiscreteTimeExport::DiscreteTimeExport(	const DiscreteTimeExport& arg
									) : IntegratorExport( arg )
{
	copy( arg );
}


DiscreteTimeExport::~DiscreteTimeExport( )
{
	clear( );
}


DiscreteTimeExport& DiscreteTimeExport::operator=( const DiscreteTimeExport& arg
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



// PROTECTED:


returnValue DiscreteTimeExport::copy(	const DiscreteTimeExport& arg
									)
{
	rhs = arg.rhs;
	diffs_rhs = arg.diffs_rhs;

	// ExportVariables
	rk_ttt = arg.rk_ttt;
	rk_xxx = arg.rk_xxx;
	
	// ExportFunctions
	integrate = arg.integrate;
	
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
