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
 *    \file src/code_generation/radau_IIA3_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/radau_IIA3_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

RadauIIA3Export::RadauIIA3Export(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ImplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	numStages = 2;
}


RadauIIA3Export::RadauIIA3Export(	const RadauIIA3Export& arg
									) : ImplicitRungeKuttaExport( arg )
{
	numStages = 2;
	copy( arg );
}


RadauIIA3Export::~RadauIIA3Export( )
{
	clear( );
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createRadauIIA3Export(	UserInteraction* _userInteraction,
											const std::string &_commonHeaderName)
{
	DMatrix AA(2,2);
	DVector bb(2);
	DVector cc(2);

	AA(0,0) = 5/(double)12;
	AA(0,1) = -1/(double)12;
	AA(1,0) = 3/(double)4;
	AA(1,1) = 1/(double)4;

	bb(0) = 3/(double)4;
	bb(1) = 1/(double)4;

	cc(0) = 1/(double)3;
	cc(1) = 1;

	ImplicitRungeKuttaExport* integrator = createImplicitRungeKuttaExport(_userInteraction, _commonHeaderName);
	integrator->initializeButcherTableau(AA, bb, cc);

	return integrator;
}

CLOSE_NAMESPACE_ACADO

// end of file.
