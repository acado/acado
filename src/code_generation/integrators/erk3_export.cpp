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
 *    \file src/code_generation/erk3_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/erk3_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ExplicitRungeKutta3Export::ExplicitRungeKutta3Export(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	numStages = 3;
}


ExplicitRungeKutta3Export::ExplicitRungeKutta3Export(	const ExplicitRungeKutta3Export& arg
									) : ExplicitRungeKuttaExport( arg )
{
	numStages = 3;
	copy( arg );
}


ExplicitRungeKutta3Export::~ExplicitRungeKutta3Export( )
{
	clear( );
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createExplicitRungeKutta3Export(	UserInteraction* _userInteraction,
													const String &_commonHeaderName)
{
	Matrix AA(3,3);
	Vector bb(3);
	Vector cc(3);

	AA(0,0) = 0.0;		AA(0,1) = 0.0;		AA(0,2) = 0.0;
	AA(1,0) = 1.0/3.0;	AA(1,1) = 0.0;		AA(1,2) = 0.0;
	AA(2,0) = 0.0;		AA(2,1) = 2.0/3.0;	AA(2,2) = 0.0;

	bb(0) = 1.0/4.0;
	bb(1) = 0.0;
	bb(2) = 3.0/4.0;

	cc(0) = 0.0;
	cc(1) = 1.0/3.0;
	cc(2) = 2.0/3.0;

	ExplicitRungeKuttaExport* integrator = createExplicitRungeKuttaExport(_userInteraction, _commonHeaderName);
	integrator->initializeButcherTableau(AA, bb, cc);

	return integrator;
}

RegisterExplicitRungeKutta3Export::RegisterExplicitRungeKutta3Export()
{
	IntegratorExportFactory::instance().registerAlgorithm(INT_RK3, createExplicitRungeKutta3Export);
}


CLOSE_NAMESPACE_ACADO

// end of file.
