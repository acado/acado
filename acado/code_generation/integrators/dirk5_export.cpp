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
 *    \file src/code_generation/integrators/dirk5_export.cpp
 *    \author Rien Quirynen
 *    \date 2013
 */

#include <acado/code_generation/integrators/dirk5_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

DiagonallyIRK5Export::DiagonallyIRK5Export(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : DiagonallyImplicitRKExport( _userInteraction,_commonHeaderName )
{
	numStages = 5;
}


DiagonallyIRK5Export::DiagonallyIRK5Export(	const DiagonallyIRK5Export& arg
									) : DiagonallyImplicitRKExport( arg )
{
	numStages = 5;
	copy( arg );
}


DiagonallyIRK5Export::~DiagonallyIRK5Export( )
{
	clear( );
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createDiagonallyIRK5Export(	UserInteraction* _userInteraction,
												const std::string &_commonHeaderName)
{
	DMatrix AA(5,5);
	DVector bb(5);
	DVector cc(5);

	AA(0,0) = (6.0-sqrt(6.0))/10.0;					AA(0,1) = 0.0;									AA(0,2) = 0.0;								AA(0,3) = 0.0;								AA(0,4) = 0.0;
	AA(1,0) = (5.0*sqrt(6.0)-6.0)/14.0;				AA(1,1) = (6.0-sqrt(6.0))/10.0;					AA(1,2) = 0.0;								AA(1,3) = 0.0;								AA(1,4) = 0.0;
	AA(2,0) = (607.0*sqrt(6.0)+888.0)/2850.0;		AA(2,1) = (126.0-161.0*sqrt(6.0))/1425.0;		AA(2,2) = (6.0-sqrt(6.0))/10.0;				AA(2,3) = 0.0;								AA(2,4) = 0.0;
	AA(3,0) = (3153.0-3082.0*sqrt(6.0))/14250.0;	AA(3,1) = (3213.0+1148.0*sqrt(6.0))/28500.0;	AA(3,2) = (88.0*sqrt(6.0)-267.0)/500.0;		AA(3,3) = (6.0-sqrt(6.0))/10.0;				AA(3,4) = 0.0;
	AA(4,0) = (14638.0*sqrt(6.0)-32583.0)/71250.0;	AA(4,1) = (364.0*sqrt(6.0)-17199.0)/142500.0;	AA(4,2) = (1329.0-544.0*sqrt(6.0))/2500.0;	AA(4,3) = (131.0*sqrt(6.0)-96.0)/625.0;		AA(4,4) = (6.0-sqrt(6.0))/10.0;

	bb(0) = 0.0;
	bb(1) = 0.0;
	bb(2) = 1.0/9.0;
	bb(3) = (16.0-sqrt(6.0))/36.0;
	bb(4) = (16.0+sqrt(6.0))/36.0;

	cc(0) = (6.0-sqrt(6.0))/10.0;
	cc(1) = (6.0+9.0*sqrt(6.0))/35.0;
	cc(2) = 1.0;
	cc(3) = (4.0-sqrt(6.0))/10.0;
	cc(4) = (4.0+sqrt(6.0))/10.0;

	DiagonallyImplicitRKExport* integrator = createDiagonallyImplicitRKExport(_userInteraction, _commonHeaderName);
	integrator->initializeButcherTableau(AA, bb, cc);

	return integrator;
}

CLOSE_NAMESPACE_ACADO

// end of file.
