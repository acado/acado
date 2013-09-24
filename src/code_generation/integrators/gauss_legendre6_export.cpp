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
 *    \file src/code_generation/gauss_legendre6_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/gauss_legendre6_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussLegendre6Export::GaussLegendre6Export(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ImplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	numStages = 3;
}


GaussLegendre6Export::GaussLegendre6Export(	const GaussLegendre6Export& arg
									) : ImplicitRungeKuttaExport( arg )
{
	numStages = 3;
	copy( arg );
}


GaussLegendre6Export::~GaussLegendre6Export( )
{
	clear( );
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createGaussLegendre6Export(	UserInteraction* _userInteraction,
												const String &_commonHeaderName)
{
	Matrix AA(3,3);
	Vector bb(3);
	Vector cc(3);

	AA(0,0) = 5.0/36.0;
	AA(0,1) = 2.0/9.0-1.0/15.0*sqrt(15.0);
	AA(0,2) = 5.0/36.0-1.0/30.0*sqrt(15.0);
	AA(1,0) = 5.0/36.0+1.0/24.0*sqrt(15.0);
	AA(1,1) = 2.0/9.0;
	AA(1,2) = 5.0/36.0-1.0/24.0*sqrt(15.0);
	AA(2,0) = 5.0/36.0+1.0/30.0*sqrt(15.0);
	AA(2,1) = 2.0/9.0+1.0/15.0*sqrt(15.0);
	AA(2,2) = 5.0/36.0;

	bb(0) = 5.0/18.0;
	bb(1) = 4.0/9.0;
	bb(2) = 5.0/18.0;

	cc(0) = 1.0/2.0-sqrt(15.0)/10.0;
	cc(1) = 1.0/2.0;
	cc(2) = 1.0/2.0+sqrt(15.0)/10.0;

	ImplicitRungeKuttaExport* integrator = createImplicitRungeKuttaExport(_userInteraction, _commonHeaderName);
	integrator->initializeButcherTableau(AA, bb, cc);

	return integrator;
}

RegisterGaussLegendre6Export::RegisterGaussLegendre6Export()
{
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_GL6, createGaussLegendre6Export);
}

CLOSE_NAMESPACE_ACADO

// end of file.
