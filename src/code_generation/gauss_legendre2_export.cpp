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
 *    \file src/code_generation/gauss_legendre2_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/gauss_legendre2_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussLegendre2Export::GaussLegendre2Export(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ImplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	numStages = 1;
}


GaussLegendre2Export::GaussLegendre2Export(	const GaussLegendre2Export& arg
									) : ImplicitRungeKuttaExport( arg )
{
	numStages = 1;
	copy( arg );
}


GaussLegendre2Export::~GaussLegendre2Export( )
{
	clear( );
}


returnValue GaussLegendre2Export::initializeButcherTableau() {
	AA = Matrix(1,1);
	bb = Vector(1);
	cc = Vector(1);
	
	AA(0,0) = 1.0/2.0;
			
	bb(0) = 1.0;
			
	cc(0) = 1.0/2.0;
	
	return SUCCESSFUL_RETURN;
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createGaussLegendre2Export(	UserInteraction* _userInteraction,
												const String &_commonHeaderName)
{
	return new GaussLegendre2Export(_userInteraction, _commonHeaderName);
}

RegisterGaussLegendre2Export::RegisterGaussLegendre2Export()
{
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_GL2, createGaussLegendre2Export);
}

CLOSE_NAMESPACE_ACADO

// end of file.
