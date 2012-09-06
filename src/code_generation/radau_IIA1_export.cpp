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
 *    \file src/code_generation/radau_IIA1_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/radau_IIA1_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

RadauIIA1Export::RadauIIA1Export(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ImplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	numStages = 1;
}


RadauIIA1Export::RadauIIA1Export(	const RadauIIA1Export& arg
									) : ImplicitRungeKuttaExport( arg )
{
	numStages = 1;
	copy( arg );
}


RadauIIA1Export::~RadauIIA1Export( )
{
	clear( );
}


returnValue RadauIIA1Export::initializeButcherTableau() {
	AA = Matrix(1,1);
	bb = Vector(1);
	cc = Vector(1);
			
	AA(0,0) = 1;				
			
	bb(0) = 1;					
	
	cc(0) = 1;	
	
	return SUCCESSFUL_RETURN;
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createRadauIIA1Export(	UserInteraction* _userInteraction,
											const String &_commonHeaderName)
{
	return new RadauIIA1Export(_userInteraction, _commonHeaderName);
}

RegisterRadauIIA1Export::RegisterRadauIIA1Export()
{
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_RIIA1, createRadauIIA1Export);
}

CLOSE_NAMESPACE_ACADO

// end of file.
