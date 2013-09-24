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
 *    \file src/code_generation/gauss_legendre8_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/gauss_legendre8_export.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussLegendre8Export::GaussLegendre8Export(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ImplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	numStages = 4;
}


GaussLegendre8Export::GaussLegendre8Export(	const GaussLegendre8Export& arg
									) : ImplicitRungeKuttaExport( arg )
{
	numStages = 4;
	copy( arg );
}


GaussLegendre8Export::~GaussLegendre8Export( )
{
	clear( );
}


// PROTECTED:

//
// Register the integrator
//

IntegratorExport* createGaussLegendre8Export(	UserInteraction* _userInteraction,
												const String &_commonHeaderName)
{
	Matrix AA(4,4);
	Vector bb(4);
	Vector cc(4);

	AA(0,0) = (1/(double)144)*(double)sqrt((double)(double)30)+(double)1/(double)8;
	AA(0,1) = -(double)(1/(double)840)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)105)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))+(double)1/(double)8;		
	AA(0,2) = (1/(double)2352)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)144)*(double)sqrt((double)(double)30)+(double)(1/(double)1680)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)1/(double)8+(double)(1/(double)1470)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))-(double)(1/(double)420)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30));	
	AA(0,3) = -(double)(1/(double)2352)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)144)*(double)sqrt((double)(double)30)+(double)(1/(double)1680)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)1/(double)8-(double)(1/(double)1470)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))-(double)(1/(double)420)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30));
	AA(1,0) = (1/(double)840)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)144)*(double)sqrt((double)(double)30)+(double)(1/(double)105)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))+(double)1/(double)8;			
	AA(1,1) = (1/(double)144)*(double)sqrt((double)(double)30)+(double)1/(double)8;		
	AA(1,2) = (1/(double)2352)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)1680)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)1/(double)8+(double)(1/(double)1470)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))+(double)(1/(double)420)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30));
	AA(1,3) = -(double)(1/(double)2352)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)1680)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)1/(double)8-(double)(1/(double)1470)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))+(double)(1/(double)420)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30));
	AA(2,0) = -(double)(1/(double)2352)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)1680)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)1470)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))+(double)1/(double)8-(double)(1/(double)420)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30));	
	AA(2,1) = (1/(double)2352)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)+(double)(1/(double)144)*(double)sqrt((double)(double)30)-(double)(1/(double)1680)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30))*(double)sqrt((double)(double)30)-(double)(1/(double)1470)*(double)sqrt((double)(double)525-(double)70*(double)sqrt((double)(double)30))+(double)1/(double)8-(double)(1/(double)420)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)(double)30));				
	AA(2,2) = -(double)(1/(double)144)*(double)sqrt((double)(double)30)+(double)1/(double)8;	
	AA(2,3) = (1/(double)840)*(double)sqrt((double)(double)525+(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)-(double)(1/(double)144)*(double)sqrt((double)30)-(double)(1/(double)105)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))+(double)1/(double)8;
	AA(3,0) = -(double)(1/(double)2352)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)+(double)(1/(double)144)*(double)sqrt((double)30)+(double)(1/(double)1680)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)+(double)(1/(double)1470)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30))+(double)1/(double)8+(double)(1/(double)420)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30));
	AA(3,1) = (1/(double)2352)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)+(double)(1/(double)144)*(double)sqrt((double)30)+(double)(1/(double)1680)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)-(double)(1/(double)1470)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30))+(double)1/(double)8+(double)(1/(double)420)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30));
	AA(3,2) = -(double)(1/(double)840)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))*(double)sqrt((double)30)-(double)(1/(double)144)*(double)sqrt((double)30)+(double)(1/(double)105)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30))+(double)1/(double)8;
	AA(3,3) = -(double)(1/(double)144)*(double)sqrt((double)30)+(double)1/(double)8;		

	bb(0) = (1/(double)72)*(double)sqrt((double)30)+(double)1/(double)4;		
	bb(1) = (1/(double)72)*(double)sqrt((double)30)+(double)1/(double)4;					
	bb(2) = -(double)(1/(double)72)*(double)sqrt((double)30)+(double)1/(double)4;
	bb(3) = -(double)(1/(double)72)*(double)sqrt((double)30)+(double)1/(double)4;

	cc(0) = 1/(double)2-(double)(1/(double)70)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30));		
	cc(1) = 1/(double)2+(double)(1/(double)70)*(double)sqrt((double)525-(double)70*(double)sqrt((double)30));	
	cc(2) = 1/(double)2-(double)(1/(double)70)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30));
	cc(3) = 1/(double)2+(double)(1/(double)70)*(double)sqrt((double)525+(double)70*(double)sqrt((double)30));

	ImplicitRungeKuttaExport* integrator = createImplicitRungeKuttaExport(_userInteraction, _commonHeaderName);
	integrator->initializeButcherTableau(AA, bb, cc);

	return integrator;
}

RegisterGaussLegendre8Export::RegisterGaussLegendre8Export()
{
	IntegratorExportFactory::instance().registerAlgorithm(INT_IRK_GL8, createGaussLegendre8Export);
}

CLOSE_NAMESPACE_ACADO

// end of file.
