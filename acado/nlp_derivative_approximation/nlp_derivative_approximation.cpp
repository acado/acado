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
 *    \file src/nlp_derivative_approximation/nlp_derivative_approximation.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_derivative_approximation/nlp_derivative_approximation.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

NLPderivativeApproximation::NLPderivativeApproximation( ) : AlgorithmicBase( )
{
	setupOptions( );
	setupLogging( );

	hessianScaling = 1.0;
}


NLPderivativeApproximation::NLPderivativeApproximation( UserInteraction* _userInteraction ) : AlgorithmicBase( _userInteraction )
{
	// setup options and loggings for stand-alone instances
	if ( _userInteraction == 0 )
	{
		setupOptions( );
		setupLogging( );
	}

	hessianScaling = 1.0;
}


NLPderivativeApproximation::NLPderivativeApproximation( const NLPderivativeApproximation& rhs ) : AlgorithmicBase( rhs )
{
	hessianScaling = rhs.hessianScaling;
}


NLPderivativeApproximation::~NLPderivativeApproximation( )
{
}


NLPderivativeApproximation& NLPderivativeApproximation::operator=( const NLPderivativeApproximation& rhs )
{

	if ( this != &rhs )
	{
		AlgorithmicBase::operator=( rhs );

		hessianScaling = rhs.hessianScaling;
	}

    return *this;
}




//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue NLPderivativeApproximation::setupOptions( )
{
	return SUCCESSFUL_RETURN;
}


returnValue NLPderivativeApproximation::setupLogging( )
{
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
