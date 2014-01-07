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
 *    \file src/nlp_solver/scp_step_linesearch.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_solver/scp_step_linesearch.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

SCPstepLinesearch::SCPstepLinesearch( ) : SCPstep( )
{
}


SCPstepLinesearch::SCPstepLinesearch( UserInteraction* _userInteraction ) : SCPstep( _userInteraction )
{
}


SCPstepLinesearch::SCPstepLinesearch( const SCPstepLinesearch& rhs ) : SCPstep( rhs )
{
}


SCPstepLinesearch::~SCPstepLinesearch( )
{
}


SCPstepLinesearch& SCPstepLinesearch::operator=( const SCPstepLinesearch& rhs )
{
    if ( this != &rhs )
    {
		SCPstep::operator=( rhs );
    }

    return *this;
}




SCPstep* SCPstepLinesearch::clone() const
{
	return new SCPstepLinesearch( *this );
}



returnValue SCPstepLinesearch::performStep(	OCPiterate& iter,
        									BandedCP& cp,
        									SCPevaluation* eval
											)
{
	returnValue returnvalue;

	double alpha = 1.0;
	double lineSearchTOL, alphaMin;

	get( LINESEARCH_TOLERANCE    , lineSearchTOL );
	get( MIN_LINESEARCH_PARAMETER, alphaMin      );

// 	iter.print();
	
	if( eval->getKKTtolerance( iter,cp ) > lineSearchTOL )
	{
		returnvalue = performLineSearch( iter,cp,*eval, alpha,alphaMin );
		if( returnvalue != SUCCESSFUL_RETURN ) ACADOERROR(returnvalue);
	}
	else
	{
		returnvalue = performLineSearch( iter,cp,*eval, alpha,1.0 );
		if( returnvalue != SUCCESSFUL_RETURN ) ACADOERROR(returnvalue);
	}
	setLast( LOG_LINESEARCH_STEPLENGTH, alpha );

	returnvalue = applyStep( iter,cp,alpha );
	if( returnvalue != SUCCESSFUL_RETURN )
		ACADOERROR( returnvalue );


	return SUCCESSFUL_RETURN;
}




//
// PROTECTED MEMBER FUNCTIONS:
//



returnValue SCPstepLinesearch::performLineSearch(	const OCPiterate& iter,
													BandedCP& cp,
													SCPevaluation& eval,
													double& alpha,
													const double& alphaMin
													)
{

    const int     maxIter = 50;
    const double  kappa   = 0.5;

    alpha   = 1.0;

    double meritFcnValue1 = INFTY;
	double meritFcnValue2 = INFTY;

	if ( meritFcn->evaluate( 0.0,iter,cp,eval, meritFcnValue1 ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNKNOWN_BUG );

    int run1 = 0;
    while( run1 < maxIter ){
        meritFcn->evaluate( alpha,iter,cp,eval, meritFcnValue2 );

//         acadoPrintf("LineSearch: T1 = %.16e  T2 = %.16e  alpha = %.3e \n", meritFcnValue1, meritFcnValue2, alpha  );

        if( meritFcnValue2 <= meritFcnValue1 + EPS ) break;
        if( alpha <= alphaMin+EPS ) break;
        alpha *= kappa;
        run1++;
    }

	setLast( LOG_MERIT_FUNCTION_VALUE,meritFcnValue2 );

    return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO

// end of file.
