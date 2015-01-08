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
 *    \file src/nlp_solver/scp_step_fullstep.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_solver/scp_step_fullstep.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

SCPstepFullstep::SCPstepFullstep( ) : SCPstep( )
{
}


SCPstepFullstep::SCPstepFullstep( UserInteraction* _userInteraction ) : SCPstep( _userInteraction )
{
}


SCPstepFullstep::SCPstepFullstep( const SCPstepFullstep& rhs ) : SCPstep( rhs )
{
}


SCPstepFullstep::~SCPstepFullstep( )
{
}


SCPstepFullstep& SCPstepFullstep::operator=( const SCPstepFullstep& rhs )
{
    if ( this != &rhs )
    {
		SCPstep::operator=( rhs );
    }

    return *this;
}




SCPstep* SCPstepFullstep::clone() const
{
	return new SCPstepFullstep( *this );
}



returnValue SCPstepFullstep::performStep(	OCPiterate& iter,
        									BandedCP& cp,
        									SCPevaluation* eval
											)
{
// 	RealClock clock;
// 	clock.reset();
// 	clock.start();
	
	double meritFcnValue;
	meritFcn->evaluate( 1.0,iter,cp,*eval, meritFcnValue );
	setLast( LOG_LINESEARCH_STEPLENGTH, 1.0 );
	setLast( LOG_MERIT_FUNCTION_VALUE,meritFcnValue );
// 	clock.stop();
// 	printf("meritFcn time = %e\n", clock.getTime());
	
	
// 	clock.reset();
// 	clock.start();
	returnValue returnvalue = applyStep( iter,cp,1.0 );
	if( returnvalue != SUCCESSFUL_RETURN )
		ACADOERROR( returnvalue );
// 	clock.stop();
// 	printf("applyStep time = %e\n", clock.getTime());
	
	
	return SUCCESSFUL_RETURN;
}




//
// PROTECTED MEMBER FUNCTIONS:
//




CLOSE_NAMESPACE_ACADO

// end of file.
