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
 *    \file src/nlp_solver/scp_step.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/nlp_solver/scp_step.hpp>
#include <acado/curve/curve.hpp>
#include <acado/bindings/acado_qpoases/qp_solver_qpoases.hpp>


BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

SCPstep::SCPstep( ) : AlgorithmicBase( )
{
	setupOptions( );
	setupLogging( );

	meritFcn = new SCPmeritFunction;
}


SCPstep::SCPstep( UserInteraction* _userInteraction ) : AlgorithmicBase( _userInteraction )
{
	// setup options and loggings for stand-alone instances
	if ( _userInteraction == 0 )
	{
		setupOptions( );
		setupLogging( );
	}

	meritFcn = new SCPmeritFunction( _userInteraction );
}


SCPstep::SCPstep( const SCPstep& rhs ) : AlgorithmicBase( rhs )
{
	if( rhs.meritFcn != 0 ) meritFcn = new SCPmeritFunction( *(rhs.meritFcn) );
	else                    meritFcn = 0;
}


SCPstep::~SCPstep( )
{
    if( meritFcn != 0 )
		delete meritFcn;
}


SCPstep& SCPstep::operator=( const SCPstep& rhs )
{
    if ( this != &rhs )
    {
		if( meritFcn != 0 )
			delete meritFcn;

		AlgorithmicBase::operator=( rhs );

		if( rhs.meritFcn != 0 ) meritFcn = new SCPmeritFunction( *(rhs.meritFcn) );
		else                    meritFcn = 0;
    }

    return *this;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue SCPstep::setupOptions( )
{
	return SUCCESSFUL_RETURN;
}


returnValue SCPstep::setupLogging( )
{
	return SUCCESSFUL_RETURN;
}


returnValue SCPstep::applyStep(	OCPiterate& iter,
								BandedCP& cp,
								double alpha
								) const
{
    return iter.applyStep( cp.deltaX,alpha );
}


// returnValue SCPstep::getUpdatedFirstControl(	const OCPiterate& iter,
// 												const BandedCP& cp,
// 												double alpha,
// 												DVector& _u
// 												) const
// {
//     DMatrix tmp;
// 
// 	cp.deltaX.getSubBlock( 3*iter.getNumPoints()+0, 0, tmp, _u.getDim(), 1 );
// 
// 	for( uint run1 = 0; run1 < _u.getDim(); run1++ )
// 		_u(run1) += alpha*tmp(run1,0);
// 
// 	return SUCCESSFUL_RETURN;
// }


CLOSE_NAMESPACE_ACADO

// end of file.
