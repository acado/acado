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
 *    \file src/nlp_solver/ip_method.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <nlp_solver/ip_method.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

IPmethod::IPmethod( ) : NLPsolver( )
{
}


IPmethod::IPmethod( UserInteraction* _userInteraction ) : NLPsolver( _userInteraction )
{
}


IPmethod::IPmethod( const NLP& nlp_ ) : NLPsolver()
{
}


IPmethod::IPmethod( const IPmethod& rhs ) : NLPsolver( )
{
}


IPmethod::~IPmethod( )
{
}


IPmethod& IPmethod::operator=( const IPmethod& rhs )
{
	if ( this != &rhs )
	{
		NLPsolver::operator=( rhs );
    }
	return *this;
}


returnValue IPmethod::init(VariablesGrid    *xd,
                                 VariablesGrid    *xa,
                                 VariablesGrid    *p,
                                 VariablesGrid    *u,
                                 VariablesGrid    *w   )
{
    return THROWERROR( RET_NOT_IMPLEMENTED_YET );
}


returnValue IPmethod::start( int maxNumSteps )
{
	return THROWERROR( RET_NOT_IMPLEMENTED_YET );
}




//
// PROTECTED MEMBER FUNCTIONS:
//





CLOSE_NAMESPACE_ACADO

// end of file.
