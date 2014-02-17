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
 *    \file src/function/output_fcn.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 31.05.2008
 */



#include <acado/function/function_.hpp>
#include <acado/function/output_fcn.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

OutputFcn::OutputFcn( ) : Function( )
{
}


OutputFcn::OutputFcn( const OutputFcn& rhs ) : Function( rhs )
{
}


OutputFcn::~OutputFcn( )
{
}


OutputFcn& OutputFcn::operator=( const OutputFcn& rhs )
{
    if ( this != &rhs )
    {
		Function::operator=( rhs );
    }

    return *this;
}


Output OutputFcn::operator()(	uint componentIdx
								)
{
	Operator* componentOperator = getExpression( componentIdx );

	Expression tmp;
	
	if ( componentOperator != 0 )
	{
		tmp = *componentOperator;
		delete componentOperator;
	}

	return Output( tmp,componentIdx );
}


returnValue OutputFcn::evaluate( double *x, double *_result ){

    return Function::evaluate( 0, x, _result );
}



returnValue OutputFcn::evaluate( const VariablesGrid *x ,
                                 const VariablesGrid *xa,
                                 const VariablesGrid *p ,
                                 const VariablesGrid *u ,
                                 const VariablesGrid *w ,
                                 VariablesGrid       *_result ){

    int run1;

    OCPiterate iter( x, xa, p, u, w );
    const int N = iter.getNumPoints();
    _result->init( getDim(), iter.getGrid() );

    EvaluationPoint z( *this, iter );

    for( run1 = 0; run1 < N; run1++ ){

        z.setZ( run1, iter ); 
        _result->setVector( run1, Function::evaluate(z) );
    }

    return SUCCESSFUL_RETURN;
}



DVector OutputFcn::evaluate( const EvaluationPoint &x,
                            const int              &number ){

    return Function::evaluate( x, number );
}




//
// PROTECTED MEMBER FUNCTIONS:
//





CLOSE_NAMESPACE_ACADO

// end of file.
