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
 *    \file src/symbolic_expression/expression.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Milan Vukov
 */

#include <acado/symbolic_expression/acado_syntax.hpp>
#include <acado/symbolic_operator/operator.hpp>

USING_NAMESPACE_ACADO


// ------------------------------------------------------------------------------------
//                               STANDARD OPERATORS:
// ------------------------------------------------------------------------------------


IntermediateState sin ( const Expression &arg ){ return arg.getSin(); }
IntermediateState cos ( const Expression &arg ){ return arg.getCos (); }
IntermediateState tan ( const Expression &arg ){ return arg.getTan (); }
IntermediateState asin( const Expression &arg ){ return arg.getAsin(); }
IntermediateState acos( const Expression &arg ){ return arg.getAcos(); }
IntermediateState atan( const Expression &arg ){ return arg.getAtan(); }
IntermediateState exp ( const Expression &arg ){ return arg.getExp (); }
IntermediateState sqrt( const Expression &arg ){ return arg.getSqrt(); }
IntermediateState ln  ( const Expression &arg ){ return arg.getLn  (); }
IntermediateState log ( const Expression &arg ){ return arg.getLn  (); }

IntermediateState pow( const Expression &arg1, const Expression &arg2 ){
  
  return arg1.getPow(arg2);
}

IntermediateState pow( const double &arg1, const Expression &arg2 ){
  
  return Expression( arg1 ).getPow( arg2 );
}

IntermediateState pow( const Expression &arg1, const double &arg2 ){
  
    if( fabs( arg2 - floor(arg2) ) <= 10.0*EPS ){
        int intarg = (int) floor(arg2);
        return arg1.getPowInt( intarg );
    }
    if( fabs( arg2 - ceil(arg2) ) <= 10.0*EPS ){
        int intarg = (int) ceil(arg2);
        return arg1.getPowInt( intarg );
    }
    return arg1.getPow( arg2 );
}



// ---------------------------------------------------------------------------------------------
//                     SPECIAL CONVEX DICIPLINED PROGRAMMING FUNCTIONS:
// ---------------------------------------------------------------------------------------------


IntermediateState square         ( const Expression &arg ){ return arg.getSumSquare    (); }
IntermediateState sum_square     ( const Expression &arg ){ return arg.getSumSquare    (); }
IntermediateState log_sum_exp    ( const Expression &arg ){ return arg.getLogSumExp    (); }
IntermediateState euclidean_norm ( const Expression &arg ){ return arg.getEuclideanNorm(); }
IntermediateState entropy        ( const Expression &arg ){ return arg.getEntropy      (); }



// ---------------------------------------------------------------------------------------------
//                    SPECIAL ROUTINES FOR THE SET UP OF DYNAMIC SYSTEMS:
// ---------------------------------------------------------------------------------------------



Expression dot ( const Expression &arg ){ return arg.getDot (); }
Expression next( const Expression &arg ){ return arg.getNext(); }

// ---------------------------------------------------------------------------------------------
//                              SYMBOLIC DERIVATIVE OPERATORS:
// ---------------------------------------------------------------------------------------------


Expression forwardDerivative( const Expression &arg1,
                              const Expression &arg2  ){

    return arg1.ADforward(arg2);
}


Expression backwardDerivative( const Expression &arg1,
                               const Expression &arg2  ){

    return arg1.ADbackward(arg2);
}


Expression forwardDerivative( const Expression &arg1,
                              const Expression &arg2,
                              const Expression &seed  ){

    return arg1.ADforward(arg2,seed);
}


Expression multipleForwardDerivative( const Expression &arg1,
                              const Expression &arg2,
                              const Expression &seed  ){

	Expression tmp;
	for( uint i = 0; i < seed.getNumCols(); i++ ) {
		tmp.appendCols( forwardDerivative( arg1, arg2, seed.getCol(i) ) );
	}
    return tmp;
}


Expression backwardDerivative( const Expression &arg1,
                               const Expression &arg2,
                               const Expression &seed  ){

    return arg1.ADbackward(arg2,seed);
}


Expression multipleBackwardDerivative( const Expression &arg1,
                              const Expression &arg2,
                              const Expression &seed  ){

	Expression tmp;
	for( uint i = 0; i < seed.getNumCols(); i++ ) {
		tmp.appendCols( backwardDerivative( arg1, arg2, seed.getCol(i) ) );
	}
    return tmp;
}

Expression symmetricDerivative( 	const Expression &arg1,
 	 	 	 	 	 	 	 	 	const Expression &arg2,
 	 	 	 	 	 	 	 	 	const Expression &forward_seed,
 	 	 	 	 	 	 	 	 	const Expression &backward_seed,
 	 	 	 	 	 	 	 	 	Expression *forward_result,
 	 	 	 	 	 	 	 	 	Expression *backward_result ) {
	return arg1.ADsymmetric( arg2, forward_seed, backward_seed, forward_result, backward_result );
}


Expression jacobian          ( const Expression &arg1,
                               const Expression &arg2 ){

    return backwardDerivative( arg1, arg2 );
}


Expression laplace           ( const Expression &arg1,
                               const Expression &arg2 ){

    return forwardDerivative( forwardDerivative(arg1,arg2), arg2 );
}

Expression getRiccatiODE( const Expression        &rhs,
                          const DifferentialState &x  ,
                          const Control           &u  ,
                          const DifferentialState &P  ,
                          const DMatrix            &Q  ,
                          const DMatrix            &R  ){

	IntermediateState RHS("", x.getDim(), x.getDim());

    IntermediateState A = forwardDerivative( rhs, x );
    IntermediateState B = forwardDerivative( rhs, u );

    return A.transpose()*P + P*A + Q - P*B*(DMatrix(R.inverse()))*B.transpose()*P;
}


Expression chol( const Expression &arg ){

    // --------------------------------------------------
    // COMPUTES THE CHOLESKY FACTOR L OF THE ARGUMENT
    // SUCH THAT
    //
    //               ARG = L * L.transpose
    //
    // WHERE THE MATRIX L IS RETURNED.
    // THIS IS A SYMBOLIC CHOLESKY ROUTINE AS ACADO
    // SIMPLIFIES PRODUCTS WITH ZERO AUTOMATICALLY.
    //
    // CAUTION: THE ROUTINE WORKS ON A SYMBOLIC LEVEL
    //          ALWAYS WITHOUT DETECTING PSD-MATRICES.
    //          HOWEVER, THE EVEALUATION WILL ONLY WORK
    //          FOR SYMMETRIC AND POSITIVE SEMI-DEFINITE
    //          MATRICES.
    //          THE SAFE-GUARD CONSTANT IS 100*EPS.
    // --------------------------------------------------


    ASSERT( arg.getNumRows() == arg.getNumCols() );

    int dim = arg.getNumRows();
    IntermediateState L("", dim,dim);

    // COMPUTE THE LOWER TRIANGLE RECURSIVELY:
    // ---------------------------------------

    int i,j,k;

    for( j = 0; j < dim; j++ ){
                                  L(j,j)  = arg(j,j)       ;
        for( k = 0; k < j; k++ )  L(j,j) -= L(j,k)*L(j,k)  ;
                                  L(j,j)  = sqrt( L(j,j) + 100.0*EPS );
        for( i = j+1; i < dim; i++ ){
                                      L(i,j)  = arg(i,j)       ;
            for( k = 0; k < j; k++ )  L(i,j) -= L(i,k)*L(j,k)  ;
                                      L(i,j)  = L(i,j)/(L(j,j) + 100.0*EPS );
        }
    }

    for( j = 0; j < dim; j++ )
        for( k = j+1; k < dim; k++ )
             L(j,k) = 0.0;

    return L;
}


returnValue clearAllStaticCounters()
{
	AlgebraicState              dummy1;
	Control                     dummy2;
	DifferentialState           dummy3;
	DifferentialStateDerivative dummy4;
	Disturbance                 dummy5;
	IntegerControl              dummy6;
	IntegerParameter            dummy7;
	IntermediateState           dummy8;
	Parameter                   dummy9;
	OnlineData                  dummy10;

	dummy1.clearStaticCounters();
	dummy2.clearStaticCounters();
	dummy3.clearStaticCounters();
	dummy4.clearStaticCounters();
	dummy5.clearStaticCounters();
	dummy6.clearStaticCounters();
	dummy7.clearStaticCounters();
	dummy8.clearStaticCounters();
	dummy9.clearStaticCounters();
	dummy10.clearStaticCounters();

	return SUCCESSFUL_RETURN;
}


// end of file.


