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
#include <acado/symbolic_expression/symbolic_expression.hpp>


USING_NAMESPACE_ACADO


// ------------------------------------------------------------------------------------
//                               STANDARD OPERATORS:
// ------------------------------------------------------------------------------------


const Expression sin ( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getSin (); }
const Expression cos ( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getCos (); }
const Expression tan ( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getTan (); }
const Expression asin( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getAsin(); }
const Expression acos( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getAcos(); }
const Expression atan( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getAtan(); }
const Expression exp ( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getExp (); }
const Expression sqrt( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getSqrt(); }
const Expression ln  ( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getLn  (); }
const Expression log ( const Expression &arg ){ ASSERT( arg.size() == 1 ); return arg(0).getLn  (); }

const Expression pow( const Expression &arg1, const Expression &arg2 ){
  
  ASSERT( arg1.size() == 1 );
  ASSERT( arg2.size() == 1 );
  
  return arg1(0).getPow(arg2(0));
}

const Expression pow( const Expression &arg1, const double &arg2 ){
  
    ASSERT( arg1.size() == 1 );
  
    if( fabs( arg2 - floor(arg2) ) <= 10.0*EPS ){
        int intarg = (int) floor(arg2);
        return arg1(0).getPowInt( intarg );
    }
    if( fabs( arg2 - ceil(arg2) ) <= 10.0*EPS ){
        int intarg = (int) ceil(arg2);
        return arg1(0).getPowInt( intarg );
    }
    return arg1(0).getPow(arg2);
}



// ---------------------------------------------------------------------------------------------
//                              SYMBOLIC DERIVATIVE OPERATORS:
// ---------------------------------------------------------------------------------------------


Expression forwardDerivative( const Expression &arg1,
                              const Expression &arg2  ){

    return multipleForwardDerivative(arg1,arg2,eye<double>(arg2.size()));
}

Expression forwardDerivative( const Expression &arg1,
                              const Expression &arg2,
                              const Expression &seed  ){

    if( arg1.size() == 1 ) return arg1(0).AD_forward(arg2,seed);
    
    Expression df(arg1.rows(),arg1.cols());
    for( int i=0; i<arg1.size(); ++i ) df(i) = arg1(i).AD_forward(arg2,seed);
    return df;
}

Expression multipleForwardDerivative( const Expression &arg1,
                                      const Expression &arg2,
                                      const Expression &seed  ){

    Expression tmp = forwardDerivative( arg1, arg2, seed.col(0) );
    
    for( int i = 1; i < seed.cols(); ++i ) {
        tmp.appendCols( forwardDerivative( arg1, arg2, seed.col(i) ) );
    }
    return tmp;
}


Expression backwardDerivative( const Expression &arg1,
                               const Expression &arg2  ){

    return multipleBackwardDerivative(arg1,arg2,eye<double>(arg1.size()));
}



Expression backwardDerivative( const Expression &arg1,
                               const Expression &arg2,
                               const Expression &seed  ){

    return arg1.AD_backward(arg2,seed);
}


Expression multipleBackwardDerivative( const Expression &arg1,
                                       const Expression &arg2,
                                       const Expression &seed  ){
  
    Expression tmp = backwardDerivative( arg1, arg2, seed.col(0) );
        
    for( int i = 1; i < seed.cols(); i++ ) {
        tmp.appendCols( backwardDerivative( arg1, arg2, seed.col(i) ) );
    }
    return tmp;
}


// 
// ScalarExpression symmetricDerivative( 	const ScalarExpression &arg1,
//  	 	 	 	 	 	 	 	 	const ScalarExpression &arg2,
//  	 	 	 	 	 	 	 	 	const ScalarExpression &forward_seed,
//  	 	 	 	 	 	 	 	 	const ScalarExpression &backward_seed,
//  	 	 	 	 	 	 	 	 	ScalarExpression *forward_result,
//  	 	 	 	 	 	 	 	 	ScalarExpression *backward_result ) {
// 	return arg1.ADsymmetric( arg2, forward_seed, backward_seed, forward_result, backward_result );
// }
// 

Expression jacobian( const Expression &arg1,
                     const Expression &arg2 ){

    return backwardDerivative( arg1, arg2 );
}


Expression laplace( const Expression &arg1,
                    const Expression &arg2 ){

    return forwardDerivative( forwardDerivative(arg1,arg2), arg2 );
}


Expression getRiccatiODE( const Expression &rhs ,
                          const Expression &x   ,
                          const Expression &u   ,
                          const Expression &P   ,
                          const Expression &Q   ,
                          const Expression &Rinv ){

    Expression A = forwardDerivative( rhs, x );
    Expression B = forwardDerivative( rhs, u );
    
    Expression C1 = Rinv*B.transpose();
    Expression C2 = B*C1;
    Expression C3 = P*C2;
    Expression C4 = C3*P;

    return A.transpose()*P + P*A + Q - C4;
}


Expression chol( const Expression &arg ){
  
    return arg.llt().matrixL();
}


// end of file.


