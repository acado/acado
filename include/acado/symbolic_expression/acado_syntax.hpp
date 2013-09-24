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
 *    \file include/acado/symbolic_expression/acado_syntax.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#ifndef ACADO_TOOLKIT_SYNTAX_HPP
#define ACADO_TOOLKIT_SYNTAX_HPP

#include <acado/symbolic_expression/expression.hpp>
#include <acado/symbolic_expression/differential_state.hpp>
#include <acado/symbolic_expression/intermediate_state.hpp>
#include <acado/symbolic_expression/control.hpp>


/**
 *
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 *    This list is a forward declaration of functions that operate on Expressions.
 *    (note that this function are not in a namespace.)
 *    It defines the syntax of ACADO Toolkit.
 *
 */


// ------------------------------------------------------------------------------------
//                               STANDARD OPERATORS:
// ------------------------------------------------------------------------------------


REFER_NAMESPACE_ACADO Expression sin ( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression cos ( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression tan ( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression asin( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression acos( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression atan( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression exp ( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression sqrt( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression ln  ( const REFER_NAMESPACE_ACADO Expression &arg   );
REFER_NAMESPACE_ACADO Expression log ( const REFER_NAMESPACE_ACADO Expression &arg   );


REFER_NAMESPACE_ACADO Expression pow ( const REFER_NAMESPACE_ACADO Expression &arg1,
                                       const REFER_NAMESPACE_ACADO Expression &arg2  );
REFER_NAMESPACE_ACADO Expression pow ( const                       double     &arg1,
                                       const REFER_NAMESPACE_ACADO Expression &arg2  );
REFER_NAMESPACE_ACADO Expression pow ( const REFER_NAMESPACE_ACADO Expression &arg1,
                                       const                       double     &arg2  );



// ---------------------------------------------------------------------------------------------
//                     SPECIAL CONVEX DICIPLINED PROGRAMMING FUNCTIONS:
// ---------------------------------------------------------------------------------------------


REFER_NAMESPACE_ACADO Expression square         ( const REFER_NAMESPACE_ACADO Expression &arg );
REFER_NAMESPACE_ACADO Expression sum_square     ( const REFER_NAMESPACE_ACADO Expression &arg );
REFER_NAMESPACE_ACADO Expression log_sum_exp    ( const REFER_NAMESPACE_ACADO Expression &arg );
REFER_NAMESPACE_ACADO Expression euclidean_norm ( const REFER_NAMESPACE_ACADO Expression &arg );
REFER_NAMESPACE_ACADO Expression entropy        ( const REFER_NAMESPACE_ACADO Expression &arg );



// ---------------------------------------------------------------------------------------------
//                    SPECIAL ROUTINES FOR THE SET UP OF DYNAMIC SYSTEMS:
// ---------------------------------------------------------------------------------------------


REFER_NAMESPACE_ACADO Expression dot ( const REFER_NAMESPACE_ACADO Expression& arg );
REFER_NAMESPACE_ACADO Expression next( const REFER_NAMESPACE_ACADO Expression& arg );



// ---------------------------------------------------------------------------------------------
//                              SYMBOLIC DERIVATIVE OPERATORS:
// ---------------------------------------------------------------------------------------------


REFER_NAMESPACE_ACADO Expression forwardDerivative ( const REFER_NAMESPACE_ACADO Expression &arg1,
                                                     const REFER_NAMESPACE_ACADO Expression &arg2 );

REFER_NAMESPACE_ACADO Expression backwardDerivative( const REFER_NAMESPACE_ACADO Expression &arg1,
                                                     const REFER_NAMESPACE_ACADO Expression &arg2 );

REFER_NAMESPACE_ACADO Expression forwardDerivative ( const REFER_NAMESPACE_ACADO Expression &arg1,
                                                     const REFER_NAMESPACE_ACADO Expression &arg2,
                                                     const REFER_NAMESPACE_ACADO Expression &seed );

REFER_NAMESPACE_ACADO Expression backwardDerivative( const REFER_NAMESPACE_ACADO Expression &arg1,
                                                     const REFER_NAMESPACE_ACADO Expression &arg2,
                                                     const REFER_NAMESPACE_ACADO Expression &seed );

REFER_NAMESPACE_ACADO Expression jacobian           ( const REFER_NAMESPACE_ACADO Expression &arg1,
                                                      const REFER_NAMESPACE_ACADO Expression &arg2 );

REFER_NAMESPACE_ACADO Expression laplace           ( const REFER_NAMESPACE_ACADO Expression &arg1,
                                                     const REFER_NAMESPACE_ACADO Expression &arg2 );


REFER_NAMESPACE_ACADO Matrix ones  ( int nRows, int nCols = 1 );
REFER_NAMESPACE_ACADO Matrix zeros ( int nRows, int nCols = 1 );
REFER_NAMESPACE_ACADO Matrix eye   ( int n                    );

REFER_NAMESPACE_ACADO Matrix diag        ( const REFER_NAMESPACE_ACADO Vector& v );
REFER_NAMESPACE_ACADO Vector diag        ( const REFER_NAMESPACE_ACADO Matrix& M );



REFER_NAMESPACE_ACADO Expression getRiccatiODE( const REFER_NAMESPACE_ACADO Expression        &rhs,
                                                const REFER_NAMESPACE_ACADO DifferentialState &x  ,
                                                const REFER_NAMESPACE_ACADO Control           &u  ,
                                                const REFER_NAMESPACE_ACADO DifferentialState &P  ,
                                                const REFER_NAMESPACE_ACADO Matrix            &Q  ,
                                                const REFER_NAMESPACE_ACADO Matrix            &R   );


REFER_NAMESPACE_ACADO Expression chol( const REFER_NAMESPACE_ACADO Expression &arg );



#endif  // ACADO_TOOLKIT_SYNTAX_HPP

/*
 *   end of file.
 */
