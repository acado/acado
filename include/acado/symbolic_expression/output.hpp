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
*    \file include/acado/symbolic_expression/output.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#ifndef ACADO_TOOLKIT_OUTPUT_HPP
#define ACADO_TOOLKIT_OUTPUT_HPP


#include <acado/symbolic_expression/expression.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements a differential state.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Output implements a differential state variable within the 
 *	family of Expressions.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class Output : public Expression{

public:

    /** Default constructor */
    Output();

    /** Default constructor */
    Output( uint nRows_, uint nCols_ = 1, String name_ = "" );

    /** vector constructor */
    Output( uint nRows_, String name_ );

    /** scalar constructor */
    Output( String name_ );

	/** expression constructor */
    Output( const Expression& _expression,
			uint componentIdx = 0
			);

	
    /** Copy constructor (deep copy). */
    Output( const Output &arg );

    /** Default destructor. */
    virtual ~Output();

    /** Assignment Operator (deep copy). */
    Output& operator=( const Output &arg );


     /** Provides a deep copy of the expression. \n
      *  \return a clone of the expression.      \n
      */
     virtual Expression* clone() const;


     /** This function clears all static counters. Although this \n
      *  function is public it should never be used in C-code.   \n
      *  It is necessary for some Matlab-specific interfaces.    \n
      *  Please have a look into the header file                 \n
      *  include/global/utils/matlab_acado_utils.hpp             \n
      *  for more details.                                       \n
      */
     returnValue clearStaticCounters();


//
//  PROTECTED MEMBERS:
//

protected:

     static int count;

};


CLOSE_NAMESPACE_ACADO



#endif
