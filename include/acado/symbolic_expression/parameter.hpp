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
*    \file include/acado/symbolic_expression/parameter.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#ifndef ACADO_TOOLKIT_PARAMETER_HPP
#define ACADO_TOOLKIT_PARAMETER_HPP


#include <acado/symbolic_expression/expression.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements a parameter.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Parameter implements a parameter within the 
 *	family of Expressions.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class Parameter : public Expression{

public:

    /** Default constructor */
    Parameter();

    /** Default constructor */
    Parameter( uint nRows_, uint nCols_ = 1, String name_ = "" );

    /** Copy constructor (deep copy). */
    Parameter( const Parameter &arg );

    /** Default destructor. */
    virtual ~Parameter();

    /** Assignment Operator (deep copy). */
    Parameter& operator=( const Parameter &arg );


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
