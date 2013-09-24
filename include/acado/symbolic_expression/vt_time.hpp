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
*    \file include/acado/symbolic_expression/vt_time.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#ifndef ACADO_TOOLKIT_VT_TIME_HPP
#define ACADO_TOOLKIT_VT_TIME_HPP


#include <acado/symbolic_expression/expression.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements the time within the family of Expressions.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class TIME implements the time within the family of Expressions.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class TIME : public Expression{

public:

    /** Default constructor */
    TIME();

    /** Default constructor */
    TIME( uint nRows_, uint nCols_ = 1, String name_ = "" );

    /** Copy constructor (deep copy). */
    TIME( const TIME &arg );

    /** Default destructor. */
    virtual ~TIME();

    /** Assignment Operator (deep copy). */
    TIME& operator=( const TIME &arg );


     /** Provides a deep copy of the expression. \n
      *  \return a clone of the expression.      \n
      */
     virtual Expression* clone() const;


//
//  PROTECTED MEMBERS:
//

protected:

};


CLOSE_NAMESPACE_ACADO



#endif
