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
*    \file include/acado/symbolic_operator/addition.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*/


#ifndef ACADO_TOOLKIT_ADDITION_HPP
#define ACADO_TOOLKIT_ADDITION_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements the scalar addition operator within the symbolic expressions family.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Addition implements the scalar addition operator within the 
 *	symbolic expressions family.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */
class Addition : public BinaryOperator{

public:

    /** Default constructor. */
    Addition();

    /** Default constructor. */
    Addition( const SharedOperator &_argument1, const SharedOperator &_argument2 );

    /** Copy constructor. */
    Addition( const Addition &arg );

    /** Default destructor. */
    ~Addition();

    /** Evaluates the expression (templated version) */
    virtual returnValue evaluate( EvaluationBase *x );

    /** Substitutes key with the expression sub. \n
     *  \return The substituted expression.      \n
     *
     */
    virtual SharedOperator substitute( SharedOperatorMap &sub /**< the substitution */ );


    /** Prints the expression into a stream. \n
     *  \return SUCCESFUL_RETURN             \n
     */
    virtual std::ostream& print( std::ostream &stream, StringMap &name ) const;

     
    /** Initializes the derivative operators. */
    virtual returnValue initDerivative();

};


CLOSE_NAMESPACE_ACADO

#endif
