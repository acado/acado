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
*    \file include/acado/symbolic_operator/asin.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#ifndef ACADO_TOOLKIT_ASIN_HPP
#define ACADO_TOOLKIT_ASIN_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements the scalar inverse sine operator (arcsin) within the symbolic operators family.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Asin implements the scalar inverse sine operator (arcsin) within the 
 *	symbolic operators family.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */
class Asin : public UnaryOperator{

public:

    /** Default constructor. */
    Asin();

    /** Default constructor. */
    Asin( Operator *_argument );

    /** Copy constructor (deep copy). */
    Asin( const Asin &arg );

    /** Default destructor. */
    ~Asin();

    /** Assignment Operator (deep copy). */
    Asin& operator=( const Asin &arg );

	/** Evaluates the expression (templated version) */
	virtual returnValue evaluate( EvaluationBase *x );
	
	
	
    /** Returns the derivative of the expression with respect     \n
     *  to the variable var(index).                               \n
     *  \return The expression for the derivative.                \n
     *
     */
     virtual Operator* differentiate( int index  /**< diff. index    */ );



    /** Substitutes var(index) with the expression sub.           \n
     *  \return The substituted expression.                       \n
     *
     */
     virtual Operator* substitute( int index             /**< subst. index    */,
                                     const Operator *sub /**< the substitution*/);


     /** Provides a deep copy of the expression. \n
      *  \return a clone of the expression.      \n
      */
     virtual Operator* clone() const;


//
//  PROTECTED FUNCTIONS:
//

protected:



    /** Automatic Differentiation in forward mode on the symbolic \n
     *  level. This function generates an expression for a        \n
     *  forward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                 \n
     */
     virtual Operator* ADforwardProtected( int                dim      , /**< dimension of the seed */
                                             VariableType      *varType  , /**< the variable types    */
                                             int               *component, /**< and their components  */
                                             Operator       **seed     , /**< the forward seed      */
                                             int                &nNewIS  , /**< the number of new IS  */
                                             TreeProjection ***newIS    /**< the new IS-pointer    */ );



    /** Automatic Differentiation in backward mode on the symbolic \n
     *  level. This function generates an expression for a         \n
     *  backward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                  \n
     */
     virtual returnValue ADbackwardProtected( int           dim      , /**< number of directions  */
                                              VariableType *varType  , /**< the variable types    */
                                              int          *component, /**< and their components  */
                                              Operator   *seed     , /**< the backward seed     */
                                              Operator  **df         /**< the result            */ );

};


CLOSE_NAMESPACE_ACADO



#endif
