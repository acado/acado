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
 *    \file include/acado/symbolic_operator/smooth_operator.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 */


#ifndef ACADO_TOOLKIT_SMOOTH_OPERATOR_HPP
#define ACADO_TOOLKIT_SMOOTH_OPERATOR_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO



/**
 *	\brief Abstract base class for all scalar-valued symbolic operators.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Operator serves as an abstract base class for all scalar-valued
 *	symbolic operators.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class SmoothOperator : public Operator{

public:


           /** Default constructor. */
           SmoothOperator();

    virtual ~SmoothOperator();


    /** Evaluates the expression (templated version) */
    virtual returnValue evaluate( EvaluationBase *x ) = 0;


    /** Automatic Differentiation in forward mode on the symbolic \n
     *  level. This function generates an expression for a        \n
     *  forward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                 \n
     */
     virtual SharedOperator AD_forward( SharedOperatorMap &seed /**< the forward seed   */ ) = 0;



    /** Automatic Differentiation in backward mode on the symbolic \n
     *  level. This function generates an expression for a         \n
     *  backward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                  \n
     */
    virtual returnValue AD_backward( SharedOperator     &seed, /**< the backward seed  */
                                     SharedOperatorMap  &df  , /**< the result         */
                                     SharedOperatorMap2 &IS    /**< the new IS-pointer */ ) = 0;
    
    
    /** Automatic Differentiation in symmetric mode on the symbolic \n
     *  level. This function generates an expression for a          \n
     *  second order derivative.                                    \n
     *  \return SUCCESSFUL_RETURN                                   \n
     */
     virtual returnValue AD_symmetric( SharedOperator     &l  ,
                                       SharedOperatorMap  &ldf,
                                       SharedOperatorMap  &df ,
                                       SharedOperatorMap2 &H  ,
                                       SharedOperatorMap2 &LIS,
                                       SharedOperatorMap2 &SIS,
                                       SharedOperatorMap3 &HIS  ) = 0;



    /** Substitutes key with the expression sub. \n
     *  \return The substituted expression.      \n
     *
     */
     virtual SharedOperator substitute( SharedOperatorMap &sub /**< the substitution */ ) = 0;


    /** Checks whether the expression is zero or one              \n
     *  \return NE_ZERO                                           \n
     *          NE_ONE                                            \n
     *          NE_NEITHER_ONE_NOR_ZERO                           \n
     *
     */
     virtual NeutralElement isOneOrZero() const = 0;


    /** Checks whether the expression is smooth in time           \n
     *  \return BT_FALSE if the expression is not smooth          \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isSmooth( ) const;


    /** Prints the expression into a stream. \n
     *  \return SUCCESFUL_RETURN             \n
     */
     virtual std::ostream& print( std::ostream &stream, StringMap &name ) const = 0;


     virtual returnValue getArgumentList( DependencyMap &exists,
                                          SharedOperatorVector &list ) = 0;


     /** Return the value of the constant */
     virtual double getValue() const;


    /** Asks whether all elements are purely symbolic.                \n
      *                                                               \n
      * \return BT_TRUE  if the complete tree is symbolic.            \n
      *         BT_FALSE otherwise (e.g. if C functions are linked).  \n
      */
    virtual BooleanType isSymbolic() const = 0;

//
//  PROTECTED FUNCTIONS:
//

protected:


};


CLOSE_NAMESPACE_ACADO

#endif

