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
 *    \file include/acado/symbolic_operator/tree_projection.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 */


#ifndef ACADO_TOOLKIT_TREE_PROJECTION_HPP
#define ACADO_TOOLKIT_TREE_PROJECTION_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


class Expression;
class ConstraintComponent;


/**
 *	\brief Implements the tree-projection operator within the family of SymbolicOperators.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class TreeProjection implements a tree projection within the 
 *	family of SymbolicOperators.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */
class TreeProjection : public Projection{

public:

    /** Default constructor */
    TreeProjection();

    /** Copy constructor. */
    TreeProjection( const TreeProjection &arg );

    /** Default destructor. */
    virtual ~TreeProjection();

    /** Sets the argument (note that arg should have dimension 1).*/
    virtual returnValue setArgument( const SharedOperator & arg );

    virtual Operator& operator+=( const ScalarExpression  & arg );
    virtual Operator& operator-=( const ScalarExpression  & arg );
    virtual Operator& operator*=( const ScalarExpression  & arg );
    virtual Operator& operator/=( const ScalarExpression  & arg );


    /** Evaluates the expression (templated version) */
    virtual returnValue evaluate( EvaluationBase *x );
    

    /** Automatic Differentiation in forward mode on the symbolic \n
     *  level. This function generates an expression for a        \n
     *  forward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                 \n
     */
     virtual SharedOperator AD_forward( SharedOperatorMap &seed /**< the forward seed   */ );



    /** Automatic Differentiation in backward mode on the symbolic \n
     *  level. This function generates an expression for a         \n
     *  backward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                  \n
     */
    virtual returnValue AD_backward( SharedOperator       &seed, /**< the backward seed  */
                                     SharedOperatorMap    &df  , /**< the result         */
                                     SharedOperatorMap2   &IS    /**< the new IS-pointer */ );
    
    
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
                                       SharedOperatorMap3 &HIS  );
     
    
     virtual returnValue getArgumentList( DependencyMap &exists,
                                          SharedOperatorVector &list  );


    /** Checks whether the expression is zero or one              \n
     *  \return NE_ZERO                                           \n
     *          NE_ONE                                            \n
     *          NE_NEITHER_ONE_NOR_ZERO                           \n
     *
     */
     virtual NeutralElement isOneOrZero() const;


     virtual returnValue initDerivative();
       
     virtual std::ostream& print(std::ostream& stream, StringMap &name ) const;
     
     
     void convert2TreeProjection(SharedOperatorMap &arg) const;
     
    //
    //  PROTECTED MEMBERS:
    //
    protected:

        SharedOperator argument;
        NeutralElement    ne;
};

CLOSE_NAMESPACE_ACADO

#endif
