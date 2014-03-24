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
*    \file include/acado/symbolic_operator/exp.hpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#ifndef ACADO_TOOLKIT_EXP_HPP
#define ACADO_TOOLKIT_EXP_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


/**
 *	\brief Implements the scalar exponential operator within the symbolic operators family.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Exp implements the scalar exponential operator within the 
 *	symbolic operators family.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */
class Exp : public UnaryOperator{

public:

    /** Default constructor. */
    Exp();

    /** Default constructor. */
    Exp( const SharedOperator &_argument );

    /** Copy constructor. */
    Exp( const Exp &arg );

    /** Default destructor. */
    ~Exp();

    /** Evaluates the expression (templated version) */
    virtual returnValue evaluate( EvaluationBase *x );

    /** Substitutes key with the expression sub. \n
     *  \return The substituted expression.      \n
     *
     */
    virtual SharedOperator substitute( SharedOperatorMap &sub /**< the substitution */ );

    /** Initializes the derivative operators */
    virtual returnValue initDerivative();
};


CLOSE_NAMESPACE_ACADO



#endif
