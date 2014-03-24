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
 *    \file src/symbolic_operator/projection.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>



BEGIN_NAMESPACE_ACADO


Projection::Projection():SmoothOperator( ){ }

Projection::~Projection(){ }

Projection::Projection( const Projection& arg ){ }


returnValue Projection::evaluate( EvaluationBase *x ){

    x->project(this);
    return SUCCESSFUL_RETURN;
}

SharedOperator Projection::AD_forward( SharedOperatorMap &seed ){

    return checkForZero(seed[this]);
}



returnValue Projection::AD_backward( SharedOperator     &seed ,
                                     SharedOperatorMap  &df   ,
                                     SharedOperatorMap2 &IS     ){

    df[this] = myAdd( checkForZero(df[this]), seed );
    return SUCCESSFUL_RETURN;
}


returnValue Projection::AD_symmetric( SharedOperator     &l  ,
                                      SharedOperatorMap  &ldf,
                                      SharedOperatorMap  &df ,
                                      SharedOperatorMap2 &H  ,
                                      SharedOperatorMap2 &LIS,
                                      SharedOperatorMap2 &SIS,
                                      SharedOperatorMap3 &HIS  ){

    df [this] = SharedOperator( new DoubleConstant(1.0,NE_ONE) );
    ldf[this] = myAdd( checkForZero(ldf[this]), l );
    return SUCCESSFUL_RETURN;
}


SharedOperator Projection::substitute( SharedOperatorMap &sub ){

    return checkForZero(sub[this]);
}

NeutralElement Projection::isOneOrZero() const{  return NE_NEITHER_ONE_NOR_ZERO; }


std::ostream& Projection::print( std::ostream &stream, StringMap &name ) const{

    return stream << name[this];
}

returnValue Projection::getArgumentList( DependencyMap &exists,
                                         SharedOperatorVector &list  ){ return SUCCESSFUL_RETURN; }

BooleanType Projection::isSymbolic() const{ return BT_TRUE; }


CLOSE_NAMESPACE_ACADO

// end of file.
