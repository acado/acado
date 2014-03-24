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
 *    \file   src/symbolic_operator/unary_operator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>


BEGIN_NAMESPACE_ACADO


UnaryOperator::UnaryOperator():SmoothOperator(){}

UnaryOperator::UnaryOperator( const SharedOperator &_argument, const std::string &_cName ):SmoothOperator(){

    argument = _argument;
    cName    = _cName;
}

UnaryOperator::UnaryOperator( const UnaryOperator &arg ){

    argument    = arg.argument;
    cName       = arg.cName;
    derivative  = arg.derivative;
    derivative2 = arg.derivative2;
}

UnaryOperator::~UnaryOperator(){}



SharedOperator UnaryOperator::AD_forward( SharedOperatorMap &seed ){

    return myProd( derivative, argument->AD_forward(seed) );
}


returnValue UnaryOperator::AD_backward( SharedOperator     &seed,
                                        SharedOperatorMap  &df  ,
                                        SharedOperatorMap2 &IS   ){

    SharedOperator tmp = myProd(derivative,seed);
    return argument->AD_backward( tmp, df, IS );
}


returnValue UnaryOperator::AD_symmetric( SharedOperator     &l  ,
                                         SharedOperatorMap  &ldf,
                                         SharedOperatorMap  &df ,
                                         SharedOperatorMap2 &H  ,
                                         SharedOperatorMap2 &LIS,
                                         SharedOperatorMap2 &SIS,
                                         SharedOperatorMap3 &HIS  ){
  
    // FIRST ORDER BACKWARD SWEEP:
    // ---------------------------
    SharedOperator ttt = convert2TreeProjection(myProd(l,derivative));
    
    argument->AD_symmetric( ttt, ldf, df, H, LIS, SIS, HIS );
    
    
  // SECOND ORDER FORWARD SWEEP:
  // -------------------------------------
    SharedOperator tmp3 = convert2TreeProjection(myProd(derivative2,l));

    SharedOperatorMap::iterator run1, run2;
    
    for( run1 = df.begin(); run1 != df.end(); ++run1 ){
    	for( run2 = df.begin(); run2 != df.end(); ++run2 ){
    		SharedOperator tmp1 = H[run1->first][run2->first];
    		SharedOperator tmp2 = myProd( df[run1->first], df[run2->first] );
    		SharedOperator tmp4 = myProd( tmp2    , tmp3     );
    		H[run1->first][run2->first] = myAdd( tmp1, tmp4 );
    	}
    }
    
  // FIRST ORDER FORWARD SWEEP:
  // -------------------------------------
    for( run1 = df.begin(); run1 != df.end(); ++run1 )
        df[run1->first] = convert2TreeProjection( myProd(derivative,df[run1->first]) );
    
    return SUCCESSFUL_RETURN;
}

NeutralElement UnaryOperator::isOneOrZero() const{ return NE_NEITHER_ONE_NOR_ZERO; }

std::ostream& UnaryOperator::print( std::ostream &stream, StringMap &name ) const{

    stream << "(" << cName << "(";
    argument->print(stream,name);
    return stream << "))";
}

returnValue UnaryOperator::getArgumentList( DependencyMap &exists,
                                            SharedOperatorVector &list  ){

    return argument->getArgumentList(exists,list);
}


BooleanType UnaryOperator::isSymbolic() const{

    if( argument->isSymbolic() == BT_FALSE ) return BT_FALSE;
    return BT_TRUE;
}

CLOSE_NAMESPACE_ACADO

// end of file.
