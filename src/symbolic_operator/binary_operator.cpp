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
 *    \file   src/symbolic_operator/binary_operator.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date   2010
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>
#include <set>


BEGIN_NAMESPACE_ACADO


BinaryOperator::BinaryOperator():SmoothOperator(){}

BinaryOperator::BinaryOperator( const SharedOperator &_argument1, const SharedOperator &_argument2 ):SmoothOperator(){

    a1 = _argument1;  a2 = _argument2;
}


BinaryOperator::BinaryOperator( const BinaryOperator &arg ){

    a1  = arg.a1;   a2  = arg.a2;
    d1  = arg.d1;   d2  = arg.d2;
    
    d11  = arg.d11; d12  = arg.d12; d22  = arg.d22;
}

BinaryOperator::~BinaryOperator(){}


SharedOperator BinaryOperator::AD_forward( SharedOperatorMap &seed ){

    return myAdd( myProd( d1, a1->AD_forward(seed) ),
                  myProd( d2, a2->AD_forward(seed) ) );
}


returnValue BinaryOperator::AD_backward( SharedOperator     &seed,
                                         SharedOperatorMap  &df  ,
                                         SharedOperatorMap2 &IS   ){

    SharedOperator tmp = convert2TreeProjection(seed);
    SharedOperator seed1 = myProd(d1,tmp);
    SharedOperator seed2 = myProd(d2,tmp);
    
    a1->AD_backward( seed1, df, IS );
    a2->AD_backward( seed2, df, IS );

    return SUCCESSFUL_RETURN;  
}


returnValue BinaryOperator::AD_symmetric( SharedOperator     &l  ,
                                          SharedOperatorMap  &ldf,
                                          SharedOperatorMap  &df ,
                                          SharedOperatorMap2 &H  ,
                                          SharedOperatorMap2 &LIS,
                                          SharedOperatorMap2 &SIS,
                                          SharedOperatorMap3 &HIS  ){
    
  // FIRST ORDER BACKWARD SWEEP:
  // ---------------------------
    SharedOperatorMap S1;
    SharedOperatorMap S2;
    
    SharedOperatorMap2 H1;
    SharedOperatorMap2 H2;
    
    SharedOperator ttt1 = convert2TreeProjection(myProd(l,d1));
    SharedOperator ttt2 = convert2TreeProjection(myProd(l,d2));

    a1->AD_symmetric( ttt1, ldf, S1, H1, LIS, SIS, HIS );
    a2->AD_symmetric( ttt2, ldf, S2, H2, LIS, SIS, HIS );
    
    
  // DETERMINE THE UNION OF ALL DEPENDENCIES:
  // ------------------------------------------------
    
    std::set<Operator*> dep;
    std::set<Operator*>::iterator run1, run2;
    SharedOperatorMap::iterator it;
    
    for( it = S1.begin(); it != S1.end(); ++it ) dep.insert(it->first);
    for( it = S2.begin(); it != S2.end(); ++it ) dep.insert(it->first);
    
    
  // SECOND ORDER FORWARD SWEEP:
  // -------------------------------------
    
    SharedOperator tmpXX = convert2TreeProjection(myProd( l,d11 ));
    SharedOperator tmpXY = convert2TreeProjection(myProd( l,d12 ));
    SharedOperator tmpYY = convert2TreeProjection(myProd( l,d22 ));
    
    for( run1 = dep.begin(); run1 != dep.end(); ++run1 ){
    	for( run2 = dep.begin(); run2 != dep.end(); ++run2 ){
    		SharedOperator tmp1 = myProd( checkForZero(S1[*run1]), checkForZero(S1[*run2]) );
    		SharedOperator tmp2 = myProd( checkForZero(S1[*run1]), checkForZero(S2[*run2]) );
    		SharedOperator tmp3 = myProd( checkForZero(S2[*run1]), checkForZero(S1[*run2]) );
    		SharedOperator tmp4 = myProd( checkForZero(S2[*run1]), checkForZero(S2[*run2]) );
    		SharedOperator tmp5;
    		if( run1 == run2 ) tmp5 = myAdd(tmp2,tmp2);
    		else               tmp5 = myAdd(tmp2,tmp3);
    		SharedOperator tmp6 = myProd( tmp1, tmpXX );
    		SharedOperator tmp7 = myProd( tmp5, tmpXY );
    		SharedOperator tmp8 = myProd( tmp4, tmpYY );
    		SharedOperator tmp9  = myAdd ( tmp6, tmp7 );
    		SharedOperator tmp10 = myAdd ( tmp8, tmp9 );
    		SharedOperator tmp12 = myAdd ( tmp10 , checkForZero(H1[*run1][*run2]) );
    		H[*run1][*run2] = myAdd ( tmp12 , checkForZero(H2[*run1][*run2]) );
    	}
    }
    
  // FIRST ORDER FORWARD SWEEP:
  // -------------------------------------
    
    for( run1 = dep.begin(); run1 != dep.end(); ++run1 ){
    	SharedOperator tmp1 = convert2TreeProjection( myProd( checkForZero(S1[*run1]), d1 ) );
    	SharedOperator tmp2 = convert2TreeProjection( myProd( checkForZero(S2[*run1]), d2 ) );
    	df[*run1] = myAdd(tmp1,tmp2);
    }

    return SUCCESSFUL_RETURN;
}

NeutralElement BinaryOperator::isOneOrZero() const{ return NE_NEITHER_ONE_NOR_ZERO; }

returnValue BinaryOperator::getArgumentList( DependencyMap &exists,
                                             SharedOperatorVector &list  ){

    a1->getArgumentList( exists, list );
    a2->getArgumentList( exists, list );
    return SUCCESSFUL_RETURN;
}

BooleanType BinaryOperator::isSymbolic() const{

    if( a1->isSymbolic() == BT_FALSE ) return BT_FALSE;
    if( a2->isSymbolic() == BT_FALSE ) return BT_FALSE;

    return BT_TRUE;
}

returnValue BinaryOperator::initDerivative() {

    a1->initDerivative();
    a2->initDerivative();
    return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

// end of file.
