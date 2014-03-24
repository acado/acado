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
*    \file src/symbolic_operator/tree_projection.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>


BEGIN_NAMESPACE_ACADO


TreeProjection::TreeProjection():Projection(){ ne = NE_ZERO; }

TreeProjection::TreeProjection( const TreeProjection &arg )
               :Projection(arg){

    argument = arg.argument;
    ne = arg.ne;
}

TreeProjection::~TreeProjection(){ }

returnValue TreeProjection::setArgument( const SharedOperator & arg ){
  
     argument = arg;
     ne = argument->isOneOrZero();
     return SUCCESSFUL_RETURN;
}

returnValue TreeProjection::evaluate( EvaluationBase *x ){

    x->project(argument.get());    
    return SUCCESSFUL_RETURN;
}


Operator& TreeProjection::operator+=( const ScalarExpression& arg ){

    setArgument( myAdd( SharedOperator(new TreeProjection(*this)), arg.element ) );
    return *this;
}

Operator& TreeProjection::operator-=( const ScalarExpression& arg ){
  
    setArgument( mySubtract( SharedOperator(new TreeProjection(*this)), arg.element ) );
    return *this;
}

Operator& TreeProjection::operator*=( const ScalarExpression& arg ){
  
    setArgument( myProd( SharedOperator(new TreeProjection(*this)), arg.element ) );
    return *this;
}

Operator& TreeProjection::operator/=( const ScalarExpression& arg ){

    setArgument( myQuotient( SharedOperator(new TreeProjection(*this)), arg.element ) );
    return *this;
}


SharedOperator TreeProjection::AD_forward( SharedOperatorMap &seed ){
    
    SharedOperator tmp = seed[argument.get()];
    
    if( tmp != 0 ) return tmp;
    
    tmp = Operator::convert2TreeProjection(argument->AD_forward(seed));
    seed[argument.get()] = tmp;
    
    return tmp;
}


void TreeProjection::convert2TreeProjection(SharedOperatorMap &arg) const{

    SharedOperatorMap::iterator it;
    for( it=arg.begin(); it!=arg.end(); ++it )
         arg[it->first] = Operator::convert2TreeProjection(it->second);
}



returnValue TreeProjection::AD_backward( SharedOperator     &seed ,
                                         SharedOperatorMap  &df   ,
                                         SharedOperatorMap2 &IS     ){

    SharedOperatorMap tmp = IS[this];
    SharedOperatorMap::iterator it;

    if( tmp.size() == 0 ){
        SharedOperator aux = SharedOperator( new DoubleConstant(1.0,NE_ONE) );
        argument->AD_backward(aux,tmp,IS);
        convert2TreeProjection(tmp);
        IS[this] = tmp;
    }

    for( it=tmp.begin(); it!=tmp.end(); ++it )
         df[it->first] = myAdd( myProd( seed, it->second ), checkForZero(df[it->first]) );

    return SUCCESSFUL_RETURN;
}



returnValue TreeProjection::AD_symmetric( SharedOperator     &l  ,
                                          SharedOperatorMap  &ldf,
                                          SharedOperatorMap  &df ,
                                          SharedOperatorMap2 &H  ,
                                          SharedOperatorMap2 &LIS,
                                          SharedOperatorMap2 &SIS,
                                          SharedOperatorMap3 &HIS  ){

    SharedOperatorMap  tmpLIS = LIS[this];
    SharedOperatorMap  tmpSIS = SIS[this];
    SharedOperatorMap2 tmpHIS = HIS[this];
    
    SharedOperatorMap ::iterator it1;
    SharedOperatorMap2::iterator it2;
    
    
  // ============================================================================

    if( tmpLIS.size() == 0 ){

        SharedOperator aux = SharedOperator( new DoubleConstant(1.0,NE_ONE) );

        argument->AD_symmetric( aux, tmpLIS, tmpSIS, tmpHIS, LIS, SIS, HIS );

        convert2TreeProjection(tmpLIS);
        convert2TreeProjection(tmpSIS);

        for(it2=tmpHIS.begin();it2!=tmpHIS.end();++it2)
            convert2TreeProjection(tmpHIS[it2->first]);

        LIS[this] = tmpLIS;
        SIS[this] = tmpSIS;
        HIS[this] = tmpHIS;
    }

  // ============================================================================
  
    for( it1=tmpLIS.begin(); it1!=tmpLIS.end(); ++it1 )
         ldf[it1->first] = myAdd( myProd( l, it1->second ), checkForZero(ldf[it1->first]) );

    df = tmpSIS;

    for( it2=tmpHIS.begin(); it2!=tmpHIS.end(); ++it2 )
        for( it1=(it2->second).begin(); it1!=(it2->second).end(); ++it1 )      
             H[it2->first][it1->first] = myProd(it1->second,l);

    return SUCCESSFUL_RETURN;
} 

 
returnValue TreeProjection::getArgumentList( DependencyMap &exists,
                                             SharedOperatorVector &list  ){

    if( exists[this] != true ){
         argument->getArgumentList(exists,list);
         list.push_back(argument);
         exists[this] = true;
    }
    return SUCCESSFUL_RETURN;
}


NeutralElement TreeProjection::isOneOrZero() const{

    return ne;
}

returnValue TreeProjection::initDerivative() {

     return argument->initDerivative();
}


std::ostream& TreeProjection::print(std::ostream& stream, StringMap &name ) const{
  
    return stream << name[argument.get()];
}


CLOSE_NAMESPACE_ACADO

// end of file.
