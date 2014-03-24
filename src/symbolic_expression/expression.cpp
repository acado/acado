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
 *    \file src/symbolic_expression/expression.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/symbolic_expression/expression.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/symbolic_expression/acado_syntax.hpp>
#include <acado/function/function.hpp>

BEGIN_NAMESPACE_ACADO


// ---------------------------------------------------------------------------------------------------
//                                 IMPLEMENTATION OF THE CONSTRUCTORS:
// ---------------------------------------------------------------------------------------------------

ScalarExpression::ScalarExpression(){ element = SharedOperator( new Projection() ); }

ScalarExpression::ScalarExpression( const double &element_ ){
  
   element = SharedOperator( new DoubleConstant(element_));
}

ScalarExpression::ScalarExpression( const SharedOperator &element_ ){ element = element_; }

ScalarExpression::ScalarExpression( const ScalarExpression &rhs ){

   element = rhs.element;
}

ScalarExpression::~ScalarExpression(){}

ScalarExpression& ScalarExpression::operator=( const ScalarExpression& arg ){
  
    if( &arg != this ){
       element = arg.element;
    }
    return *this;
}

ScalarExpression& ScalarExpression::operator+=( const ScalarExpression& arg ){ return operator=(element->myAdd     (element,arg.element));}
ScalarExpression& ScalarExpression::operator-=( const ScalarExpression& arg ){ return operator=(element->mySubtract(element,arg.element));}
ScalarExpression& ScalarExpression::operator*=( const ScalarExpression& arg ){ return operator=(element->myProd    (element,arg.element));}
ScalarExpression& ScalarExpression::operator/=( const ScalarExpression& arg ){ return operator=(element->myQuotient(element,arg.element));}

ScalarExpression& ScalarExpression::operator= ( const Expression& arg ){ ASSERT(arg.size() == 1); return operator= (arg(0)); }
ScalarExpression& ScalarExpression::operator+=( const Expression& arg ){ ASSERT(arg.size() == 1); return operator+=(arg(0)); }
ScalarExpression& ScalarExpression::operator-=( const Expression& arg ){ ASSERT(arg.size() == 1); return operator-=(arg(0)); }
ScalarExpression& ScalarExpression::operator*=( const Expression& arg ){ ASSERT(arg.size() == 1); return operator*=(arg(0)); }
ScalarExpression& ScalarExpression::operator/=( const Expression& arg ){ ASSERT(arg.size() == 1); return operator/=(arg(0)); }

ScalarExpression& ScalarExpression::operator= ( const double& arg ){ return operator= (ScalarExpression(arg)); }
ScalarExpression& ScalarExpression::operator+=( const double& arg ){ return operator+=(ScalarExpression(arg)); }
ScalarExpression& ScalarExpression::operator-=( const double& arg ){ return operator-=(ScalarExpression(arg)); }
ScalarExpression& ScalarExpression::operator*=( const double& arg ){ return operator*=(ScalarExpression(arg)); }
ScalarExpression& ScalarExpression::operator/=( const double& arg ){ return operator/=(ScalarExpression(arg)); }


const ScalarExpression ScalarExpression::operator-( ) const{ 
    return ScalarExpression( element->mySubtract( SharedOperator( new DoubleConstant(0.,NE_ZERO) ), element ) ); 
}

const ScalarExpression ScalarExpression::operator+( const ScalarExpression& arg ) const{  
    return ScalarExpression( element->myAdd(element,arg.element) );
}

const ScalarExpression ScalarExpression::operator-( const ScalarExpression  & arg ) const{
    return  ScalarExpression( element->mySubtract(element,arg.element) );
}

const ScalarExpression ScalarExpression::operator*( const ScalarExpression& arg ) const{  
    return ScalarExpression( element->myProd(element,arg.element) );
}

const ScalarExpression ScalarExpression::operator/( const ScalarExpression  & arg ) const{
    return  ScalarExpression( element->myQuotient(element,arg.element) );
}

const ScalarExpression operator+( const double &arg1, const ScalarExpression& arg2 ){ return ScalarExpression(arg1)+arg2; }
const ScalarExpression operator-( const double &arg1, const ScalarExpression& arg2 ){ return ScalarExpression(arg1)-arg2; }
const ScalarExpression operator*( const double &arg1, const ScalarExpression& arg2 ){ return ScalarExpression(arg1)*arg2; }
const ScalarExpression operator/( const double &arg1, const ScalarExpression& arg2 ){ return ScalarExpression(arg1)/arg2; }

bool ScalarExpression::operator==( const ScalarExpression& arg ) const{
  
    bool a = acadoIsFinite(     element->getValue() );
    bool b = acadoIsFinite( arg.element->getValue() );
  
    if ( !a || !b ) return element == arg.element;
    
    return acadoIsEqual( element->getValue(), arg.element->getValue());
}

bool ScalarExpression::operator!=( const ScalarExpression& arg ) const{ return !(element == arg.element); }

bool ScalarExpression::operator<=( const ScalarExpression& arg ) const{
  
    bool a = acadoIsFinite(     element->getValue() );
    bool b = acadoIsFinite( arg.element->getValue() );
  
    if ( !a && !b ) return element.get() <= arg.element.get();
    if ( !a &&  b ) return false;
    if (  a && !b ) return true;
    
    return acadoIsSmaller( element->getValue() , arg.element->getValue() );
}

bool ScalarExpression::operator< ( const ScalarExpression& arg ) const{

    bool a = acadoIsFinite(     element->getValue() );
    bool b = acadoIsFinite( arg.element->getValue() );
  
    if ( !a && !b ) return element.get() < arg.element.get();
    if ( !a &&  b ) return false;
    if (  a && !b ) return true;

    return acadoIsStrictlySmaller( element->getValue() , arg.element->getValue() );
}

bool ScalarExpression::operator>=( const ScalarExpression& arg ) const{ return !operator< (arg); }
bool ScalarExpression::operator> ( const ScalarExpression& arg ) const{ return !operator<=(arg); }


const ScalarExpression ScalarExpression::getSin    ( ) const{ return ScalarExpression( element->mySin      (element) ); }
const ScalarExpression ScalarExpression::getCos    ( ) const{ return ScalarExpression( element->myCos      (element) ); }
const ScalarExpression ScalarExpression::getTan    ( ) const{ return ScalarExpression( element->myTan      (element) ); }
const ScalarExpression ScalarExpression::getAsin   ( ) const{ return ScalarExpression( element->myAsin     (element) ); }
const ScalarExpression ScalarExpression::getAcos   ( ) const{ return ScalarExpression( element->myAcos     (element) ); }
const ScalarExpression ScalarExpression::getAtan   ( ) const{ return ScalarExpression( element->myAtan     (element) ); }
const ScalarExpression ScalarExpression::getExp    ( ) const{ return ScalarExpression( element->myExp      (element) ); }
const ScalarExpression ScalarExpression::getSqrt   ( ) const{ return ScalarExpression( element->mySqrt     (element) ); }
const ScalarExpression ScalarExpression::getLn     ( ) const{ return ScalarExpression( element->myLogarithm(element) ); }

const ScalarExpression ScalarExpression::getPow   ( const ScalarExpression &arg ) const{return ScalarExpression( element->myPower(element,arg.element) );}
const ScalarExpression ScalarExpression::getPowInt( const int        &arg ) const{return ScalarExpression( element->myPowerInt(element,arg     ) );}


SharedOperatorMap2 ScalarExpression::ADsymmetric( const ScalarExpression &l,
                                                  SharedOperatorMap &dfS, SharedOperatorMap &ldf ) const{

    SharedOperatorMap2 H  ;
    SharedOperatorMap2 LIS;
    SharedOperatorMap2 SIS;
    SharedOperatorMap3 HIS;
    
    SharedOperator seed = l.element;
    element->initDerivative();
    element->AD_symmetric( seed, ldf, dfS, H, LIS, SIS, HIS );

    return H;
}

const ScalarExpression ScalarExpression::AD_forward ( const Expression &arg, const Expression &seed ) const{
  
   ASSERT( arg.size() == seed.size() );
   SharedOperatorMap Seed;
   
   for( int i=0; i<arg.size(); ++i )
       Seed[(arg(i).element).get()] = seed(i).element;
   element->initDerivative();
   
   return ScalarExpression( element->AD_forward(Seed) );
}


SharedOperatorMap ScalarExpression::AD_backward( const ScalarExpression &seed_ ) const{
  
    SharedOperatorMap  df;
    SharedOperatorMap2 IS;
    SharedOperator seed = seed_.element;
    element->initDerivative();
    element->AD_backward( seed, df, IS );
    return df;
}


BooleanType ScalarExpression::isVariable( ) const{
 
    Projection *p = dynamic_cast<Projection *>(element.get());
    return p != 0;
}


// =====================================================================
//
//               IMPLEMENTATION OF THE EXPRESSION CLASS
//
// =====================================================================


Expression::Expression( const int &nRows, const int &nCols ) : Base(nRows,nCols){ }

Expression::Expression( const ScalarExpression &arg ) : Base(1,1){ operator()(0,0) = arg; }

Expression::Expression( const DMatrix &arg ) : Base(arg.rows(),arg.cols()){
  
  for( int i=0; i<arg.size(); ++i )operator()(i) = ScalarExpression( (double) arg(i) );
}

Expression::Expression( const double &arg ) : Base(1,1){
  
  operator()(0) = ScalarExpression(arg);
}

Expression Expression::operator,( const Expression &arg ) const{
  
    Expression result(*this);
    return result.appendRows(arg);
}

Expression& Expression::appendRows( const Expression &arg ){
  
    if (size() == 0){
        Base::operator=(arg);
        return *this;
    }
    ASSERT(Base::cols() == arg.cols());

    int oldRows = Base::rows();
    int argRows = arg.rows();

    Base::conservativeResize(oldRows+argRows, Base::cols());
    Base::block(oldRows, 0, argRows, Base::cols()) = arg;

    return *this;
}

Expression& Expression::appendCols( const Expression &arg ){

    if (size() == 0){
        Base::operator=(arg);
        return *this;
    }
    ASSERT(Base::rows() == arg.rows());

    int oldCols = Base::cols();
    int argCols = arg.cols();

    Base::conservativeResize(Base::rows(), oldCols + argCols);
    Base::block(0, oldCols, Base::rows(), argCols) = arg;

    return *this;
}

Expression Expression::AD_backward( const Expression &arg, const Expression &seed ) const{
 
    ASSERT( size() == seed.size() );
    
    Expression tmp = zeros<double>(arg.size());
    SharedOperatorMap df;
    
    for( int i=0; i<size(); ++i ){

         df = operator()(i).AD_backward(seed(i));

         for( int j=0; j<arg.size(); ++j )
              tmp(j).element = arg(0).element->myAdd( arg(0).element->checkForZero( tmp(j).element          ),
                                                      arg(0).element->checkForZero( df[arg(j).element.get()])  );
    }
    return tmp;
}

Expression Expression::AD_symmetric( const Expression &arg, const Expression &seed,
                                     Expression *dfS , Expression *ldf ) const{


    ASSERT( size() == seed.size() );
    
    Expression tmp = zeros<double>(arg.size(),arg.size());
//     SharedOperatorMap  D;
//     SharedOperatorMap  L;
//     SharedOperatorMap2 H;
//     
//     for( int i=0; i<size(); ++i ){
// 
//          H = operator()(i).AD_symmetric(seed(i),D,L);
// 
//          for( int j=0; j<arg.size(); ++j )
//              for( int k=0; k<arg.size(); ++k )
//                  tmp(j,k).element = arg(0).element->myAdd( arg(0).element->checkForZero( tmp(j).element          ),
//                                                       arg(0).element->checkForZero( df[arg(j).element.get()])  );
//     }
    return tmp;
/*				       
				       
				       SharedOperatorMap2 ScalarExpression::ADsymmetric( const ScalarExpression &l,
                                                  SharedOperatorMap &dfS, SharedOperatorMap &ldf ) const{*/

}


BooleanType Expression::isVariable( ) const{
 
    for( int i=0; i<size(); ++i ){
        if(operator()(i).isVariable() == BT_FALSE ) return BT_FALSE;
    }
    return BT_TRUE;
}

std::vector<NeutralElement> Expression::isOneOrZero() const{
 
    std::vector<NeutralElement> ne(size());
    for( int i=0; i<size(); ++i ){
        ne[i] = operator()(i).element->isOneOrZero();
    }
    return ne;
}

BooleanType Expression::isSmooth() const{
 
    for( int i=0; i<size(); ++i ){
        if( operator()(i).element->isSmooth() == BT_FALSE )
            return BT_FALSE;
    }
    return BT_TRUE;
}
  
  

CLOSE_NAMESPACE_ACADO

// end of file.
