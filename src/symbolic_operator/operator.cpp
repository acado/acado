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
*    \file src/symbolic_operator/operator.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/symbolic_expression/constraint_component.hpp>


BEGIN_NAMESPACE_ACADO


//TreeProjection emptyTreeProjection;


Operator::Operator(){ }

Operator::~Operator(){ }


int Operator::getGlobalIndex( ) const{

	ACADOERROR( RET_UNKNOWN_BUG );
    return -1;
}


Operator& Operator::operator= ( const double         & arg ){ return *this; }
Operator& Operator::operator= ( const Expression     & arg ){ return *this; }
Operator& Operator::operator= ( const SharedOperator & arg ){ return *this; }
Operator& Operator::operator+=( const Expression     & arg ){ return *this; }
Operator& Operator::operator-=( const Expression     & arg ){ return *this; }
Operator& Operator::operator*=( const Expression     & arg ){ return *this; }
Operator& Operator::operator/=( const Expression     & arg ){ return *this; }


double Operator::getValue() const{ return INFTY; }

std::ostream& operator<<(std::ostream &stream, const Operator &arg)
{
	return arg.print( stream );
}

SharedOperator Operator::passArgument() const{

    Operator *tmp = 0;
    return SharedOperator(tmp);
}

returnValue Operator::setVariableExportName(	const VariableType &_type,
												const std::vector< std::string >& _name
												)
{
	return SUCCESSFUL_RETURN;
}



SharedOperator Operator::myProd(const SharedOperator &a, const SharedOperator &b) const{

    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    if( b->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
    
    if( a->isOneOrZero() == NE_ONE  ) return b;
    if( b->isOneOrZero() == NE_ONE  ) return a;
    
    if( a == b ) return SharedOperator( new Power_Int(a,2) );
    
    return SharedOperator( new Product(a,b) ); 
}
  

SharedOperator Operator::myAdd( const SharedOperator &a, const SharedOperator &b) const{

    if( a->isOneOrZero() == NE_ZERO && b->isOneOrZero() == NE_ZERO )
        return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    if( a->isOneOrZero() == NE_ZERO ) return b;
    if( b->isOneOrZero() == NE_ZERO ) return a;
    
    if( a->isOneOrZero() == NE_ONE && b->isOneOrZero() == NE_ONE )
        return SharedOperator( new DoubleConstant( 2.0 , NE_NEITHER_ONE_NOR_ZERO ) );
    
    if( a == b ) return SharedOperator( new Product( a, SharedOperator( new DoubleConstant( 2.0 , NE_NEITHER_ONE_NOR_ZERO) ) ) );
    
    return SharedOperator( new Addition(a,b) );
}


SharedOperator Operator::mySubtract (const SharedOperator &a,const SharedOperator &b) const{

    if( a->isOneOrZero() == NE_ZERO && b->isOneOrZero() == NE_ZERO )
        return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    if( a->isOneOrZero() == NE_ZERO ) return SharedOperator( new Subtraction(SharedOperator( new DoubleConstant( 0.0 , NE_ZERO )), b));
    if( b->isOneOrZero() == NE_ZERO ) return a;

    if( a->isOneOrZero() == NE_ONE && b->isOneOrZero() == NE_ONE )
        return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    if( a == b ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    return SharedOperator( new Subtraction(a,b) );
}


SharedOperator Operator::myPower (const SharedOperator &a, const SharedOperator &b) const{

    if( a->isOneOrZero() == NE_ZERO && b->isOneOrZero() == NE_ONE ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    if( b->isOneOrZero() == NE_ZERO ) return SharedOperator( new DoubleConstant( 1.0 , NE_ONE ) );
    if( a->isOneOrZero() == NE_ONE ) return SharedOperator( new DoubleConstant( 1.0 , NE_ONE ) );

    if( b->isOneOrZero() == NE_ONE ) return a;

    return SharedOperator( new Power(a,b) );
}


SharedOperator Operator::myPowerInt (const SharedOperator &a, const int &b) const{

	if( a->isOneOrZero() == NE_ZERO && b>0 ) 	return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );
	if( b == 0 ) 					return SharedOperator( new DoubleConstant( 1.0 , NE_ONE  ) );
	if( a->isOneOrZero() == NE_ONE ) 		return SharedOperator( new DoubleConstant( 1.0 , NE_ONE  ) );
	if( b == 1 ) 					return a;

    return SharedOperator( new Power_Int(a,b) );
}


SharedOperator Operator::myLogarithm (const SharedOperator &a) const{

    if( a->isOneOrZero() == NE_ONE ) return SharedOperator( new DoubleConstant( 0.0 , NE_ZERO ) );

    return SharedOperator( new Logarithm(a) );
}


SharedOperator Operator::convert2TreeProjection( const SharedOperator &a ) const{
  
   TreeProjection b;
   b = a;
   return SharedOperator( new TreeProjection(b) );
}


returnValue Operator::ADsymCommon( const SharedOperator       &a  ,
                                   const SharedOperator &da ,
                                   const SharedOperator &dda,
                                   int            dim       ,
                                   VariableType  *varType   ,
                                   int           *component ,
                                   SharedOperator        &l ,
                                   SharedOperator        *S ,
                                   int                 dimS ,
                                   SharedOperator      *dfS ,
                                   SharedOperator      *ldf ,
                                   SharedOperator      *H   ,
                                   std::vector<SharedOperator> &newLIS , /**< the new LIS-pointer   */
                                   std::vector<SharedOperator> &newSIS , /**< the new SIS-pointer   */
                                   std::vector<SharedOperator> &newHIS   /**< the new HIS-pointer   */ ){

  // FIRST ORDER BACKWARD SWEEP:
  // ---------------------------
    SharedOperator ttt = convert2TreeProjection(myProd(l,da));
    
    a->AD_symmetric( dim, varType, component, ttt,
                    S, dimS, dfS, ldf, H, newLIS, newSIS, newHIS 
              );
    
    
  // SECOND ORDER FORWARD SWEEP:
  // -------------------------------------
    SharedOperator tmp3 = convert2TreeProjection(myProd(dda,l));

    int run1, run2;
    for( run1 = 0; run1 < dimS; run1++ ){
    	for( run2 = 0; run2 <= run1; run2++ ){
    		SharedOperator tmp1 = H[run1*dimS+run2];
    		SharedOperator tmp2 = myProd( dfS[run1], dfS[run2] );
    		SharedOperator tmp4 = myProd( tmp2     , tmp3      );
    		H[run1*dimS+run2] = myAdd( tmp1, tmp4 );
    	}
    }
    
    
  // FIRST ORDER FORWARD SWEEP:
  // -------------------------------------
    
    for( run1 = 0; run1 < dimS; run1++ ){
        SharedOperator tmp1 = dfS[run1];
        dfS[run1] = convert2TreeProjection( myProd( tmp1,da ) );
    }
    
    return SUCCESSFUL_RETURN;
}


returnValue Operator::ADsymCommon2( const SharedOperator &a  ,
				     const SharedOperator &b  ,
				   	   	   	   	   	  const  SharedOperator &dx ,
				   	   	   	   	   	  const  SharedOperator &dy ,
				   	   	   	   	   	  const  SharedOperator &dxx,
				   	   	   	   	   	  const  SharedOperator &dxy,
				   	   	   	   	   	  const  SharedOperator &dyy,
                                       int            dim       , /**< number of directions  */
                                       VariableType  *varType   , /**< the variable types    */
                                       int           *component , /**< and their components  */
                                       SharedOperator     &l         , /**< the backward seed     */
                                       SharedOperator     *S         , /**< forward seed matrix   */
                                       int            dimS      , /**< dimension of forward seed             */
                                       SharedOperator     *dfS       , /**< first order foward result             */
                                       SharedOperator     *ldf       , /**< first order backward result           */
                                       SharedOperator     *H         , /**< upper trianglular part of the Hessian */
                                       std::vector<SharedOperator> &newLIS , /**< the new LIS-pointer   */
                                       std::vector<SharedOperator> &newSIS , /**< the new SIS-pointer   */
                                       std::vector<SharedOperator> &newHIS   /**< the new HIS-pointer   */ ){
  
    int run1, run2;
  
  // FIRST ORDER BACKWARD SWEEP:
  // ---------------------------
    SharedOperator    *S1 = new SharedOperator[dimS];
    SharedOperator    *S2 = new SharedOperator[dimS];
    SharedOperator    *H1 = new SharedOperator[dimS*dimS];
    SharedOperator    *H2 = new SharedOperator[dimS*dimS];
  
    for( run2 = 0; run2 < dimS; run2++ ){
         S1[run2] = SharedOperator( new DoubleConstant(0.0,NE_ZERO));
         S2[run2] = SharedOperator( new DoubleConstant(0.0,NE_ZERO));
    }
    
    for( run2 = 0; run2 < dimS*dimS; run2++ ){
         H1[run2] = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
         H2[run2] = SharedOperator( new DoubleConstant(0.0,NE_ZERO) );
    }
    
    SharedOperator ttt1 = convert2TreeProjection(myProd(l,dx));
    SharedOperator ttt2 = convert2TreeProjection(myProd(l,dy));

    a->AD_symmetric( dim, varType, component, ttt1,
                    S, dimS, S1, ldf, H1, newLIS, newSIS, newHIS );


    b->AD_symmetric( dim, varType, component, ttt2,
                    S, dimS, S2, ldf, H2, newLIS, newSIS, newHIS );
    
   
    
  // SECOND ORDER FORWARD SWEEP:
  // -------------------------------------
    
    SharedOperator tmpXX = convert2TreeProjection(myProd( l,dxx ));
    SharedOperator tmpXY = convert2TreeProjection(myProd( l,dxy ));
    SharedOperator tmpYY = convert2TreeProjection(myProd( l,dyy ));
    
    for( run1 = 0; run1 < dimS; run1++ ){
    	for( run2 = 0; run2 <= run1; run2++ ){
    		SharedOperator tmp1 = myProd( S1[run1], S1[run2] );
    		SharedOperator tmp2 = myProd( S1[run1], S2[run2] );
    		SharedOperator tmp3 = myProd( S2[run1], S1[run2] );
    		SharedOperator tmp4 = myProd( S2[run1], S2[run2] );
    		SharedOperator tmp5;
    		if( run1 == run2 ) tmp5 = myAdd(tmp2,tmp2);
    		else               tmp5 = myAdd(tmp2,tmp3);
    		SharedOperator tmp6 = myProd( tmp1, tmpXX );
    		SharedOperator tmp7 = myProd( tmp5, tmpXY );
    		SharedOperator tmp8 = myProd( tmp4, tmpYY );
    		SharedOperator tmp9  = myAdd ( tmp6, tmp7 );
    		SharedOperator tmp10 = myAdd ( tmp8, tmp9 );
    		SharedOperator tmp12 = myAdd ( tmp10 , H1[run1*dimS+run2] );
    		H[run1*dimS+run2] = myAdd ( tmp12 , H2[run1*dimS+run2] );
    	}
    }    
    
  // FIRST ORDER FORWARD SWEEP:
  // -------------------------------------
    
    for( run1 = 0; run1 < dimS; run1++ ){
    	SharedOperator tmp1 = convert2TreeProjection( myProd( S1[run1], dx ) );
    	SharedOperator tmp2 = convert2TreeProjection( myProd( S2[run1], dy ) );
    	dfS[run1] = myAdd(tmp1,tmp2);
    }
    
  // CLEAR MEMORY FROM SWEEPS:
  // -------------------------------------
    delete[] S1;
    delete[] S2;
    delete[] H1;
    delete[] H2;

    return SUCCESSFUL_RETURN;
}


BooleanType Operator::isTrivial() const {
	return BT_FALSE;
}


returnValue Operator::initDerivative() {

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO


// end of file.
