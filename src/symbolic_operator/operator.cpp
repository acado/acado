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


Operator::Operator(){

    nCount = 0;
}

Operator::~Operator(){ }


TreeProjection& Operator::operator=( const double &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

TreeProjection& Operator::operator=( const Vector &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

TreeProjection& Operator::operator=( const Matrix &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

TreeProjection& Operator::operator=( const Expression &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

TreeProjection& Operator::operator=( const Operator &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}


TreeProjection* Operator::cloneTreeProjection() const{

    // if this routine is ever called something went
    // really wrong ....

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return new TreeProjection();
}


int Operator::getGlobalIndex( ) const{

	ACADOERROR( RET_UNKNOWN_BUG );
    return -1;
}



TreeProjection& Operator::operator+=( const double    & arg ){ return operator=( this->operator+(arg) ); }
TreeProjection& Operator::operator+=( const Vector    & arg ){ return operator=( this->operator+(arg) ); }
TreeProjection& Operator::operator+=( const Matrix    & arg ){ return operator=( this->operator+(arg) ); }
TreeProjection& Operator::operator+=( const Expression& arg ){ return operator=( this->operator+(arg) ); }

TreeProjection& Operator::operator-=( const double      & arg ){ return operator=( this->operator-(arg) ); }
TreeProjection& Operator::operator-=( const Vector      & arg ){ return operator=( this->operator-(arg) ); }
TreeProjection& Operator::operator-=( const Matrix      & arg ){ return operator=( this->operator-(arg) ); }
TreeProjection& Operator::operator-=( const Expression  & arg ){ return operator=( this->operator-(arg) ); }

TreeProjection& Operator::operator*=( const double      & arg ){ return operator=( this->operator*(arg) ); }
TreeProjection& Operator::operator*=( const Vector      & arg ){ return operator=( this->operator*(arg) ); }
TreeProjection& Operator::operator*=( const Matrix      & arg ){ return operator=( this->operator*(arg) ); }
TreeProjection& Operator::operator*=( const Expression  & arg ){ return operator=( this->operator*(arg) ); }

TreeProjection& Operator::operator/=( const double      & arg ){ return operator=( this->operator/(arg) ); }
TreeProjection& Operator::operator/=( const Expression  & arg ){ return operator=( this->operator/(arg) ); }




Expression Operator::operator+( const double        & arg ) const{ return Expression(*this)+arg; }
Expression Operator::operator+( const Vector        & arg ) const{ return Expression(*this)+arg; }
Expression Operator::operator+( const Matrix        & arg ) const{ return Expression(*this)+arg; }
Expression Operator::operator+( const Operator& arg ) const{ return Expression(*this)+arg; }
Expression Operator::operator+( const Expression    & arg ) const{ return Expression(*this)+arg; }

Expression operator+( const double & arg1, const Operator& arg2 ){ return arg1 + Expression(arg2); }
Expression operator+( const Vector & arg1, const Operator& arg2 ){ return arg1 + Expression(arg2); }
Expression operator+( const Matrix & arg1, const Operator& arg2 ){ return arg1 + Expression(arg2); }

Expression Operator::operator-( const double        & arg ) const{ return Expression(*this)-arg; }
Expression Operator::operator-( const Vector        & arg ) const{ return Expression(*this)-arg; }
Expression Operator::operator-( const Matrix        & arg ) const{ return Expression(*this)-arg; }
Expression Operator::operator-( const Operator& arg ) const{ return Expression(*this)-arg; }
Expression Operator::operator-( const Expression    & arg ) const{ return Expression(*this)-arg; }

Expression Operator::operator-( ) const{ return -Expression(*this); }

Expression operator-( const double & arg1, const Operator& arg2 ){ return arg1 - Expression(arg2); }
Expression operator-( const Vector & arg1, const Operator& arg2 ){ return arg1 - Expression(arg2); }
Expression operator-( const Matrix & arg1, const Operator& arg2 ){ return arg1 - Expression(arg2); }

Expression Operator::operator*( const double        & arg ) const{ return Expression(*this)*arg; }
Expression Operator::operator*( const Vector        & arg ) const{ return Expression(*this)*arg; }
Expression Operator::operator*( const Matrix        & arg ) const{ return Expression(*this)*arg; }


Expression Operator::operator*( const Operator& arg ) const{

    Expression tmp2(arg);


    Expression tmp1(*this);


    return tmp1*tmp2;
}


Expression Operator::operator*( const Expression    & arg ) const{ return Expression(*this)*arg; }

Expression operator*( const double & arg1, const Operator& arg2 ){ return arg1 * Expression(arg2); }
Expression operator*( const Vector & arg1, const Operator& arg2 ){ return arg1 * Expression(arg2); }
Expression operator*( const Matrix & arg1, const Operator& arg2 ){ return arg1 * Expression(arg2); }

Expression Operator::operator/( const double        & arg ) const{ return Expression(*this)/arg; }
Expression Operator::operator/( const Operator& arg ) const{ return Expression(*this)/arg; }
Expression Operator::operator/( const Expression    & arg ) const{ return Expression(*this)/arg; }

Expression operator/( const double & arg1, const Operator& arg2 ){ return arg1 / Expression(arg2); }
Expression operator/( const Vector & arg1, const Operator& arg2 ){ return arg1 / Expression(arg2); }
Expression operator/( const Matrix & arg1, const Operator& arg2 ){ return arg1 / Expression(arg2); }

ConstraintComponent Operator::operator<=( const double& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent Operator::operator>=( const double& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent Operator::operator==( const double&  b ) const{ return Expression(*this) ==  b; }

ConstraintComponent Operator::operator<=( const Vector& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent Operator::operator>=( const Vector& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent Operator::operator==( const Vector&  b ) const{ return Expression(*this) ==  b; }

ConstraintComponent Operator::operator<=( const VariablesGrid& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent Operator::operator>=( const VariablesGrid& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent Operator::operator==( const VariablesGrid&  b ) const{ return Expression(*this) ==  b; }

ConstraintComponent operator<=( double lb, const Operator &arg ){ return lb <= Expression(arg); }
ConstraintComponent operator==( double  b, const Operator &arg ){ return  b == Expression(arg); }
ConstraintComponent operator>=( double ub, const Operator &arg ){ return ub >= Expression(arg); }

ConstraintComponent operator<=( Vector lb, const Operator &arg ){ return lb <= Expression(arg); }
ConstraintComponent operator==( Vector  b, const Operator &arg ){ return  b == Expression(arg); }
ConstraintComponent operator>=( Vector ub, const Operator &arg ){ return ub >= Expression(arg); }

ConstraintComponent operator<=( VariablesGrid lb, const Operator &arg ){ return lb <= Expression(arg); }
ConstraintComponent operator==( VariablesGrid  b, const Operator &arg ){ return  b == Expression(arg); }
ConstraintComponent operator>=( VariablesGrid ub, const Operator &arg ){ return ub >= Expression(arg); }



double Operator::getValue() const{ return INFTY; }


Stream& Operator::operator<<( Stream &stream ) const{

    return print( stream );
}


Stream& operator<<( Stream &stream, const Operator &arg ){

    return arg.print( stream );
}


returnValue operator<<( FILE* file, const Operator &arg ){

    Stream tmp;
    tmp = arg.print(tmp);
	file << tmp;
    return SUCCESSFUL_RETURN;
}


Operator* Operator::passArgument() const{

    return 0;
}

returnValue Operator::setVariableExportName( const VariableType &_type, const Stream *_name ){

	return SUCCESSFUL_RETURN;
}



Operator* Operator::myProd(Operator* a,Operator* b){
  
    if( a->isOneOrZero() == NE_ZERO ) return new DoubleConstant( 0.0 , NE_ZERO );
    if( b->isOneOrZero() == NE_ZERO ) return new DoubleConstant( 0.0 , NE_ZERO );
    
    if( a->isOneOrZero() == NE_ONE  ) return b->clone();
    if( b->isOneOrZero() == NE_ONE  ) return a->clone();
    
    if( a == b ) return new Power_Int( a->clone(), 2 );
    
    return new Product(a->clone(),b->clone()); 
}
  

Operator* Operator::myAdd (Operator* a,Operator* b){

    if( a->isOneOrZero() == NE_ZERO ) return b->clone();
    if( b->isOneOrZero() == NE_ZERO ) return a->clone();
    
    if( a->isOneOrZero() == NE_ONE && b->isOneOrZero() == NE_ONE )
        return new DoubleConstant( 2.0 , NE_NEITHER_ONE_NOR_ZERO );
    
    if( a == b ) return new Product( a->clone(), new DoubleConstant( 2.0 , NE_NEITHER_ONE_NOR_ZERO ) );
    
    return new Addition(a->clone(),b->clone());
}


returnValue Operator::ADsymCommon( Operator     *a  ,
                                        TreeProjection &da ,
                                        TreeProjection &dda,
                                        int            dim       , /**< number of directions  */
                                        VariableType  *varType   , /**< the variable types    */
                                        int           *component , /**< and their components  */
                                        Operator      *l         , /**< the backward seed     */
                                        Operator     **S         , /**< forward seed matrix   */
                                        int            dimS      , /**< dimension of forward seed             */
                                        Operator     **dfS       , /**< first order foward result             */
                                        Operator     **ldf       , /**< first order backward result           */
                                        Operator     **H         , /**< upper trianglular part of the Hessian */
                                      int            &nNewLIS  , /**< the number of newLIS  */
                                      TreeProjection ***newLIS , /**< the new LIS-pointer   */
                                      int            &nNewSIS  , /**< the number of newSIS  */
                                      TreeProjection ***newSIS , /**< the new SIS-pointer   */
                                      int            &nNewHIS  , /**< the number of newHIS  */
                                      TreeProjection ***newHIS   /**< the new HIS-pointer   */ ){

  // FIRST ORDER BACKWARD SWEEP:
  // ---------------------------
  
    a->ADsymmetric( dim, varType, component,
                    myProd( l, &da ),
                    S, dimS, dfS, ldf, H, nNewLIS, newLIS, nNewSIS, newSIS, nNewHIS, newHIS 
              );
    
    
  // SECOND ORDER FORWARD SWEEP:
  // -------------------------------------
    
    int run1, run2;
    for( run1 = 0; run1 < dimS; run1++ ){
      for( run2 = 0; run2 <= run1; run2++ ){
        Operator *tmp1 = H[run1*dimS+run2]->clone();
        delete H[run1*dimS+run2];
	Operator *tmp2 = myProd( dfS[run1], dfS[run2] );
	Operator *tmp3 = myProd( &dda     , l         );
	Operator *tmp4 = myProd( tmp2     , tmp3      );
        H[run1*dimS+run2] = myAdd( tmp1, tmp4 );
        delete tmp1;
	delete tmp2;
	delete tmp3;
	delete tmp4;
      }
    }
    
    
  // FIRST ORDER FORWARD SWEEP:
  // -------------------------------------
    
    for( run1 = 0; run1 < dimS; run1++ ){
        Operator *tmp1 = dfS[run1]->clone();
        delete dfS[run1];
        dfS[run1] = myProd( tmp1, &da );
        delete tmp1;
    }
    
  // CLEAR MEMORY FROM BACKWARD SWEEP:
  // -------------------------------------
    
    delete l;
    return SUCCESSFUL_RETURN;
}


returnValue Operator::ADsymCommon2( Operator       *a  ,
				   Operator       *b  ,
                                  TreeProjection &dx ,
                                  TreeProjection &dy ,
                                  TreeProjection &dxx,
                                  TreeProjection &dxy,
                                  TreeProjection &dyy,
                                        int            dim       , /**< number of directions  */
                                        VariableType  *varType   , /**< the variable types    */
                                        int           *component , /**< and their components  */
                                        Operator      *l         , /**< the backward seed     */
                                        Operator     **S         , /**< forward seed matrix   */
                                        int            dimS      , /**< dimension of forward seed             */
                                        Operator     **dfS       , /**< first order foward result             */
                                        Operator     **ldf       , /**< first order backward result           */
                                        Operator     **H         , /**< upper trianglular part of the Hessian */
                                      int            &nNewLIS  , /**< the number of newLIS  */
                                      TreeProjection ***newLIS , /**< the new LIS-pointer   */
                                      int            &nNewSIS  , /**< the number of newSIS  */
                                      TreeProjection ***newSIS , /**< the new SIS-pointer   */
                                      int            &nNewHIS  , /**< the number of newHIS  */
                                      TreeProjection ***newHIS   /**< the new HIS-pointer   */ ){
  
    int run1, run2;
  
  // FIRST ORDER BACKWARD SWEEP:
  // ---------------------------
    Operator    **S1 = new Operator*[dimS];
    Operator    **S2 = new Operator*[dimS];
    Operator    **H1 = new Operator*[dimS*dimS];
    Operator    **H2 = new Operator*[dimS*dimS];
  
    for( run2 = 0; run2 < dimS; run2++ ){
         S1[run2] = new DoubleConstant(0.0,NE_ZERO);
         S2[run2] = new DoubleConstant(0.0,NE_ZERO);
    }
    
    for( run2 = 0; run2 < dimS*dimS; run2++ ){
         H1[run2] = new DoubleConstant(0.0,NE_ZERO);
         H2[run2] = new DoubleConstant(0.0,NE_ZERO);
    }
    

    a->ADsymmetric( dim, varType, component, myProd( l, &dx ),
                    S, dimS, S1, ldf, H1, nNewLIS, newLIS, nNewSIS, newSIS, nNewHIS, newHIS );


    b->ADsymmetric( dim, varType, component, myProd( l, &dy ),
                    S, dimS, S2, ldf, H2, nNewLIS, newLIS, nNewSIS, newSIS, nNewHIS, newHIS );
    
   
    
  // SECOND ORDER FORWARD SWEEP:
  // -------------------------------------
    
    for( run1 = 0; run1 < dimS; run1++ ){
      for( run2 = 0; run2 <= run1; run2++ ){
        delete H[run1*dimS+run2];
	Operator *tmp1 = myProd( S1[run1], S1[run2] );
	Operator *tmp2 = myProd( S1[run1], S2[run2] );
	Operator *tmp3 = myProd( S2[run1], S1[run2] );
	Operator *tmp4 = myProd( S2[run1], S2[run2] );
	Operator *tmp5;
	if( run1 == run2 ) tmp5 = myAdd(tmp2,tmp2);
	else               tmp5 = myAdd(tmp2,tmp3);
	Operator *tmp6 = myProd( tmp1, &dxx );
	Operator *tmp7 = myProd( tmp5, &dxy );
	Operator *tmp8 = myProd( tmp4, &dyy );
	Operator *tmp9  = myAdd ( tmp6, tmp7 );
	Operator *tmp10 = myAdd ( tmp8, tmp9 );
	Operator *tmp11 = myProd( l, tmp10 );
	Operator *tmp12 = myAdd ( tmp11 , H1[run1*dimS+run2] );
	H[run1*dimS+run2] = myAdd ( tmp12 , H2[run1*dimS+run2] );
        delete tmp1;
	delete tmp2;
	delete tmp3;
	delete tmp4;
        delete tmp5;
	delete tmp6;
	delete tmp7;
	delete tmp8;
        delete tmp9;
	delete tmp10;
	delete tmp11;
	delete tmp12;
      }
    }
    
    
  // FIRST ORDER FORWARD SWEEP:
  // -------------------------------------
    
    for( run1 = 0; run1 < dimS; run1++ ){
        delete dfS[run1];
	Operator *tmp1 = myProd( S1[run1], &dx );
	Operator *tmp2 = myProd( S2[run1], &dy );
        dfS[run1] = myAdd(tmp1,tmp2);
        delete tmp1;
	delete tmp2;
    }
    
  // CLEAR MEMORY FROM SWEEPS:
  // -------------------------------------
    
    for( run2 = 0; run2 < dimS; run2++ ){
         delete S1[run2];
         delete S2[run2];
    }
    for( run2 = 0; run2 < dimS*dimS; run2++ ){
         delete H1[run2];
         delete H2[run2];
    }
    delete[] S1;
    delete[] S2;
    delete[] H1;
    delete[] H2;
    
    delete l;
    return SUCCESSFUL_RETURN;
}






CLOSE_NAMESPACE_ACADO


// end of file.
