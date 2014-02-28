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
*    \file src/symbolic_operator/nonsmooth_operator.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/symbolic_expression/constraint_component.hpp>


BEGIN_NAMESPACE_ACADO


NonsmoothOperator::NonsmoothOperator() : Operator( )
{
}

NonsmoothOperator::~NonsmoothOperator()
{
}


Operator& NonsmoothOperator::operator=( const double &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

Operator& NonsmoothOperator::operator=( const DVector &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

Operator& NonsmoothOperator::operator=( const DMatrix &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

Operator& NonsmoothOperator::operator=( const Expression &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

Operator& NonsmoothOperator::operator=( const Operator &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}


TreeProjection* NonsmoothOperator::cloneTreeProjection() const{

    // if this routine is ever called something went
    // really wrong ....

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return new TreeProjection();
}


Operator& NonsmoothOperator::operator+=( const double    & arg ){ return operator=( this->operator+(arg) ); }
Operator& NonsmoothOperator::operator+=( const DVector    & arg ){ return operator=( this->operator+(arg) ); }
Operator& NonsmoothOperator::operator+=( const DMatrix    & arg ){ return operator=( this->operator+(arg) ); }
Operator& NonsmoothOperator::operator+=( const Expression& arg ){ return operator=( this->operator+(arg) ); }

Operator& NonsmoothOperator::operator-=( const double      & arg ){ return operator=( this->operator-(arg) ); }
Operator& NonsmoothOperator::operator-=( const DVector      & arg ){ return operator=( this->operator-(arg) ); }
Operator& NonsmoothOperator::operator-=( const DMatrix      & arg ){ return operator=( this->operator-(arg) ); }
Operator& NonsmoothOperator::operator-=( const Expression  & arg ){ return operator=( this->operator-(arg) ); }

Operator& NonsmoothOperator::operator*=( const double      & arg ){ return operator=( this->operator*(arg) ); }
Operator& NonsmoothOperator::operator*=( const DVector      & arg ){ return operator=( this->operator*(arg) ); }
Operator& NonsmoothOperator::operator*=( const DMatrix      & arg ){ return operator=( this->operator*(arg) ); }
Operator& NonsmoothOperator::operator*=( const Expression  & arg ){ return operator=( this->operator*(arg) ); }

Operator& NonsmoothOperator::operator/=( const double      & arg ){ return operator=( this->operator/(arg) ); }
Operator& NonsmoothOperator::operator/=( const Expression  & arg ){ return operator=( this->operator/(arg) ); }




Expression NonsmoothOperator::operator+( const double        & arg ) const{ return Expression(*this)+arg; }
Expression NonsmoothOperator::operator+( const DVector        & arg ) const{ return Expression(*this)+arg; }
Expression NonsmoothOperator::operator+( const DMatrix        & arg ) const{ return Expression(*this)+arg; }
Expression NonsmoothOperator::operator+( const Operator& arg ) const{ return Expression(*this)+arg; }
Expression NonsmoothOperator::operator+( const Expression    & arg ) const{ return Expression(*this)+arg; }

Expression NonsmoothOperator::operator-( const double        & arg ) const{ return Expression(*this)-arg; }
Expression NonsmoothOperator::operator-( const DVector        & arg ) const{ return Expression(*this)-arg; }
Expression NonsmoothOperator::operator-( const DMatrix        & arg ) const{ return Expression(*this)-arg; }
Expression NonsmoothOperator::operator-( const Operator& arg ) const{ return Expression(*this)-arg; }
Expression NonsmoothOperator::operator-( const Expression    & arg ) const{ return Expression(*this)-arg; }

Expression NonsmoothOperator::operator-( ) const{ return -Expression(*this); }

Expression NonsmoothOperator::operator*( const double        & arg ) const{ return Expression(*this)*arg; }
Expression NonsmoothOperator::operator*( const DVector        & arg ) const{ return Expression(*this)*arg; }
Expression NonsmoothOperator::operator*( const DMatrix        & arg ) const{ return Expression(*this)*arg; }


Expression NonsmoothOperator::operator*( const Operator& arg ) const{

    Expression tmp2(arg);


    Expression tmp1(*this);


    return tmp1*tmp2;
}


Expression NonsmoothOperator::operator*( const Expression    & arg ) const{ return Expression(*this)*arg; }

Expression NonsmoothOperator::operator/( const double        & arg ) const{ return Expression(*this)/arg; }
Expression NonsmoothOperator::operator/( const Operator& arg ) const{ return Expression(*this)/arg; }
Expression NonsmoothOperator::operator/( const Expression    & arg ) const{ return Expression(*this)/arg; }


ConstraintComponent NonsmoothOperator::operator<=( const double& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent NonsmoothOperator::operator>=( const double& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent NonsmoothOperator::operator==( const double&  b ) const{ return Expression(*this) ==  b; }

ConstraintComponent NonsmoothOperator::operator<=( const DVector& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent NonsmoothOperator::operator>=( const DVector& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent NonsmoothOperator::operator==( const DVector&  b ) const{ return Expression(*this) ==  b; }

ConstraintComponent NonsmoothOperator::operator<=( const VariablesGrid& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent NonsmoothOperator::operator>=( const VariablesGrid& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent NonsmoothOperator::operator==( const VariablesGrid&  b ) const{ return Expression(*this) ==  b; }




returnValue NonsmoothOperator::evaluate( int number, double *x, double *result ){

    //result[0] = value;
    return SUCCESSFUL_RETURN;
}


returnValue NonsmoothOperator::evaluate( EvaluationBase *x ){

    return SUCCESSFUL_RETURN;
}



Operator* NonsmoothOperator::differentiate( int index ){

  return new NonsmoothOperator();
}


Operator* NonsmoothOperator::AD_forward( int dim,
                                        VariableType *varType,
                                        int *component,
                                        Operator **seed,
                                        int &nNewIS,
                                        TreeProjection ***newIS ){

    return new NonsmoothOperator();
}



returnValue NonsmoothOperator::AD_backward( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component, /**< and their components  */
                                        Operator     *seed     , /**< the backward seed     */
                                        Operator    **df       , /**< the result            */
                                        int           &nNewIS  , /**< the number of new IS  */
                                        TreeProjection ***newIS  /**< the new IS-pointer    */ ){

    delete seed;
    return SUCCESSFUL_RETURN;
}



returnValue NonsmoothOperator::AD_symmetric( int            dim       , /**< number of directions  */
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

    delete l;
    return SUCCESSFUL_RETURN; 
}



Operator* NonsmoothOperator::substitute( int index, const Operator *sub ){

    return clone();
}



NeutralElement NonsmoothOperator::isOneOrZero() const{

    return NE_NEITHER_ONE_NOR_ZERO;
}


BooleanType NonsmoothOperator::isDependingOn( VariableType var ) const{

    return BT_FALSE;
}


BooleanType NonsmoothOperator::isDependingOn( int dim,
                                             VariableType *varType,
                                             int *component,
                                             BooleanType   *implicit_dep ){

    return BT_FALSE;
}


BooleanType NonsmoothOperator::isLinearIn( int dim,
                                          VariableType *varType,
                                          int *component,
                                          BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType NonsmoothOperator::isPolynomialIn( int dim,
                                              VariableType *varType,
                                              int *component,
                                              BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType NonsmoothOperator::isRationalIn( int dim,
                                            VariableType *varType,
                                            int *component,
                                            BooleanType   *implicit_dep ){

    return BT_TRUE;
}


BooleanType NonsmoothOperator::isSmooth( ) const
{
    return BT_FALSE;
}


MonotonicityType NonsmoothOperator::getMonotonicity( ){

    return MT_CONSTANT;
}


CurvatureType NonsmoothOperator::getCurvature( ){

    return CT_CONSTANT;
}


returnValue NonsmoothOperator::setMonotonicity( MonotonicityType monotonicity_ ){

    return SUCCESSFUL_RETURN;
}


returnValue NonsmoothOperator::setCurvature( CurvatureType curvature_ ){

    return SUCCESSFUL_RETURN;
}


returnValue NonsmoothOperator::AD_forward( int number, double *x, double *seed,
                                        double *f, double *df ){

      f[0] =  0.0;//value;
     df[0] =  0.0;

     return SUCCESSFUL_RETURN;
}



returnValue NonsmoothOperator::AD_forward( int number, double *seed, double *df ){

     df[0] =  0.0;
     return SUCCESSFUL_RETURN;
}


returnValue NonsmoothOperator::AD_backward( int number, double seed, double *df ){

     return SUCCESSFUL_RETURN;
}


returnValue NonsmoothOperator::AD_forward2( int number, double *seed, double *dseed,
                                         double *df, double *ddf ){

     df[0] = 0.0;
    ddf[0] = 0.0;

    return SUCCESSFUL_RETURN;
}


returnValue NonsmoothOperator::AD_backward2( int number, double seed1, double seed2,
                                          double *df, double *ddf ){

    return SUCCESSFUL_RETURN;
}


std::ostream& NonsmoothOperator::print( std::ostream &stream ) const
{
    return stream; // << value;
}


Operator* NonsmoothOperator::clone() const{

    return new NonsmoothOperator(*this);
}


returnValue NonsmoothOperator::clearBuffer(){

    return SUCCESSFUL_RETURN;
}


returnValue NonsmoothOperator::enumerateVariables( SymbolicIndexList *indexList ){

    return SUCCESSFUL_RETURN;
}

//
// PROTECTED MEMBER FUNCTIONS:
// ---------------------------

OperatorName NonsmoothOperator::getName(){

    return ON_DOUBLE_CONSTANT;
}

BooleanType NonsmoothOperator::isVariable( VariableType &varType, int &component ) const
{
    return BT_FALSE;
}


returnValue NonsmoothOperator::loadIndices( SymbolicIndexList *indexList ){

    return SUCCESSFUL_RETURN;
}


BooleanType NonsmoothOperator::isSymbolic() const{

    return BT_TRUE;
}



double NonsmoothOperator::getValue() const{ return INFTY; }

int NonsmoothOperator::getGlobalIndex( ) const{

	ACADOERROR( RET_UNKNOWN_BUG );
    return -1;
}


Operator* NonsmoothOperator::passArgument() const{

    return 0;
}



CLOSE_NAMESPACE_ACADO


// end of file.
