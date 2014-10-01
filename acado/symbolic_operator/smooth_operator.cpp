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
*    \file src/symbolic_operator/smooth_operator.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_operator/symbolic_operator.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/symbolic_expression/constraint_component.hpp>


BEGIN_NAMESPACE_ACADO


SmoothOperator::SmoothOperator() : Operator( )
{
}

SmoothOperator::~SmoothOperator()
{
}


Operator& SmoothOperator::operator=( const double &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

Operator& SmoothOperator::operator=( const DVector &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

Operator& SmoothOperator::operator=( const DMatrix &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

Operator& SmoothOperator::operator=( const Expression &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}

Operator& SmoothOperator::operator=( const Operator &arg ){

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return emptyTreeProjection;
}


TreeProjection* SmoothOperator::cloneTreeProjection() const{

    // if this routine is ever called something went
    // really wrong ....

    ACADOERROR( RET_UNKNOWN_BUG );
    ASSERT( 1 == 0 );
    return new TreeProjection();
}




Operator& SmoothOperator::operator+=( const double    & arg ){ return operator=( this->operator+(arg) ); }
Operator& SmoothOperator::operator+=( const DVector    & arg ){ return operator=( this->operator+(arg) ); }
Operator& SmoothOperator::operator+=( const DMatrix    & arg ){ return operator=( this->operator+(arg) ); }
Operator& SmoothOperator::operator+=( const Expression& arg ){ return operator=( this->operator+(arg) ); }

Operator& SmoothOperator::operator-=( const double      & arg ){ return operator=( this->operator-(arg) ); }
Operator& SmoothOperator::operator-=( const DVector      & arg ){ return operator=( this->operator-(arg) ); }
Operator& SmoothOperator::operator-=( const DMatrix      & arg ){ return operator=( this->operator-(arg) ); }
Operator& SmoothOperator::operator-=( const Expression  & arg ){ return operator=( this->operator-(arg) ); }

Operator& SmoothOperator::operator*=( const double      & arg ){ return operator=( this->operator*(arg) ); }
Operator& SmoothOperator::operator*=( const DVector      & arg ){ return operator=( this->operator*(arg) ); }
Operator& SmoothOperator::operator*=( const DMatrix      & arg ){ return operator=( this->operator*(arg) ); }
Operator& SmoothOperator::operator*=( const Expression  & arg ){ return operator=( this->operator*(arg) ); }

Operator& SmoothOperator::operator/=( const double      & arg ){ return operator=( this->operator/(arg) ); }
Operator& SmoothOperator::operator/=( const Expression  & arg ){ return operator=( this->operator/(arg) ); }




Expression SmoothOperator::operator+( const double        & arg ) const{ return Expression(*this)+arg; }
Expression SmoothOperator::operator+( const DVector        & arg ) const{ return Expression(*this)+arg; }
Expression SmoothOperator::operator+( const DMatrix        & arg ) const{ return Expression(*this)+arg; }
Expression SmoothOperator::operator+( const Operator& arg ) const{ return Expression(*this)+arg; }
Expression SmoothOperator::operator+( const Expression    & arg ) const{ return Expression(*this)+arg; }

Expression SmoothOperator::operator-( const double        & arg ) const{ return Expression(*this)-arg; }
Expression SmoothOperator::operator-( const DVector        & arg ) const{ return Expression(*this)-arg; }
Expression SmoothOperator::operator-( const DMatrix        & arg ) const{ return Expression(*this)-arg; }
Expression SmoothOperator::operator-( const Operator& arg ) const{ return Expression(*this)-arg; }
Expression SmoothOperator::operator-( const Expression    & arg ) const{ return Expression(*this)-arg; }

Expression SmoothOperator::operator-( ) const{ return -Expression(*this); }

Expression SmoothOperator::operator*( const double        & arg ) const{ return Expression(*this)*arg; }
Expression SmoothOperator::operator*( const DVector        & arg ) const{ return Expression(*this)*arg; }
Expression SmoothOperator::operator*( const DMatrix        & arg ) const{ return Expression(*this)*arg; }


Expression SmoothOperator::operator*( const Operator& arg ) const{

    Expression tmp2(arg);


    Expression tmp1(*this);


    return tmp1*tmp2;
}


Expression SmoothOperator::operator*( const Expression    & arg ) const{ return Expression(*this)*arg; }

Expression SmoothOperator::operator/( const double        & arg ) const{ return Expression(*this)/arg; }
Expression SmoothOperator::operator/( const Operator& arg ) const{ return Expression(*this)/arg; }
Expression SmoothOperator::operator/( const Expression    & arg ) const{ return Expression(*this)/arg; }

ConstraintComponent SmoothOperator::operator<=( const double& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent SmoothOperator::operator>=( const double& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent SmoothOperator::operator==( const double&  b ) const{ return Expression(*this) ==  b; }

ConstraintComponent SmoothOperator::operator<=( const DVector& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent SmoothOperator::operator>=( const DVector& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent SmoothOperator::operator==( const DVector&  b ) const{ return Expression(*this) ==  b; }

ConstraintComponent SmoothOperator::operator<=( const VariablesGrid& ub ) const{ return Expression(*this) <= ub; }
ConstraintComponent SmoothOperator::operator>=( const VariablesGrid& lb ) const{ return Expression(*this) >= lb; }
ConstraintComponent SmoothOperator::operator==( const VariablesGrid&  b ) const{ return Expression(*this) ==  b; }


BooleanType SmoothOperator::isSmooth( ) const
{
    return BT_TRUE;
}


double SmoothOperator::getValue() const{ return INFTY; }

int SmoothOperator::getGlobalIndex( ) const{

	ACADOERROR( RET_UNKNOWN_BUG );
    return -1;
}


Operator* SmoothOperator::passArgument() const{

    return 0;
}

CLOSE_NAMESPACE_ACADO


// end of file.
