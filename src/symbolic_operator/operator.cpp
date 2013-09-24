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



CLOSE_NAMESPACE_ACADO


// end of file.
