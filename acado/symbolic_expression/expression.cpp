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
#include <acado/symbolic_operator/symbolic_operator.hpp>

BEGIN_NAMESPACE_ACADO


// ---------------------------------------------------------------------------------------------------
//                                 IMPLEMENTATION OF THE CONSTRUCTORS:
// ---------------------------------------------------------------------------------------------------

Expression::Expression()
{
	construct(VT_UNKNOWN, 0, 0, 0, "");
}

Expression::Expression(const std::string& name_, uint nRows_, uint nCols_, VariableType variableType_, uint globalTypeID)
{
	construct(variableType_, globalTypeID, nRows_, nCols_, name_);
}

Expression::Expression(const std::string &name_)
{
	construct(VT_UNKNOWN, 0, 0, 0, name_);
}

Expression::Expression(uint nRows_, uint nCols_, VariableType variableType_, uint globalTypeID)
{
	construct(variableType_, globalTypeID, nRows_, nCols_, "");
}

Expression::Expression(int nRows_, int nCols_, VariableType variableType_, int globalTypeID)
{
	construct(variableType_, globalTypeID, nRows_, nCols_, "");
}

Expression::Expression(const Operator &tree_)
{
	VariableType tmpType;
	int tmpComp;

	if (tree_.isVariable(tmpType, tmpComp) == BT_TRUE)
	{
		construct(tmpType, tmpComp, 1, 1, "");
	}
	else
	{
		construct(VT_UNKNOWN, 0, 1, 1, "");
	}

	delete element[0];
	element[0] = tree_.clone();
}

Expression::Expression( const double& rhs )
{
	construct(VT_UNKNOWN, 0, 1, 1, "");
	delete element[ 0 ];
	element[ 0 ] = new DoubleConstant(rhs, NE_NEITHER_ONE_NOR_ZERO);
}

Expression::Expression( const DVector& rhs )
{
	construct(VT_UNKNOWN, 0, rhs.getDim(), 1, "");
	for(unsigned el = 0; el < rhs.getDim(); el++ )
	{
		delete element[ el ];
		element[ el ] = new DoubleConstant(rhs( el ), NE_NEITHER_ONE_NOR_ZERO);
	}
}

Expression::Expression( const DMatrix& rhs )
{
	construct(VT_UNKNOWN, 0, rhs.getNumRows(), rhs.getNumCols(), "");
	for(unsigned run1 = 0; run1 < rhs.getNumRows(); run1++ )
	{
		for(unsigned run2 = 0; run2 < rhs.getNumCols(); run2++ )
		{
			delete element[rhs.getNumCols() * run1 + run2];
			element[rhs.getNumCols() * run1 + run2] = 
				new DoubleConstant(rhs(run1, run2), NE_NEITHER_ONE_NOR_ZERO);
		}
	}
}

Expression::Expression(const Expression& rhs)
{
	copy(rhs);
}

Expression::~Expression()
{
	deleteAll();
}

Expression& Expression::operator=( const Expression& rhs )
{
	if (this != &rhs)
	{
		deleteAll();
		copy(rhs);
	}
	return *this;
}

Expression&  Expression::appendRows(const Expression& arg) {
	if (getDim()==0) {operator=(arg);return *this;}
	ASSERT(arg.getNumCols() == getNumCols());
	
        uint run1;
	uint oldDim = dim;
	dim     += arg.dim;
	nRows += arg.getNumRows();

	if( arg.variableType != variableType )
		variableType = VT_UNKNOWN;
	element = (Operator**)realloc(element, dim*sizeof(Operator*) );
	    
	for( run1 = oldDim; run1 < dim; run1++ )
		element[run1] = arg.element[run1-oldDim]->clone();
	
	return *this;
}

// this is still very unefficient code

Expression& Expression::appendCols(const Expression& arg) {
	if (getDim()==0) {operator=(arg);return *this;}
	ASSERT(arg.getNumRows() == getNumRows());
	
	
	Expression E(transpose());
	E.appendRows(arg.transpose());
	
	operator=(E.transpose());
	
	return *this;
}

Expression& Expression::operator<<( const Expression& arg ){
  
    if( dim == 0 ) return operator=(arg);

    uint run1;
    uint oldDim = dim;

    dim   += arg.dim  ;
    nRows += arg.dim  ;

    variableType = VT_VARIABLE;

    if( arg.isVariable() == BT_FALSE ) variableType = VT_UNKNOWN;

    element = (Operator**)realloc(element, dim*sizeof(Operator*) );

    for( run1 = oldDim; run1 < dim; run1++ )
        element[run1] = arg.element[run1-oldDim]->clone();

    return *this;
}


std::ostream& Expression::print(std::ostream& stream) const
{
	uint run1;
	stream << "[ ";
	if (dim) {
		for (run1 = 0; run1 < dim - 1; run1++)
			stream << *element[run1] << " , ";
		stream << *element[dim - 1];
	}
	stream << "]";

	return stream;
}


std::ostream& operator<<( std::ostream& stream, const Expression &arg )
{
    return arg.print(stream);
}

Expression Expression::operator()( uint idx ) const{

    ASSERT( idx < getDim( ) );

    Expression tmp(1);

    delete tmp.element[0];
    tmp.element[0] = element[idx]->clone();

    tmp.component    = component + idx;
    tmp.variableType = variableType;

    return tmp;
}

Expression Expression::operator()( uint rowIdx, uint colIdx ) const{

    ASSERT( rowIdx < getNumRows( ) );
    ASSERT( colIdx < getNumCols( ) );

    Expression tmp(1);

    delete tmp.element[0];
    tmp.element[0] = element[rowIdx*getNumCols()+colIdx]->clone();

    tmp.component    = component + rowIdx*getNumCols() + colIdx;
    tmp.variableType = variableType;

    return tmp;
}


Operator& Expression::operator()( uint idx ){

    switch( variableType ){

        case  VT_INTERMEDIATE_STATE:
              ASSERT( idx < getDim( ) );
              return *element[idx];

        case VT_UNKNOWN:
              ASSERT( idx < getDim( ) );
              delete  element[idx];
              element[idx] = new TreeProjection();
              return *element[idx];

        default:
              ASSERT( idx < getDim( ) );
              return *element[idx];
    }
    ASSERT( 1 == 0 );
    return *element[0];
}


Operator& Expression::operator()( uint rowIdx, uint colIdx ){

    switch( variableType ){

        case  VT_INTERMEDIATE_STATE:
              ASSERT( rowIdx < getNumRows( ) );
              ASSERT( colIdx < getNumCols( ) );
              return *element[rowIdx*getNumCols()+colIdx];

//        case  VT_UNKNOWN:
        default:
              ASSERT( rowIdx < getNumRows( ) );
              ASSERT( colIdx < getNumCols( ) );
              delete  element[rowIdx*getNumCols()+colIdx];
              element[rowIdx*getNumCols()+colIdx] = new TreeProjection();
              return *element[rowIdx*getNumCols()+colIdx];

//               ASSERT( 1 == 0 );
//               return *element[0];
    }
    ASSERT( 1 == 0 );
    return *element[0];
}


Expression operator+( const Expression  & arg1, const Expression  & arg2 )
{return arg1.add(arg2);}
Expression operator-( const Expression  & arg1, const Expression  & arg2 )
{return arg1.sub(arg2);}
Expression operator*( const Expression  & arg1, const Expression  & arg2 )
{return arg1.mul(arg2);}
Expression operator/( const Expression  & arg1, const Expression  & arg2 )
{return arg1.div(arg2);}

Expression& Expression::operator+=(const Expression &arg)
{return static_cast<Expression&>(*this) = static_cast<Expression*>(this)->add(arg);}

Expression& Expression::operator-=(const Expression &arg)
{return static_cast<Expression&>(*this) = static_cast<Expression*>(this)->sub(arg);}

Expression& Expression::operator*=(const Expression &arg)
{return static_cast<Expression&>(*this) = static_cast<Expression*>(this)->mul(arg);}

Expression& Expression::operator/=(const Expression &arg)
{return static_cast<Expression&>(*this) = static_cast<Expression*>(this)->div(arg);}

Expression Expression::add( const Expression& arg ) const{

    ASSERT( getNumRows() == arg.getNumRows() );
    ASSERT( getNumCols() == arg.getNumCols() );

    uint i,j;

    Expression tmp("", getNumRows(), getNumCols() );

    for( i=0; i<getNumRows(); ++i ){
        for( j=0; j<getNumCols(); ++j ){

            delete tmp.element[i*getNumCols()+j];
            if( element[i*getNumCols()+j]->isOneOrZero() != NE_ZERO ){
                if( arg.element[i*getNumCols()+j]->isOneOrZero() != NE_ZERO )
                    tmp.element[i*getNumCols()+j] = new Addition( element[i*getNumCols()+j]->clone(),
                                                    arg.element[i*getNumCols()+j]->clone() );
                else
                    tmp.element[i*getNumCols()+j] = element[i*getNumCols()+j]->clone();
            }
            else{
                if( arg.element[i*getNumCols()+j]->isOneOrZero() != NE_ZERO )
                     tmp.element[i*getNumCols()+j] = arg.element[i*getNumCols()+j]->clone();
                else tmp.element[i*getNumCols()+j] = new DoubleConstant(0.0,NE_ZERO);
            }
        }
    }
    return tmp;
}


Expression Expression::sub( const Expression& arg ) const{

    ASSERT( getNumRows() == arg.getNumRows() );
    ASSERT( getNumCols() == arg.getNumCols() );

    uint i,j;

    Expression tmp("", getNumRows(), getNumCols() );

    for( i=0; i<getNumRows(); ++i ){
        for( j=0; j<getNumCols(); ++j ){

            delete tmp.element[i*getNumCols()+j];
            if( element[i*getNumCols()+j]->isOneOrZero() != NE_ZERO ){
                if( arg.element[i*getNumCols()+j]->isOneOrZero() != NE_ZERO )
                    tmp.element[i*getNumCols()+j] = new Subtraction( element[i*getNumCols()+j]->clone(),
                                                    arg.element[i*getNumCols()+j]->clone() );
                else
                    tmp.element[i*getNumCols()+j] = element[i*getNumCols()+j]->clone();
            }
            else{
                if( arg.element[i*getNumCols()+j]->isOneOrZero() != NE_ZERO )
                     tmp.element[i*getNumCols()+j] = new Subtraction( new DoubleConstant(0.0,NE_ZERO),
                                                                      arg.element[i*getNumCols()+j]->clone() );
                else tmp.element[i*getNumCols()+j] = new DoubleConstant(0.0,NE_ZERO);
            }
        }
    }
    return tmp;
}


Operator* Expression::product( const Operator *a, const Operator *b ) const{


    switch( a->isOneOrZero() ){

        case NE_ZERO:
             return new DoubleConstant( 0.0, NE_ZERO );

        case NE_ONE:
             return b->clone();

        default:

             switch( b->isOneOrZero() ){

                 case NE_ZERO:
                      return new DoubleConstant( 0.0, NE_ZERO );

                 case NE_ONE:
                      return a->clone();

                 default:
                      return new Product( a->clone(), b->clone() );
             }
    }
    return 0;
}



Expression Expression::mul( const Expression& arg ) const{
    if (getDim()==0 || arg.getDim()==0) return Expression();
    // uninitialized expressions yield uninitialized results

    uint i,j,k;

    if( getNumRows() == 1 && getNumCols( ) == 1 ){

        Expression tmp("", arg.getNumRows(), arg.getNumCols() );

        for( i = 0; i< arg.getDim(); i++ ){

             delete tmp.element[i];
             Operator *prod = product( element[0], arg.element[i] );
             tmp.element[i] = prod->clone();

             delete prod;
        }
        return tmp;
    }


    if( arg.getNumRows() == 1 && arg.getNumCols( ) == 1 ){

        Expression tmp("", getNumRows(), getNumCols() );

        for( i = 0; i< getDim(); i++ ){

             delete tmp.element[i];
             Operator *prod = product( arg.element[0], element[i] );
             tmp.element[i] = prod->clone();

             delete prod;
        }
        return tmp;
    }


    ASSERT( getNumCols( ) == arg.getNumRows( ) );

    uint newNumRows = getNumRows( );
    uint newNumCols = arg.getNumCols( );
    IntermediateState tmp = zeros<double>( newNumRows, newNumCols );

    for( i=0; i<newNumRows; ++i ){
        for( j=0; j<newNumCols; ++j ){
            for( k=0; k<getNumCols( ); ++k ){

                 Operator *tmpO = product( element[i*getNumCols()+k],
                                           arg.element[k*arg.getNumCols()+j] );

                 if( tmpO->isOneOrZero() != NE_ZERO )
                     tmp(i,j) += *tmpO;

                 delete tmpO;
            }
        }
    }
    return tmp;
}


Expression Expression::div( const Expression& arg ) const{

    ASSERT( arg.getNumRows() == 1 );
    ASSERT( arg.getNumCols() == 1 );

    uint i;

    Expression tmp("", getNumRows(), getNumCols() );

    for( i = 0; i< getDim(); i++ ){
         delete tmp.element[i];
         tmp.element[i] = new Quotient( element[i]->clone(), arg.element[0]->clone() );
    }
    return tmp;
}

Expression Expression::getInverse() const{

    ASSERT( getNumRows() == getNumCols() );

    int i,j,k;                 // must really be int, never use uint here.
    int M = getNumRows();      // must really be int, never use uint here.

	IntermediateState tmp("", M, 2 * M);

    for( i = 0; i < M; i++ ){
        for( j = 0; j < 2*M; j++ ){
            if( j <  M   ) tmp(i,j) = operator()(i,j);
            else           tmp(i,j) = 0.0        ;
            if( j == M+i ) tmp(i,j) = 1.0        ;
        }
    }

    for( i = 0; i < M-1; i++ )
         for( k = i+1; k < M; k++ )
             for( j = 2*M-1; j >= i; j-- )
                  tmp(k,j) -= ( tmp(i,j)*tmp(k,i) )/tmp(i,i);

    for( i = M-1; i > 0; i-- )
        for( k = i-1; k >= 0; k-- )
            for( j = 2*M-1; j >= i; j-- )
                tmp(k,j) -= (tmp(i,j)*tmp(k,i))/tmp(i,i);

    for( i = 0; i < M; i++ )
        for( j = 2*M-1; j >= 0; j-- )
             tmp(i,j) = tmp(i,j)/tmp(i,i);

    Expression I("", M,M);
    for( i = 0; i < M; i++ ){
        for( j = 0; j < M; j++ ){
            delete I.element[i*M+j];
            I.element[i*M+j] = tmp.element[i*2*M+j+M]->clone();
        }
    }

    return I;
}


Expression Expression::getRow( const uint& rowIdx ) const{

    uint run1;
    ASSERT( rowIdx < getNumRows() );

    Expression tmp("", 1, (int) getNumCols() );

    for( run1 = 0; run1 < getNumCols(); run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = element[rowIdx*getNumCols()+run1]->clone();
    }
    return tmp;
}


Expression Expression::getRows( const uint& rowIdx1, const uint& rowIdx2 ) const{

    uint run1, run2, _nRows;
    ASSERT( rowIdx1 < getNumRows() );
    ASSERT( rowIdx2 <= getNumRows() );
    _nRows = rowIdx2 - rowIdx1;

    Expression tmp("", _nRows, (int) getNumCols() );

    for( run1 = 0; run1 < _nRows; run1++ ){
    	for( run2 = 0; run2 < getNumCols(); run2++ ){
    		delete tmp.element[run1*getNumCols()+run2];
    		tmp.element[run1*getNumCols()+run2] = element[(rowIdx1+run1)*getNumCols()+run2]->clone();
    	}
    }
    return tmp;
}


Expression Expression::getCol( const uint& colIdx ) const{

    uint run1;
    ASSERT( colIdx < getNumCols() );

    Expression tmp("", (int) getNumRows(), 1 );

    for( run1 = 0; run1 < getNumRows(); run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = element[run1*getNumCols()+colIdx]->clone();
    }
    return tmp;
}


Expression Expression::getCols( const uint& colIdx1, const uint& colIdx2 ) const{

    uint run1, run2, _nCols;
    ASSERT( colIdx1 < getNumCols() );
    ASSERT( colIdx2 <= getNumCols() );
    _nCols = colIdx2 - colIdx1;

    Expression tmp("", (int) getNumRows(), _nCols );

    for( run1 = 0; run1 < getNumRows(); run1++ ){
    	for( run2 = 0; run2 < _nCols; run2++ ){
    		delete tmp.element[run1*_nCols+run2];
    		tmp.element[run1*_nCols+run2] = element[run1*getNumCols()+colIdx1+run2]->clone();
    	}
    }
    return tmp;
}


Expression Expression::getSubMatrix( const uint& rowIdx1, const uint& rowIdx2, const uint& colIdx1, const uint& colIdx2 ) const{

    uint run1, run2, _nRows, _nCols;
    ASSERT( colIdx1 < getNumCols() );
    ASSERT( colIdx2 <= getNumCols() );
    _nCols = colIdx2 - colIdx1;
    ASSERT( rowIdx1 < getNumRows() );
    ASSERT( rowIdx2 <= getNumRows() );
    _nRows = rowIdx2 - rowIdx1;

    Expression tmp("", _nRows, _nCols );

    for( run1 = 0; run1 < _nRows; run1++ ){
    	for( run2 = 0; run2 < _nCols; run2++ ){
    		delete tmp.element[run1*_nCols+run2];
    		tmp.element[run1*_nCols+run2] = element[(rowIdx1+run1)*getNumCols()+colIdx1+run2]->clone();
    	}
    }
    return tmp;
}


DMatrix Expression::getDependencyPattern( const Expression& arg ) const{

	DMatrix tmp;
	if( arg.getDim() == 0 ) return tmp;

    Function f;
    f << backwardDerivative( *this, arg );

//    FILE *test=fopen("test-getdep.txt","w");
//    test << f;
//    fclose(test);
    
    double *x      = new double[ f.getNumberOfVariables()+1 ];
    double *result = new double[ f.getDim()                 ];

    // TODO: DANGEROUS CODE --> TALK TO MILAN ABOUT THIS
    int run1;
    srand(1.0);
    for( run1 = 0; run1 < f.getNumberOfVariables()+1; run1++ )
    	x[run1] = 1.0 + (double)rand() / RAND_MAX;

    // EVALUATE f AT THE POINT  (tt,xx):
    // ---------------------------------
    f.evaluate( 0, x, result );

    tmp = DMatrix( getDim(), arg.getDim(), result );

    delete[] result;
    delete[] x;

    return tmp;
}

DMatrix Expression::getSparsityPattern() const
{
	DMatrix res = zeros<double>(getNumRows(), getNumCols());

	for (unsigned el = 0; el < getDim(); ++el)
	{
		Operator* foo = getOperatorClone(el);
		if (foo->isOneOrZero() != NE_ZERO)
			res(el) = 1.0;
		delete foo;
	}

	return res;
}



Expression Expression::getSin( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Sin( element[run1]->clone() ); 
    }
    return tmp;
}

Expression Expression::getCos( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Cos( element[run1]->clone() ); 
    }
    return tmp;
}

Expression Expression::getTan( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Tan( element[run1]->clone() ); 
    }
    return tmp;
}

Expression Expression::getAsin( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Asin( element[run1]->clone() ); 
    }
    return tmp;
}

Expression Expression::getAcos( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Acos( element[run1]->clone() ); 
    }
    return tmp;
}

Expression Expression::getAtan( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Atan( element[run1]->clone() ); 
    }
    return tmp;
}

Expression Expression::getExp( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Exp( element[run1]->clone() ); 
    }
    return tmp;
}

Expression Expression::getSqrt( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Power( element[run1]->clone(), new DoubleConstant( 0.5, NE_NEITHER_ONE_NOR_ZERO ) ); 
    }
    return tmp;
}

Expression Expression::getLn( ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Logarithm( element[run1]->clone() ); 
    }
    return tmp;
}


Expression Expression::getPow( const Expression& arg ) const{

    ASSERT( arg.getDim() == 1 );

    Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Power( element[run1]->clone(), arg.element[0]->clone() ); 
    }
    return tmp;
}


Expression Expression::getPowInt( const int &arg ) const{

	Expression tmp("", nRows, nCols);
    uint run1;

    for( run1 = 0; run1 < dim; run1++ ){
        delete tmp.element[run1];
        tmp.element[run1] = new Power_Int( element[run1]->clone(), arg );
    }
    return tmp;
}



Expression Expression::transpose( ) const{

	Expression tmp("", getNumCols(), getNumRows());

    uint run1, run2;

    for( run1 = 0; run1 < getNumRows(); run1++ ){
        for( run2 = 0; run2 < getNumCols(); run2++ ){
             delete tmp.element[run2*getNumRows()+run1];
             tmp.element[run2*getNumRows()+run1] = element[run1*getNumCols()+run2]->clone();
        }
    }
    return tmp;
}



Expression Expression::getSumSquare( ) const{

    uint run1;

    Expression result = operator*(transpose(), *this);

    CurvatureType c = CT_CONSTANT;
    CurvatureType cc;

    for( run1 = 0; run1 < getDim(); run1++ ){

        cc = element[run1]->getCurvature();

        if( cc != CT_CONSTANT ){
            if( cc == CT_AFFINE &&
                (c == CT_CONSTANT || c == CT_AFFINE) )
                 c = CT_AFFINE;
            else c = CT_NEITHER_CONVEX_NOR_CONCAVE;
        }
    }

    if( c == CT_CONSTANT )
        result.element[0]->setCurvature(CT_CONSTANT);

    if( c == CT_AFFINE )
        result.element[0]->setCurvature(CT_CONVEX);

    return result;

}


Expression Expression::getLogSumExp( ) const{

    uint run1;

    Expression result;
    result = exp( operator()(0) );

    for( run1 = 1; run1 < getDim(); run1++ )
        result = result + exp( operator()(run1) );

    result = ln( result );

    CurvatureType c = CT_CONSTANT;
    CurvatureType cc;

    for( run1 = 0; run1 < getDim(); run1++ ){

        cc = element[run1]->getCurvature();

        if( cc != CT_CONSTANT ){
            if( cc == CT_AFFINE ){
                if( c == CT_CONSTANT || c == CT_AFFINE )
                    c = CT_AFFINE;
                else{
                    if( c == CT_CONVEX ){
                        c = CT_CONVEX;
                    }
                    else{
                        c = CT_NEITHER_CONVEX_NOR_CONCAVE;
                    }
                }
            }
            else{
                if( cc == CT_CONVEX &&
                    (c == CT_CONSTANT || c == CT_AFFINE || c == CT_CONVEX ) )
                     c = CT_CONVEX;
                else
                     c = CT_NEITHER_CONVEX_NOR_CONCAVE;
            }
        }
    }

    if( c == CT_CONSTANT )
        result.element[0]->setCurvature(CT_CONSTANT);

    if( c == CT_AFFINE || c == CT_CONVEX )
        result.element[0]->setCurvature(CT_CONVEX);

    return result;
}


Expression Expression::getEuclideanNorm( ) const{

    uint run1;

    Expression result = sqrt( getSumSquare() );

    CurvatureType c = CT_CONSTANT;
    CurvatureType cc;

    for( run1 = 0; run1 < getDim(); run1++ ){

        cc = element[run1]->getCurvature();

        if( cc != CT_CONSTANT ){
            if( cc == CT_AFFINE &&
                (c == CT_CONSTANT || c == CT_AFFINE) )
                 c = CT_AFFINE;
            else c = CT_NEITHER_CONVEX_NOR_CONCAVE;
        }
    }

    if( c == CT_CONSTANT )
        result.element[0]->setCurvature(CT_CONSTANT);

    if( c == CT_AFFINE )
        result.element[0]->setCurvature(CT_CONVEX);

    return result;
}




Expression Expression::getEntropy( ) const{

    ACADOWARNING( RET_NOT_IMPLEMENTED_YET );
    return *this;
}


Expression Expression::getDot( ) const{

    if( variableType != VT_DIFFERENTIAL_STATE ){

        ACADOERROR( RET_INVALID_ARGUMENTS );
        ASSERT( 1 == 0 );
    }

    Expression tmp("", getNumRows(), getNumCols(), VT_DDIFFERENTIAL_STATE, component );
    return tmp;
}


Expression Expression::getNext( ) const{

    return getDot();
}


Expression Expression::ADforward ( const Expression &arg ) const{

    ASSERT(     getNumCols() == 1 );
    ASSERT( arg.getNumCols() == 1 );

	Expression result("", getNumRows(), arg.getNumRows());

    uint run1, run2;

    Expression seed( arg.getNumRows() );

    for( run1 = 0; run1 < arg.getNumRows(); run1++ ){

        delete seed.element[run1];
        seed.element[run1] = new DoubleConstant( 1.0, NE_ONE );

        Expression tmp = ADforward( arg, seed );

        delete seed.element[run1];
        seed.element[run1] = new DoubleConstant( 0.0, NE_ZERO );

        for( run2 = 0; run2 < getNumRows(); run2++ ){
            delete result.element[run2*arg.getNumRows()+run1];
            result.element[run2*arg.getNumRows()+run1] = tmp.element[run2]->clone();
        }
    }

    return result;
}


Expression Expression::ADforward ( const VariableType &varType_, const int *arg, int nV ) const{
 
    ASSERT( getNumCols() == 1 );

	Expression result("", (int) getNumRows(), nV);

    int run1, run2;

    IntermediateState seed( nV );

    for( run1 = 0; run1 < nV; run1++ ) seed(run1) = 0;

    for( run1 = 0; run1 < nV; run1++ ){

        seed(run1) = 1.0;

        Expression tmp = ADforward( varType_, arg, seed );

        seed(run1) = 0.0;

        for( run2 = 0; run2 < (int) getNumRows(); run2++ ){
            delete result.element[run2*nV+run1];
            result.element[run2*nV+run1] = tmp.element[run2]->clone();
        }
    }
    return result;
}



Expression Expression::ADbackward ( const Expression &arg ) const{

    ASSERT(     getNumCols() == 1 );
    ASSERT( arg.getNumCols() == 1 );

	Expression result("", getNumRows(), arg.getNumRows());

    uint run1, run2;

    Expression seed( getNumRows() );

    for( run1 = 0; run1 < getNumRows(); run1++ ){

        delete seed.element[run1];
        seed.element[run1] = new DoubleConstant( 1.0, NE_ONE );

        Expression tmp = ADbackward( arg, seed );

        delete seed.element[run1];
        seed.element[run1] = new DoubleConstant( 0.0, NE_ZERO );

        for( run2 = 0; run2 < arg.getNumRows(); run2++ ){
            delete result.element[run1*arg.getNumRows()+run2];
            result.element[run1*arg.getNumRows()+run2] = tmp.element[run2]->clone();
        }
    }

    return result;
}


Expression Expression::ADforward ( const Expression &arg, const Expression &seed ) const{

    unsigned int run1;
    const unsigned int n = arg.getDim();

    ASSERT( arg .isVariable() == BT_TRUE );
    ASSERT( seed.getDim    () == n       );

    VariableType *varType   = new VariableType[n];
    int          *Component = new int[n];

    for( run1 = 0; run1 < n; run1++ ){
    	arg.element[run1]->isVariable(varType[run1],Component[run1]);
    }

	Expression result = ADforward( varType, Component, seed );
    delete[] Component;
    delete[] varType;

    return result;
}


Expression Expression::ADforward (  const VariableType &varType_,
								   	   const int          *arg     ,
								   	   const Expression   &seed      ) const{

	VariableType *varType = new VariableType[seed.getDim()];
	for( uint run1 = 0; run1 < seed.getDim(); run1++ ) varType[run1] = varType_;
	Expression tmp = ADforward( varType, arg, seed );
	delete[] varType;
	return tmp;
}


Expression Expression::ADforward ( const VariableType *varType_,
								   const int          *arg     ,
								   const Expression   &seed      ) const{

    unsigned int run1, run2;
    const unsigned int n = seed.getDim();

	Expression result("", getNumRows(), getNumCols());

    VariableType  *varType   = new VariableType[n];
    int           *Component = new int         [n];
    Operator     **seed1     = new Operator*   [n];

    for( run1 = 0; run1 < n; run1++ ){
        varType  [run1] = varType_[run1];
        Component[run1] = arg[run1];
        seed1    [run1] = seed.element[run1]->clone();
    }

    for( run1 = 0; run1 < getDim(); run1++ ){

        delete result.element[run1];

        int Dim = n;
        int nIS = 0;
        TreeProjection **IS = 0;

        element[run1]->initDerivative();
        result.element[run1] = element[run1]->AD_forward( Dim, varType, Component, seed1, nIS, &IS );

        for( run2 = 0; (int) run2 < nIS; run2++ ){
            if( IS[run2] != 0 ){
                delete IS[run2];
            }
        }
        if( IS != 0 )
            free(IS);
    }

    delete[] varType  ;
    delete[] Component;

    for( run1 = 0; run1 < n; run1++ )
        delete seed1[run1];

    delete[] seed1;

    return result;
}


Expression Expression::getODEexpansion( const int &order, const int *arg ) const{
 
	IntermediateState coeff("", (int) dim, order+2 );
	
    VariableType  *vType = new VariableType[dim*(order+1)+1];
    int           *Comp  = new int         [dim*(order+1)+1];
    Operator     **seed  = new Operator*   [dim*(order+1)+1];
	
	Operator **der = new Operator*[dim*(order+1)];
	
	vType[0] = VT_TIME;
	Comp [0] = 0      ;
	seed [0] = new DoubleConstant( 1.0 , NE_ONE );
	
	for( uint i=0; i<dim; i++ ){
		coeff(i,0)   = Expression("",1,1,VT_DIFFERENTIAL_STATE,arg[i]);
		coeff(i,1)   = operator()(i);
		vType[i+1]   = VT_DIFFERENTIAL_STATE;
		Comp [i+1]   = arg[i];
		seed [i+1]   = coeff.element[(order+2)*i+1];
		der[i]       = element[i]->clone();
	}
	
	int nIS = 0;
	TreeProjection **IS = 0;
	
	for( int j=0; j<order; j++ ){
		for( uint i=0; i<dim; i++ ){
			der[dim*j+i]->initDerivative();
			der[dim*(j+1)+i] = der[dim*j+i]->AD_forward( (j+1)*dim+1, vType, Comp, seed, nIS, &IS );
		}
		for( uint i=0; i<dim; i++ ){
			coeff(i,j+2) = *der[dim*(j+1)+i];
			vType[dim*(j+1)+i+1] = VT_INTERMEDIATE_STATE;
			Comp [dim*(j+1)+i+1] = coeff.element[(order+2)*i+j+1]->getGlobalIndex();
			seed [dim*(j+1)+i+1] = coeff.element[(order+2)*i+j+2];
		}
	}
	
	for( int run = 0; run < nIS; run++ ){
		
		if( IS[run] != 0 ) delete IS[run];
	}
	if( IS != 0 ) free(IS);
	
	delete[] vType;
	delete[] Comp;
	for( uint i=0; i<dim*(order+1); i++ ) delete der[i];
	delete[] der;
	delete seed[0];
	delete[] seed;
	
	return coeff;
}



Expression Expression::ADbackward( const Expression &arg, const Expression &seed ) const{

    int Dim = arg.getDim();
    int run1, run2;

    ASSERT( arg .isVariable() == BT_TRUE  );
    ASSERT( seed.getDim    () == getDim() );

	Expression result("", arg.getNumRows(), arg.getNumCols());

    VariableType *varType   = new VariableType[Dim];
    int          *Component = new int         [Dim];
    Operator    **iresult   = new Operator*   [Dim];

    for( run1 = 0; run1 < Dim; run1++ ){
        arg.element[run1]->isVariable(varType[run1],Component[run1]);
    }

	int nIS = 0;
	TreeProjection **IS = 0;
	
    for( run1 = 0; run1 < (int) getDim(); run1++ ){

        Operator *seed1 = seed.element[run1]->clone();

        for( run2 = 0; run2 < Dim; run2++ )
             iresult[run2] = new DoubleConstant(0.0,NE_ZERO);

        element[run1]->initDerivative();
        element[run1]->AD_backward( Dim, varType, Component, seed1, iresult, nIS, &IS );

        for( run2 = 0; run2 < Dim; run2++ ){
            Operator *sum = result.element[run2]->clone();
            delete result.element[run2];
            result.element[run2] = new Addition( sum->clone(), iresult[run2]->clone() );
            delete sum;
            delete iresult[run2];
        }
    }


	for( int run = 0; run < nIS; run++ ){
		
		if( IS[run] != 0 ) delete IS[run];
	}
	if( IS != 0 ) free(IS);
    
    delete[] iresult   ;
    delete[] varType   ;
    delete[] Component ;

    return result;
}

Expression Expression::ADsymmetric( 	const Expression &arg, /** argument      */
					const Expression &S  , /** forward seed  */
					const Expression &l  , /** backward seed */
					Expression *dfS,    /** first order forward  result */
					Expression *ldf   /** first order backward result */ ) const{

	int Dim = arg.getDim();
	ASSERT( (int) S.getNumRows() == Dim      );
	ASSERT( (int) S.getNumRows() == (int) S.getNumCols() );
	
	IntermediateState H    = ADsymmetric( arg, l, dfS, ldf );
	IntermediateState sum  = zeros<double>(Dim,Dim);
	IntermediateState sum2 = zeros<double>(Dim,Dim);
	
	int i,j,k;
	for( i=0; i<Dim; i++ )
		for( j=0; j<Dim; j++ ){
			for( k=0; k<=i; k++ ){
				sum(i,j) += H(i,k)*S(k,j);
			}
			for( k=i+1; k<Dim; k++ ){
				sum(i,j) += H(k,i)*S(k,j);
			}
		}

	for( i=0; i<Dim; i++ )
		for( j=0; j<=i; j++ )
			for( k=0; k<Dim; k++ )
				sum2(i,j) += S(k,i)*sum(k,j);

	for( i=0; i<Dim; i++ )
		for( j=0; j<i; j++ )
			sum2(j,i) = sum2(i,j);

	if( dfS != 0 ) {
		// multiplication with the proper forward seeds:
		*dfS = *dfS*S;
	}
	  
	return sum2;
}


Expression Expression::ADsymmetric( 	const Expression &arg, /** argument      */
					const Expression &l  , /** backward seed */
					Expression *dfS,    /** first order forward  result */
					Expression *ldf   /** first order backward result */ ) const{

	int Dim = arg.getDim();
	int nS  = Dim;
	int run1, run2;
	
	Expression S = eye<double>(Dim);
	
	ASSERT( arg .isVariable()    == BT_TRUE  );
	ASSERT( l.getDim()           == getDim() );
	
	Expression result( nS, nS );

	VariableType *varType   = new VariableType[Dim];
	int          *Component = new int         [Dim];
	Operator    **dS        = new Operator*   [nS];
	Operator    **ld        = new Operator*   [Dim];
	Operator    **H         = new Operator*   [nS*nS];

	for( run1 = 0; run1 < Dim; run1++ ){
		arg.element[run1]->isVariable(varType[run1],Component[run1]);
	}

	int nLIS = 0;
	int nSIS = 0;
	int nHIS = 0;
	TreeProjection **LIS = 0;
	TreeProjection **SIS = 0;
	TreeProjection **HIS = 0;

	Expression tmp((int) getDim(),Dim);
	Expression tmp2(Dim);
	
	for( run1 = 0; run1 < (int) getDim(); run1++ ){

		Operator *l1 = l.element[run1]->clone();
		Operator **S1 = new Operator*[Dim*nS];

		for( run2 = 0; run2 < Dim*nS; run2++ )
			S1[run2] = S.element[run2]->clone();

		for( run2 = 0; run2 < nS; run2++ )
			dS[run2] = new DoubleConstant(0.0,NE_ZERO);

		for( run2 = 0; run2 < Dim; run2++ )
			ld[run2] = new DoubleConstant(0.0,NE_ZERO);

		for( run2 = 0; run2 < nS*nS; run2++ )
			H[run2] = new DoubleConstant(0.0,NE_ZERO);

		element[run1]->initDerivative();
		element[run1]->AD_symmetric( Dim, varType, Component, l1, S1, nS, dS, ld, H, nLIS, &LIS, nSIS, &SIS, nHIS, &HIS );

		int run3 = 0;

		for( run2 = 0; run2 < nS; run2++ ){
			for( run3 = 0; run3 < run2; run3++ ){
				Operator *sum = result.element[run2*nS+run3]->clone();
				delete result.element[run2*nS+run3];
				delete result.element[run3*nS+run2];
				result.element[run2*nS+run3] = sum->myAdd( sum, H[run2*nS+run3] );
				result.element[run3*nS+run2] = sum->myAdd( sum, H[run2*nS+run3] );
				delete sum;
			}
			Operator *sum = result.element[run2*nS+run2]->clone();
			delete result.element[run2*nS+run2];
			result.element[run2*nS+run2] = sum->myAdd( sum, H[run2*nS+run2] );
			delete sum;
		}

		if( dfS != 0 ){
			for( run2 = 0; run2 < nS; run2++ ){
				delete tmp.element[run1*nS+run2];
				tmp.element[run1*nS+run2] = dS[run2]->clone();
			}
		}

		if( ldf != 0 ){
		   for( run2 = 0; run2 < nS; run2++ ){
			Operator *sum = tmp2.element[run2]->clone();
			delete tmp2.element[run2];
			tmp2.element[run2] = sum->myAdd( sum, ld[run2] );
			delete sum;
		  }
		}

		for( run2 = 0; run2 < Dim*nS; run2++ )
			delete S1[run2];

		for( run2 = 0; run2 < nS; run2++ )
			delete dS[run2];

		for( run2 = 0; run2 < Dim; run2++ )
			delete ld[run2];

		for( run2 = 0; run2 < nS*nS; run2++ )
			delete H[run2];

		delete[] S1;
	}

	if( dfS != 0 ) *dfS = tmp ;
	if( ldf != 0 ) *ldf = tmp2;
	
	
	for( int run = 0; run < nLIS; run++ ){
		if( LIS[run] != 0 ) delete LIS[run];
	}
	if( LIS != 0 ) free(LIS);

	for( int run = 0; run < nSIS; run++ ){
		if( SIS[run] != 0 ) delete SIS[run];
	}
	if( SIS != 0 ) free(SIS);

	for( int run = 0; run < nHIS; run++ ){
		if( HIS[run] != 0 ) delete HIS[run];
	}
	if( HIS != 0 ) free(HIS);

	delete[] dS        ;
	delete[] ld        ;
	delete[] H         ;
	delete[] varType   ;
	delete[] Component ;

	return result;
}


returnValue Expression::substitute( int idx, const Expression &arg ) const{

    ASSERT( arg.getDim() == 1 );

    uint i;
    for( i = 0; i < getDim(); i++ )
        if( element[i] != 0 ) element[i]->substitute( idx, arg.element[0] );

    return SUCCESSFUL_RETURN;
}


Expression Expression::operator-() const{

    uint run1, run2;
	Expression tmp("", getNumRows(), getNumCols());

    for( run1 = 0; run1 < getNumRows(); run1++ ){
        for( run2 = 0; run2 < getNumCols(); run2++ ){
             delete tmp.element[run1*getNumCols()+run2];
             tmp.element[run1*getNumCols()+run2] = new Subtraction( new DoubleConstant(0.0,NE_ZERO),
                                                                    element[run1*getNumCols()+run2]->clone() );
        }
    }
    return tmp;
}





//
// PROTECTED MEMBER FUNCTIONS:
//

void Expression::construct( VariableType variableType_, uint globalTypeID, uint nRows_, uint nCols_, const std::string &name_ ){

    nRows        = nRows_       ;
    nCols        = nCols_       ;
    dim          = nRows*nCols  ;
    variableType = variableType_;
    component    = globalTypeID ;
    name         = name_        ;

    uint i;
    element = (Operator**)calloc(dim,sizeof(Operator*));

    for( i = 0; i < dim; i++ ){

        switch( variableType ){
            case VT_UNKNOWN           : element[i] = new DoubleConstant( 0.0, NE_ZERO ); break;
            case VT_INTERMEDIATE_STATE:
                                        element[i] = new TreeProjection( "" );
                                        break;
            default                   : element[i] = new Projection( variableType_, globalTypeID+i, "" ); break;
        }
    }
}


void Expression::copy(const Expression &rhs)
{
	nRows = rhs.nRows;
	nCols = rhs.nCols;
	dim = rhs.dim;
	variableType = rhs.variableType;
	component = rhs.component;

	uint i;
	element = (Operator**) calloc(dim, sizeof(Operator*));

	for (i = 0; i < dim; i++)
	{
		if (rhs.element[i] != 0)
			element[i] = rhs.element[i]->clone();
		else
			element[i] = 0;
	}

	// Name not copied?
}

Expression& Expression::assignmentSetup(const Expression &arg)
{
	deleteAll();
	nRows = arg.getNumRows();
	nCols = arg.getNumCols();
	dim = nRows * nCols;
	variableType = VT_INTERMEDIATE_STATE;

	element = (Operator**) calloc(dim, sizeof(Operator*));

	VariableType tt = VT_UNKNOWN;
	int comp = 0;

	for (uint i = 0; i < dim; i++)
	{
		arg.element[i]->isVariable(tt, comp);
		if (tt == VT_INTERMEDIATE_STATE)
			element[i] = arg.element[i]->clone();
		else
		{
			std::stringstream tmpName;
			if (name.empty() == false)
			{
				if (dim > 1)
					tmpName << name << "[" << i << "]";
				else
					tmpName << name;
			}
			TreeProjection tmp(tmpName.str());
			tmp.operator=(*(arg.element[i]));
			element[i] = tmp.clone();
		}
	}
	return *this;
}


void Expression::deleteAll( ){

    uint i;

    for( i = 0; i < dim; i++ )
        if( element[i] != 0 )
            delete element[i];

    if( element != 0 ) free(element);
}




BooleanType Expression::isDependingOn( VariableType type ) const{

    uint run1;

    for( run1 = 0; run1 < getDim(); run1++ )
        if( element[run1]->isDependingOn(type) == BT_TRUE )
            return BT_TRUE;

    return BT_FALSE;
}



BooleanType Expression::isDependingOn( const Expression &e ) const{
    ASSERT( e.getDim() ==1 );
    DVector sum=getDependencyPattern(e).sumRow();

    if( fabs(sum(0) - EPS) > 0 ) return BT_TRUE;
    return BT_FALSE;
}

Operator* Expression::getOperatorClone( uint idx ) const{

    ASSERT( idx < getDim() );

    Operator *tmp = element[idx]->passArgument();
    if( tmp == 0 ) tmp = element[idx];

    return tmp->clone();
}


CLOSE_NAMESPACE_ACADO

// end of file.
