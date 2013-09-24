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
*    \file src/symbolic_expression/intermediate_state.cpp
*    \author Boris Houska, Hans Joachim Ferreau, Joris Gillis
*    \date 2008
*/


#include <acado/symbolic_expression/intermediate_state.hpp>



BEGIN_NAMESPACE_ACADO


int IntermediateState::count = 0;


IntermediateState::IntermediateState()
                  :Expression( 1, 1, VT_INTERMEDIATE_STATE, count ){

    count++;
}


IntermediateState::IntermediateState( uint nRows_, uint nCols_, String name_ )
                  :Expression( nRows_, nCols_, VT_INTERMEDIATE_STATE, (uint) count, name_ ){

    count += nRows_*nCols_;
}

IntermediateState::IntermediateState( uint nRows_, String name_ )
                  :Expression( nRows_,(uint) 1, VT_INTERMEDIATE_STATE, (uint) count, name_ ){
    count += nRows_;
}

IntermediateState::IntermediateState(String name_ )
                  :Expression((uint) 1,(uint) 1, VT_INTERMEDIATE_STATE, (uint) count, name_ ){

    count ++;
}



IntermediateState::IntermediateState( int nRows_, int nCols_, String name_ )
                  :Expression( nRows_, nCols_, VT_INTERMEDIATE_STATE, count, name_ ){

    count += nRows_*nCols_;
}


IntermediateState::IntermediateState( const double& arg ):Expression(){

    nRows = 1;
    nCols = 1;
    operator=(arg);
}


IntermediateState::IntermediateState( const Vector& arg ):Expression(){

    nRows = arg.getDim();
    nCols = 1;
    operator=(arg);
}


IntermediateState::IntermediateState( const Matrix& arg ):Expression(){

    nRows = arg.getNumRows();
    nCols = arg.getNumCols();
    operator=(arg);
}


IntermediateState::IntermediateState( const Operator& arg ):Expression(){

    nRows = 1;
    nCols = 1;
    operator=( Expression(arg) );
}


IntermediateState::IntermediateState( const Expression& arg ):Expression(){

    nRows = arg.getNumRows();
    nCols = arg.getNumCols();
    operator=(arg);
}



IntermediateState::~IntermediateState(){ }


Expression& IntermediateState::operator= ( const double & arg ){

    return operator= ( convert(arg)  );
}



Expression& IntermediateState::operator= ( const Vector & arg ){ return operator= ( convert(arg)  ); }
Expression& IntermediateState::operator= ( const Matrix & arg ){ return operator= ( convert(arg)  ); }

Expression& IntermediateState::operator=( const Expression &arg ){     nRows = arg.getNumRows();nCols = arg.getNumCols();return assignmentSetup( arg ); }



Expression& IntermediateState::operator+=( const double      & arg ){ return operator=( this->operator+(arg) ); }
Expression& IntermediateState::operator+=( const Vector      & arg ){ return operator=( this->operator+(arg) ); }
Expression& IntermediateState::operator+=( const Matrix      & arg ){ return operator=( this->operator+(arg) ); }
Expression& IntermediateState::operator+=( const Expression  & arg ){ return operator=( this->operator+(arg) ); }

Expression& IntermediateState::operator-=( const double      & arg ){ return operator=( this->operator-(arg) ); }
Expression& IntermediateState::operator-=( const Vector      & arg ){ return operator=( this->operator-(arg) ); }
Expression& IntermediateState::operator-=( const Matrix      & arg ){ return operator=( this->operator-(arg) ); }
Expression& IntermediateState::operator-=( const Expression  & arg ){ return operator=( this->operator-(arg) ); }

Expression& IntermediateState::operator*=( const double      & arg ){ return operator=( this->operator*(arg) ); }
Expression& IntermediateState::operator*=( const Vector      & arg ){ return operator=( this->operator*(arg) ); }
Expression& IntermediateState::operator*=( const Matrix      & arg ){ return operator=( this->operator*(arg) ); }
Expression& IntermediateState::operator*=( const Expression  & arg ){ return operator=( this->operator*(arg) ); }

Expression& IntermediateState::operator/=( const double      & arg ){ return operator=( this->operator/(arg) ); }
Expression& IntermediateState::operator/=( const Expression  & arg ){ return operator=( this->operator/(arg) ); }




Expression* IntermediateState::clone() const{

    return new Expression(*this);
}


returnValue IntermediateState::clearStaticCounters(){

    count = 0;
    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
