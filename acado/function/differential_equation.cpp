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
 *    \file src/function/differential_equation.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *
 */

#include <acado/utils/acado_utils.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function_.hpp>
#include <acado/function/differential_equation.hpp>
#include <acado/symbolic_expression/lyapunov.hpp>


BEGIN_NAMESPACE_ACADO




//
// PUBLIC MEMBER FUNCTIONS:
//

DifferentialEquation::DifferentialEquation( ) : Function( ){

    T1 = 0;
    T2 = 0;
    t1 = 0.0;
    t2 = 1.0;
    setup();
}


DifferentialEquation::DifferentialEquation( const double &tStart, const double &tEnd ){

    T1 = 0;
    T2 = 0;
    t1 = tStart;
    t2 = tEnd;
    setup();
}


DifferentialEquation::DifferentialEquation( const double &tStart, const Parameter &tEnd ){

    ASSERT( tEnd.getDim() == 1 );

    T1 = 0;
    T2 = new Parameter(tEnd);
    t1 = tStart;
    t2 = 0.0;
    setup();
}


DifferentialEquation::DifferentialEquation( const Parameter &tStart, const double &tEnd ){

    ASSERT( tStart.getDim() == 1 );

    T1 = new Parameter(tStart);
    T2 = 0;
    t1 = 0.0;
    t2 = tEnd;
    setup();
}


DifferentialEquation::DifferentialEquation( const Parameter &tStart, const Parameter &tEnd ){

    ASSERT( tStart.getDim() == 1 );
    ASSERT( tEnd.getDim() == 1 );

    T1 = new Parameter(tStart);
    T2 = new Parameter(tEnd);
    t1 = 0.0;
    t2 = 0.0;
    setup();
}


void DifferentialEquation::setup(){

    det             = DET_UNKNOWN;
    is_implicit     = BT_FALSE;
	is_discretized  = BT_FALSE;

    counter   = 0;
    component = 0;
	
	stepLength = -INFTY;
}


void DifferentialEquation::copy( const DifferentialEquation &arg ){

    det            = arg.det        ;
    is_implicit    = arg.is_implicit;
	is_discretized = arg.is_discretized;

    lyap = arg.lyap;
    counter = arg.counter;

    if( arg.component == 0 ){
          component = 0;
    }
    else{
        int run1;
        component = (int*)calloc(counter,sizeof(int));
        for( run1 = 0; run1 < counter; run1++ ){
            component[run1] = arg.component[run1];
        }
    }

    if( arg.T1 != 0 ) T1 = new Parameter(*arg.T1);
    else              T1 = 0;

    if( arg.T2 != 0 ) T2 = new Parameter(*arg.T2);
    else              T2 = 0;

    t1 = arg.t1;
    t2 = arg.t2;
	
	stepLength = arg.stepLength;
}



DifferentialEquation::DifferentialEquation( const DifferentialEquation& arg )
                     :Function( arg ){

    copy( arg );
}



DifferentialEquation::~DifferentialEquation( ){

    if( component != 0 )
        free(component);

    if( T1 != 0 ) delete T1;
    if( T2 != 0 ) delete T2;
}



DifferentialEquation& DifferentialEquation::operator=( const DifferentialEquation& arg ){

    if ( this != &arg ){

        if( component != 0 )
            free(component);

        if( T1 != 0 ) delete T1;
        if( T2 != 0 ) delete T2;

        Function::operator=( arg );

        copy( arg );
    }
    return *this;
}


DifferentialEquation* DifferentialEquation::clone() const{

    return new DifferentialEquation(*this);
}



DifferentialEquation& DifferentialEquation::operator<<( const Expression& arg ){

    uint run1;

    if( arg.getVariableType() == VT_DDIFFERENTIAL_STATE ){

        if( det == DET_DAE ){
            ACADOERROR(RET_INVALID_USE_OF_FUNCTION);
            ASSERT(1 == 0);
            return *this;
        }

        component = (int*)realloc(component,(counter+arg.getDim())*sizeof(int));

        for( run1 = 0; run1 < arg.getDim(); run1++ ){
            counter++;
            component[counter-1] = arg.getComponent(run1);
        }

        return *this;
    }
    addDifferential( arg );
    if( getNDX() > 0 ) is_implicit = BT_TRUE;
    return *this;
}


DifferentialEquation& DifferentialEquation::operator<<( const int& arg ){

    if( arg != 0 ){
        ACADOWARNING(RET_INVALID_USE_OF_FUNCTION);
        return *this;
    }

    det = DET_DAE;
    return *this;
}


DifferentialEquation& DifferentialEquation::addDifferential( const Expression& arg ){

    if( T1 == 0 && T2 == 0 ){
       Function::operator<<(arg);
    }
    if( T1 != 0 && T2 == 0 ){
        Function::operator<<( (t2-T1[0])*arg );
    }
    if( T1 == 0 && T2 != 0 ){
        Function::operator<<( (T2[0]-t1)*arg );
    }
    if( T1 != 0 && T2 != 0 ){
        Function::operator<<( (T2[0]-T1[0])*arg );
    }

    if( arg.isDependingOn( VT_DDIFFERENTIAL_STATE ) == BT_TRUE ){

        det = DET_DAE;
    }
    else{
        if( det != DET_DAE )
            det = DET_ODE;
    }

    return *this;
}



DifferentialEquation& DifferentialEquation::addDifferential( const double &arg ){

    Expression tmp;
    tmp = arg;
    return addDifferential( tmp );
}


DifferentialEquation& DifferentialEquation::addAlgebraic( const Expression& arg ){

    Function::operator<<(arg);
    det = DET_DAE;
    return *this;
}


DifferentialEquation& DifferentialEquation::addAlgebraic( const double &arg ){

    det = DET_DAE;
    if( fabs(arg) < EPS ) return *this;
    ACADOWARNING( RET_INFEASIBLE_ALGEBRAIC_CONSTRAINT );
    return *this;
}


DifferentialEquation& DifferentialEquation::addDifferential( const DVector& arg ){

    uint run1;

    for( run1 = 0; run1 < arg.getDim(); run1++ )
        addDifferential( arg(run1) );

    return *this;
}


DifferentialEquation& DifferentialEquation::addAlgebraic( const DVector& arg ){

    uint run1;

    for( run1 = 0; run1 < arg.getDim(); run1++ )
        addAlgebraic( arg(run1) );

    return *this;
}


DifferentialEquation& DifferentialEquation::addDifferential( const DMatrix& arg ){

    uint run1, run2;

    for( run1 = 0; run1 < arg.getNumRows(); run1++ )
        for( run2 = 0; run2 < arg.getNumCols(); run2++ )
            addDifferential( arg.operator()(run1,run2) );

    return *this;
}


DifferentialEquation& DifferentialEquation::addAlgebraic( const DMatrix& arg ){

    uint run1, run2;

    for( run1 = 0; run1 < arg.getNumRows(); run1++ )
        for( run2 = 0; run2 < arg.getNumCols(); run2++ )
            addAlgebraic( arg.operator()(run1,run2) );

    return *this;
}


DifferentialEquation& DifferentialEquation::operator==(const Lyapunov& arg ){

  lyap = arg;

  return operator==( arg.A*arg.P+arg.P*(arg.A).transpose()+(arg.B)*(arg.B).transpose() );
}



Lyapunov DifferentialEquation::getLyapunovObject( ) const{
 
  return lyap;
}

BooleanType DifferentialEquation::hasLyapunovEquation( ) const
{
    if( lyap.isEmpty() == BT_TRUE ) return BT_FALSE;
    return BT_TRUE;
    }



DifferentialEquation& DifferentialEquation::operator==( const Expression& arg ){

    if( det == DET_DAE ) return addAlgebraic(arg);
    return addDifferential(arg);
}


DifferentialEquation& DifferentialEquation::operator==( const double &arg ){

    if( det == DET_DAE ) return addAlgebraic(arg);
    return addDifferential(arg);
}


DifferentialEquation& DifferentialEquation::operator==( const DVector& arg ){

    if( det == DET_DAE ) return addAlgebraic(arg);
    return addDifferential(arg);
}


DifferentialEquation& DifferentialEquation::operator==( const DMatrix& arg ){

    if( det == DET_DAE ) return addAlgebraic(arg);
    return addDifferential(arg);
}



BooleanType DifferentialEquation::isDAE( ) const
{
    if( getNumAlgebraicEquations() > 0 )  return BT_TRUE;
    return BT_FALSE;
}


BooleanType DifferentialEquation::isODE( ) const
{
    if( getNumAlgebraicEquations() == 0 )  return BT_TRUE;
    return BT_FALSE;
}


Expression DifferentialEquation::getODEexpansion( const int &order ) const{

	if ( order < 0 ){ ACADOERROR( RET_INDEX_OUT_OF_BOUNDS ); return 0; }
	
	Expression rhs;
	getExpression(rhs);
	
	return rhs.getODEexpansion( order, component );
}



BooleanType DifferentialEquation::makeImplicit(){

    if ( is_implicit == BT_FALSE ){

        evaluationTree.makeImplicit( getDim() - getNumAlgebraicEquations() );
        is_implicit = BT_TRUE;

        return BT_TRUE;
    }
    return BT_FALSE;
}


BooleanType DifferentialEquation::isDiscretized( ) const{

    return is_discretized;
}


double DifferentialEquation::getStepLength() const{

    return stepLength;
}


CLOSE_NAMESPACE_ACADO

// end of file.
