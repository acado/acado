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
 *    \file src/ocp/ocp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Milan Vukov, Rien Quirynen
 *    \date 2008-2013
 */

#include <acado/ocp/ocp.hpp>
#include <acado/code_generation/export_variable.hpp>


BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


OCP::OCP( const double &tStart_, const double &tEnd_, const int &N_ )
    :MultiObjectiveFunctionality(){

    if( N_ < 0 ) ACADOERROR( RET_INVALID_ARGUMENTS );
    setupGrid( tStart_, tEnd_, N_+1 );
}


OCP::OCP( const double &tStart_, const double &tEnd_, const Vector& _numSteps )
    :MultiObjectiveFunctionality(){

        if( _numSteps.getDim() <= 0 ) ACADOERROR( RET_INVALID_ARGUMENTS );
      
	Vector times( _numSteps.getDim()+1 );
	times(0) = tStart_;
	
	double totalSteps = 0;
	uint i;
	for( i = 0; i < _numSteps.getDim(); i++ ) {
		totalSteps += _numSteps(i);
	}
	double h = (tEnd_ - tStart_)/totalSteps;
	for( i = 0; i < _numSteps.getDim(); i++ ) {
		times(i+1) = times(i) + h*_numSteps(i);
	}
	
    setupGrid( times );
    modelData.setIntegrationGrid(grid, totalSteps);
}


OCP::OCP( const Grid &grid_ )
    :MultiObjectiveFunctionality(){

    if( grid_.getNumPoints() <= 1 ) ACADOERROR( RET_INVALID_ARGUMENTS );
    grid = grid_;
    objective.init ( grid );
    constraint.init( grid );
    setN( grid.getNumIntervals() );
}


OCP::OCP( const double    &tStart_,
          const Parameter &tEnd_  ,
          const int       &N_       )
    :MultiObjectiveFunctionality(){

    if( N_ <= 0 ) ACADOERROR( RET_INVALID_ARGUMENTS );
    setupGrid( tStart_, tStart_ + 1.0, N_+1);
}


void OCP::copy( const OCP &rhs )
{
    grid                 = rhs.grid                ;
    modelData			 = rhs.modelData		   ;
    objective            = rhs.objective           ;
    constraint           = rhs.constraint          ;
}

OCP::OCP( const OCP& rhs )
    :MultiObjectiveFunctionality( rhs ){

    copy( rhs );
}


OCP::~OCP( ){ 
}


OCP& OCP::operator=( const OCP& rhs ){

    if ( this != &rhs ){

        MultiObjectiveFunctionality::operator=(rhs);
        copy(rhs);
    }
    return *this;
}



returnValue OCP::minimizeMayerTerm( const int &multiObjectiveIdx,  const Expression& arg ){

    return MultiObjectiveFunctionality::minimizeMayerTerm( multiObjectiveIdx, arg );
}


returnValue OCP::minimizeLSQ( const MatrixVariablesGrid &S,
                              const Function            &h,
                              const char*        rFilename ){

    VariablesGrid r = readFromFile( rFilename );

    if( r.isEmpty() == BT_TRUE )
        return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

    return minimizeLSQ( S,h,r );
}


returnValue OCP::minimizeLSQ( const Matrix        &S,
                              const Function      &h,
                              const char*  rFilename  ){

    VariablesGrid r = readFromFile( rFilename );

    if( r.isEmpty() == BT_TRUE )
        return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

    return minimizeLSQ( S,h,r );
}


returnValue OCP::minimizeLSQ( const Function      &h,
                              const char*  rFilename  ){

    VariablesGrid r = readFromFile( rFilename );

    if( r.isEmpty() == BT_TRUE )
        return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

    return minimizeLSQ( h,r );
}



returnValue OCP::subjectTo( const DifferentialEquation& differentialEquation_ ){

    modelData.setModel(differentialEquation_);
    return SUCCESSFUL_RETURN;
}



returnValue OCP::subjectTo( const ConstraintComponent& component ){

    for( uint i=0; i<component.getDim(); ++i )
        constraint.add( component(i) );

    return SUCCESSFUL_RETURN;
}


returnValue OCP::subjectTo( const int index_, const ConstraintComponent& component ){

    for( uint i=0; i<component.getDim(); ++i )
        constraint.add( index_,component(i) );

    return SUCCESSFUL_RETURN;
}


returnValue OCP::subjectTo( const TimeHorizonElement index_, const ConstraintComponent& component ){

    uint i;

    switch( index_ ){

        case AT_START:
             for( i = 0; i < component.getDim(); i++ )
                 ACADO_TRY( constraint.add( 0,component(i) ) );
             return SUCCESSFUL_RETURN;

        case AT_END:
             for( i = 0; i < component.getDim(); i++ )
                 ACADO_TRY( constraint.add( grid.getLastIndex(),component(i) ) );
             return SUCCESSFUL_RETURN;

        default:
             return ACADOERROR(RET_UNKNOWN_BUG);
    }
    return SUCCESSFUL_RETURN;
}


returnValue OCP::subjectTo( const double lb_, const Expression& arg1,
                                   const Expression& arg2, const double ub_ ){

    return constraint.add( lb_, arg1, arg2, ub_ );
}


returnValue OCP::subjectTo( const double lb_, const Expression *arguments, const double ub_ )
{
    return constraint.add( lb_, arguments, ub_ );
}

returnValue OCP::minimizeMayerTerm   ( const Expression& arg ){ return objective.addMayerTerm   ( arg ); }
returnValue OCP::maximizeMayerTerm   ( const Expression& arg ){ return objective.addMayerTerm   (-arg ); }
returnValue OCP::minimizeLagrangeTerm( const Expression& arg ){ return objective.addLagrangeTerm( arg ); }
returnValue OCP::maximizeLagrangeTerm( const Expression& arg ){ return objective.addLagrangeTerm(-arg ); }


returnValue OCP::minimizeLSQ( const Matrix&S, const Function &h, const Vector &r ){

	if ( S.isPositiveSemiDefinite() == BT_FALSE )
		return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

    MatrixVariablesGrid tmpS(S);
    VariablesGrid       tmpR(r);

    return objective.addLSQ( &tmpS, h, &tmpR );
}

returnValue OCP::minimizeLSQ( const Function &h, const Vector &r ){

    Matrix S( h.getDim( ),h.getDim( ) );
    S.setIdentity( );

    return minimizeLSQ( S, h, r );
}

returnValue OCP::minimizeLSQ( const Function &h ){

    Matrix S( h.getDim( ),h.getDim( ) );
    S.setIdentity( );

    Vector r(h.getDim());
    r.setZero();

    return minimizeLSQ( S, h, r );
}


returnValue OCP::minimizeLSQ( const MatrixVariablesGrid &S,
                              const Function            &h,
                              const VariablesGrid       &r  ){

    return objective.addLSQ( &S, h, &r );
}

returnValue OCP::minimizeLSQ( const Matrix        &S,
                              const Function      &h,
                              const VariablesGrid &r ){

	if ( S.isPositiveSemiDefinite() == BT_FALSE )
		return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

    MatrixVariablesGrid tmpS(S);
    return objective.addLSQ( &tmpS, h, &r );
}

returnValue OCP::minimizeLSQ( const Function      &h,
                              const VariablesGrid &r ){

    return objective.addLSQ( 0, h, &r );
}


returnValue OCP::minimizeLSQEndTerm( const Matrix   & S,
                                     const Function & m,
                                     const Vector   & r  ){

	if ( S.isPositiveSemiDefinite() == BT_FALSE )
		return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

    return objective.addLSQEndTerm( S, m, r );
}

returnValue OCP::minimizeLSQEndTerm( const Function & m,
                                     const Vector   & r  ){

    Matrix S( m.getDim( ),m.getDim( ) );
    S.setIdentity( );
    return minimizeLSQEndTerm( S, m, r );
}


BooleanType OCP::hasObjective() const{

    if( objective.isEmpty() == BT_TRUE ) return BT_FALSE;
    return BT_TRUE;
}

BooleanType OCP::hasConstraint() const{

    if( constraint.isEmpty() == BT_TRUE ) return BT_FALSE;
    return BT_TRUE;
}


returnValue OCP::getGrid     ( Grid       & grid_      ) const{ grid_       = grid      ; return SUCCESSFUL_RETURN; }
returnValue OCP::getObjective( Objective  & objective_ ) const{ objective_  = objective ; return SUCCESSFUL_RETURN; }
returnValue OCP::getConstraint( Constraint& constraint_) const{ constraint_ = constraint; return SUCCESSFUL_RETURN; }


returnValue OCP::setObjective ( const Objective & objective_  ){ objective   = objective_; return SUCCESSFUL_RETURN; }
returnValue OCP::setConstraint( const Constraint& constraint_ ){ constraint = constraint_; return SUCCESSFUL_RETURN; }


returnValue OCP::setNumberIntegrationSteps( const uint numSteps )
{
	if( hasEquidistantControlGrid() ) {
		setIntegrationGrid( grid, numSteps );
	}
	return SUCCESSFUL_RETURN;
}


returnValue OCP::getObjective( const int &multiObjectiveIdx, Expression **arg ) const{

    return MultiObjectiveFunctionality::getObjective( multiObjectiveIdx, arg );
}


double OCP::getStartTime ( ) const{ return grid.getFirstTime(); }
double OCP::getEndTime   ( ) const{ return grid.getLastTime (); }

BooleanType OCP::hasEquidistantGrid( ) const{
	
	Vector numSteps;
	modelData.getNumSteps(numSteps);
	return numSteps.isEmpty();
}

returnValue OCP::minimizeLSQ(const ExportVariable& S, const Function& h)
{
	return objective.addLSQ(S, h);
}

returnValue OCP::minimizeLSQEndTerm(const ExportVariable& S, const Function& h)
{
	return objective.addLSQEndTerm(S, h);
}

returnValue OCP::minimizeLSQ(const ExportVariable& S, const String& h)
{
	return objective.addLSQ(S, h);
}

returnValue OCP::minimizeLSQEndTerm(const ExportVariable& S, const String& h)
{
	return objective.addLSQEndTerm(S, h);
}

returnValue OCP::minimizeLSQLinearTerms(const Vector& Slx, const Vector& Slu)
{
	return objective.addLSQLinearTerms(Slx, Slu);
}

returnValue OCP::minimizeLSQLinearTerms(const ExportVariable& Slx, const ExportVariable& Slu)
{
	return objective.addLSQLinearTerms(Slx, Slu);
}



// PROTECTED FUNCTIONS:
// --------------------
void OCP::setupGrid( double tStart, double tEnd, int N ){

    grid.init( tStart, tEnd, N );
    objective.init ( grid );
    constraint.init( grid );
    setN( grid.getNumIntervals() );
}


void OCP::setupGrid( const Vector& times ){

    grid.init( times );
    objective.init ( grid );
    constraint.init( grid );
    setN( grid.getNumIntervals() );
}



CLOSE_NAMESPACE_ACADO

// end of file.
