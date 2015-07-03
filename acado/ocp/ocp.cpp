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
 *    \file src/ocp/ocp.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Milan Vukov, Rien Quirynen
 *    \date 2008 - 2014
 */

#include <acado/ocp/ocp.hpp>
#include <acado/variables_grid/grid.hpp>
#include <acado/objective/objective.hpp>

using namespace std;
BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//


OCP::OCP(const double &tStart_, const double &tEnd_, const int &N_)
	: MultiObjectiveFunctionality(),
	  grid(new Grid()), objective(new Objective()), constraint(new Constraint())
{

	if (N_ < 0)
		ACADOERROR(RET_INVALID_ARGUMENTS);
	setupGrid(tStart_, tEnd_, N_ + 1);
}


OCP::OCP( const double &tStart_, const double &tEnd_, const DVector& _numSteps )
    : MultiObjectiveFunctionality(),
      grid(new Grid()), objective(new Objective()), constraint(new Constraint())
{
	if( _numSteps.getDim() <= 0 ) ACADOERROR( RET_INVALID_ARGUMENTS );
      
	DVector times( _numSteps.getDim()+1 );
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
    modelData.setIntegrationGrid(*grid, totalSteps);
}


OCP::OCP( const Grid &grid_ )
    : MultiObjectiveFunctionality(),
      grid(new Grid(grid_)), objective(new Objective()), constraint(new Constraint())
{
    if( grid->getNumPoints() <= 1 ) ACADOERROR( RET_INVALID_ARGUMENTS );
    objective->init ( *grid );
    constraint->init( *grid );
    setN( grid->getNumIntervals() );
}


OCP::OCP( const double    &tStart_,
          const Parameter &tEnd_  ,
          const int       &N_       )
    : MultiObjectiveFunctionality(),
      grid(new Grid()), objective(new Objective()), constraint(new Constraint())
{
    if( N_ <= 0 ) ACADOERROR( RET_INVALID_ARGUMENTS );
    setupGrid( tStart_, tStart_ + 1.0, N_+1);
}

OCP::~OCP( )
{}

returnValue OCP::minimizeMayerTerm( const int &multiObjectiveIdx,  const Expression& arg ){

    return MultiObjectiveFunctionality::minimizeMayerTerm( multiObjectiveIdx, arg );
}


returnValue OCP::minimizeLSQ( const MatrixVariablesGrid &S,
                              const Function            &h,
                              const char*        rFilename )
{

    VariablesGrid r;
    r.read( rFilename );

    if( r.isEmpty() == BT_TRUE )
        return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

    return minimizeLSQ( S,h,r );
}


returnValue OCP::minimizeLSQ( const DMatrix        &S,
                              const Function      &h,
                              const char*  rFilename  ){

    VariablesGrid r;
    r.read( rFilename );

    if( r.isEmpty() == BT_TRUE )
        return ACADOERROR( RET_FILE_CAN_NOT_BE_OPENED );

    return minimizeLSQ( S,h,r );
}


returnValue OCP::minimizeLSQ( const Function      &h,
                              const char*  rFilename  ){

    VariablesGrid r;
    r.read( rFilename );

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
        constraint->add( component(i) );

    return SUCCESSFUL_RETURN;
}


returnValue OCP::subjectTo( int index_, const ConstraintComponent& component )
{
	ASSERT(index_ >= AT_START);

	if (index_ == AT_START)
	{
		for (unsigned el = 0; el < component.getDim(); ++el)
			ACADO_TRY( constraint->add( 0,component( el ) ) );
	}
	else if (index_ == AT_END)
	{
		for (unsigned el = 0; el < component.getDim(); ++el)
			ACADO_TRY(constraint->add(grid->getLastIndex(), component( el )));
	}
	else
	{
		for (unsigned el = 0; el < component.getDim(); ++el)
			constraint->add(index_, component(el));
	}

    return SUCCESSFUL_RETURN;
}


returnValue OCP::subjectTo( const double lb_, const Expression& arg1,
							const Expression& arg2, const double ub_ ){

    return constraint->add( lb_, arg1, arg2, ub_ );
}


returnValue OCP::subjectTo( const double lb_, const Expression *arguments, const double ub_ )
{
    return constraint->add( lb_, arguments, ub_ );
}

returnValue OCP::subjectTo( const DVector& _lb, const Expression& _expr, const DVector& _ub )
{
	ASSERT(_lb.getDim() == _expr.getDim() && _lb.getDim() == _ub.getDim());
	constraint->add(_lb, _expr, _ub);

	return SUCCESSFUL_RETURN;
}

returnValue OCP::subjectTo( int _index, const DVector& _lb, const Expression& _expr, const DVector& _ub )
{
	ASSERT(_index >= AT_START);
	cout << _lb.getDim() << " " << _expr.getDim() << endl;
	ASSERT(_lb.getDim() == _expr.getDim());
	ASSERT(_lb.getDim() == _ub.getDim());

	if (_index == AT_START)
	{
		for (unsigned el = 0; el < _lb.getDim(); ++el)
			ACADO_TRY( constraint->add(0, _lb( el ), _expr( el ), _ub( el )) );
	}
	else if (_index == AT_END)
	{
		for (unsigned el = 0; el < _lb.getDim(); ++el)
			ACADO_TRY(constraint->add(grid->getLastIndex(), _lb( el ), _expr( el ), _ub( el )) );
	}
	else
		for (unsigned el = 0; el < _lb.getDim(); ++el)
			constraint->add(_index, _lb( el ), _expr( el ), _ub( el ));

	return SUCCESSFUL_RETURN;
}

returnValue OCP::minimizeMayerTerm   ( const Expression& arg ){ return objective->addMayerTerm   ( arg ); }
returnValue OCP::maximizeMayerTerm   ( const Expression& arg ){ return objective->addMayerTerm   (-arg ); }
returnValue OCP::minimizeLagrangeTerm( const Expression& arg ){ return objective->addLagrangeTerm( arg ); }
returnValue OCP::maximizeLagrangeTerm( const Expression& arg ){ return objective->addLagrangeTerm(-arg ); }


returnValue OCP::minimizeLSQ( const DMatrix&S, const Function &h, const DVector &r )
{
    MatrixVariablesGrid tmpS(S);
    VariablesGrid       tmpR(r);

    return objective->addLSQ( &tmpS, h, &tmpR );
}

returnValue OCP::minimizeLSQ( const Function &h, const DVector &r ){

    DMatrix S( h.getDim( ),h.getDim( ) );
    S.setIdentity( );

    return minimizeLSQ( S, h, r );
}

returnValue OCP::minimizeLSQ( const Function &h ){

    DMatrix S( h.getDim( ),h.getDim( ) );
    S.setIdentity( );

    DVector r(h.getDim());
    r.setZero();

    return minimizeLSQ( S, h, r );
}


returnValue OCP::minimizeLSQ( const MatrixVariablesGrid &S,
                              const Function            &h,
                              const VariablesGrid       &r  ){

    return objective->addLSQ( &S, h, &r );
}

returnValue OCP::minimizeLSQ( const DMatrix        &S,
                              const Function      &h,
                              const VariablesGrid &r ){

	if ( S.isPositiveSemiDefinite() == BT_FALSE )
		return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

    MatrixVariablesGrid tmpS(S);
    return objective->addLSQ( &tmpS, h, &r );
}

returnValue OCP::minimizeLSQ( const Function      &h,
                              const VariablesGrid &r ){

    return objective->addLSQ( 0, h, &r );
}


returnValue OCP::minimizeLSQEndTerm( const DMatrix   & S,
                                     const Function & m,
                                     const DVector   & r  ){

	if ( S.isPositiveSemiDefinite() == BT_FALSE )
		return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

    return objective->addLSQEndTerm( S, m, r );
}

returnValue OCP::minimizeLSQEndTerm( const Function & m,
                                     const DVector   & r  ){

    DMatrix S( m.getDim( ),m.getDim( ) );
    S.setIdentity( );
    return minimizeLSQEndTerm( S, m, r );
}


BooleanType OCP::hasObjective() const{

    if( objective->isEmpty() == BT_TRUE ) return BT_FALSE;
    return BT_TRUE;
}

BooleanType OCP::hasConstraint() const{

    if( constraint->isEmpty() == BT_TRUE ) return BT_FALSE;
    return BT_TRUE;
}


returnValue OCP::getGrid     ( Grid       & grid_      ) const{ grid_       = *grid      ; return SUCCESSFUL_RETURN; }
returnValue OCP::getObjective( Objective  & objective_ ) const{ objective_  = *objective ; return SUCCESSFUL_RETURN; }
returnValue OCP::getConstraint( Constraint& constraint_) const{ constraint_ = *constraint; return SUCCESSFUL_RETURN; }


returnValue OCP::setObjective ( const Objective & objective_  )
{
	objective   = std::shared_ptr<Objective> (new Objective( objective_ ));
	return SUCCESSFUL_RETURN;
}
returnValue OCP::setConstraint( const Constraint& constraint_ )
{
	constraint = std::shared_ptr<Constraint> (new Constraint( constraint_ ));;
	return SUCCESSFUL_RETURN;
}


returnValue OCP::setNumberIntegrationSteps( const uint numSteps )
{
	if( hasEquidistantControlGrid() ) {
		setIntegrationGrid( *grid, numSteps );
	}
	return SUCCESSFUL_RETURN;
}


returnValue OCP::getObjective( const int &multiObjectiveIdx, Expression **arg ) const{

    return MultiObjectiveFunctionality::getObjective( multiObjectiveIdx, arg );
}


double OCP::getStartTime ( ) const{ return grid->getFirstTime(); }
double OCP::getEndTime   ( ) const{ return grid->getLastTime (); }

BooleanType OCP::hasEquidistantGrid( ) const{
	
	DVector numSteps;
	modelData.getNumSteps(numSteps);
	return numSteps.isEmpty();
}

returnValue OCP::minimizeLSQ(const DMatrix& S, const Function& h)
{
	return objective->addLSQ(S, h);
}

returnValue OCP::minimizeLSQEndTerm(const DMatrix& S, const Function& h)
{
	return objective->addLSQEndTerm(S, h);
}

returnValue OCP::minimizeLSQ(const BMatrix& S, const Function& h)
{
	return objective->addLSQ(S, h);
}

returnValue OCP::minimizeLSQEndTerm(const BMatrix& S, const Function& h)
{
	return objective->addLSQEndTerm(S, h);
}

returnValue OCP::minimizeLSQ(const DMatrix& S, const std::string& h)
{
	return objective->addLSQ(S, h);
}

returnValue OCP::minimizeLSQEndTerm(const DMatrix& S, const std::string& h)
{
	return objective->addLSQEndTerm(S, h);
}

returnValue OCP::minimizeLSQ(const BMatrix& S, const std::string& h)
{
	return objective->addLSQ(S, h);
}

returnValue OCP::minimizeLSQEndTerm(const BMatrix& S, const std::string& h)
{
	return objective->addLSQEndTerm(S, h);
}

returnValue OCP::minimizeLSQLinearTerms(const DVector& Slx, const DVector& Slu)
{
	return objective->addLSQLinearTerms(Slx, Slu);
}

returnValue OCP::minimizeLSQLinearTerms(const BVector& Slx, const BVector& Slu)
{
	return objective->addLSQLinearTerms(Slx, Slu);
}

// PROTECTED FUNCTIONS:
// --------------------
void OCP::setupGrid( double tStart, double tEnd, int N ){

    grid->init( tStart, tEnd, N );
    objective->init ( *grid );
    constraint->init( *grid );
    setN( grid->getNumIntervals() );
}


void OCP::setupGrid( const DVector& times ){

    grid->init( times );
    objective->init ( *grid );
    constraint->init( *grid );
    setN( grid->getNumIntervals() );
}



CLOSE_NAMESPACE_ACADO

// end of file.
