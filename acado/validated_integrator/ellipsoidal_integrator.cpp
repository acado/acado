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
 *    \file src/validated_integrator/ellipsoidal_integrator.cpp
 *    \author Boris Houska, Mario Villanueva, Benoit Chachuat
 *    \date 2013
 */

#include <acado/validated_integrator/ellipsoidal_integrator.hpp>


BEGIN_NAMESPACE_ACADO



// IMPLEMENTATION OF PUBLIC MEMBER FUNCTIONS:
// ------------------------------------------

EllipsoidalIntegrator::EllipsoidalIntegrator():AlgorithmicBase(){ setupOptions(); }

EllipsoidalIntegrator::EllipsoidalIntegrator( const DifferentialEquation &rhs_, const int &N_ ):AlgorithmicBase(){

    ASSERT( rhs_.isODE( ) == BT_TRUE );
	setupOptions();
    init( rhs_, N_ );
}

EllipsoidalIntegrator::EllipsoidalIntegrator( const EllipsoidalIntegrator& arg ):AlgorithmicBase( arg ){ copy(arg); }

EllipsoidalIntegrator::~EllipsoidalIntegrator( ){ }

EllipsoidalIntegrator& EllipsoidalIntegrator::operator=( const EllipsoidalIntegrator& arg ){
  
	if( this != &arg ){
		AlgorithmicBase::operator=(arg);
		copy(arg);
	}
	return *this;
}



returnValue EllipsoidalIntegrator::init( const DifferentialEquation &rhs_, const int &N_ ){
 
  if( rhs_.isODE( ) == BT_FALSE ) return ACADOERROR(RET_CANNOT_TREAT_DAE);
  
  nx = rhs_.getDim();
  
  Q.resize(nx,nx);
  for( int i=0; i<nx*nx; i++ ) Q(i) = 0.0;
  N = N_;
  
  IntermediateState derivatives = rhs_.getODEexpansion( N );
  
  IntermediateState gg = derivatives.getCol(0);
  for( int i=0; i<N; i++ ) gg << derivatives.getCol(i+1)/acadoFactorial(i+1);
  
  g  << gg;
  
//   FILE *file = fopen("g.c","w");
//   file << g;
//   fclose(file);
  
  gr << derivatives.getCol(N+1)/acadoFactorial(N+1);
  dg << gg.ADforward( VT_DIFFERENTIAL_STATE, rhs_.getComponents(), nx );
  
  DifferentialState r("", nx, 1);
  
  IntermediateState dgg = gg.ADforward( VT_DIFFERENTIAL_STATE, rhs_.getComponents(), r );
  ddg << 0.5*dgg.ADforward( VT_DIFFERENTIAL_STATE, rhs_.getComponents(), r );
  
  return SUCCESSFUL_RETURN;
}



Tmatrix<Interval> EllipsoidalIntegrator::integrate( double t0, double tf, int M,
													const Tmatrix<Interval> &x ){

	Tmatrix<Interval> p(0);
	Tmatrix<Interval> w(0);
	return integrate( t0, tf, M, x, p, w );
}

Tmatrix<Interval> EllipsoidalIntegrator::integrate( double t0, double tf, int M,
													const Tmatrix<Interval> &x,
													const Tmatrix<Interval> &p ){

	Tmatrix<Interval> w(0);
	return integrate( t0, tf, M, x, p, w );
}

Tmatrix<Interval> EllipsoidalIntegrator::integrate( double t0, double tf, int M,
													const Tmatrix<Interval> &x,
													const Tmatrix<Interval> &p,
													const Tmatrix<Interval> &w ){

  
	typedef TaylorVariable<Interval> T;
	
	Tmatrix< TaylorVariable<Interval> > xx(x.getDim());
	
	Tmatrix<T> *pp = 0;
	Tmatrix<T> *ww = 0;
	
	if( p.getDim() > 0 ) pp = new Tmatrix<T>(p.getDim());
	if( w.getDim() > 0 ) ww = new Tmatrix<T>(w.getDim());
	
	int nn = 0;
	for( int i=0; i<(int) x.getDim(); i++ ) if( diam(x(i)) > EQUALITY_EPS ) nn++;
	for( int i=0; i<(int) p.getDim(); i++ ) if( diam(p(i)) > EQUALITY_EPS ) nn++;
	for( int i=0; i<(int) w.getDim(); i++ ) if( diam(w(i)) > EQUALITY_EPS ) nn++;
	
	TaylorModel<Interval> Mod( nn, M );
	
	nn = 0;
	for( int i=0; i<(int) x.getDim(); i++ ){
		if( diam(x(i)) > EQUALITY_EPS ){ xx(i) = T( &Mod, nn, x(i) ); nn++; }
		else xx(i) = x(i);
	}
	for( int i=0; i<(int) p.getDim(); i++ ){
		if( diam(p(i)) > EQUALITY_EPS ){ pp->operator()(i) = T( &Mod, nn, p(i) ); nn++; }
		else pp->operator()(i) = p(i);
	}
	for( int i=0; i<(int) w.getDim(); i++ ){
		if( diam(w(i)) > EQUALITY_EPS ){ ww->operator()(i) = T( &Mod, nn, w(i) ); nn++; }
		else ww->operator()(i) = w(i);
	}
	
	integrate( t0, tf, &xx, pp, ww );
	
	return getStateBound( xx );
}



// IMPLEMENTATION OF PRIVATE MEMBER FUNCTIONS:
// ======================================================================================


void EllipsoidalIntegrator::copy( const EllipsoidalIntegrator& arg ){
  
	nx  = arg.nx ;
	N   = arg.N  ;
	g   = arg.g  ;
	gr  = arg.gr ;
	dg  = arg.dg ;
	ddg = arg.ddg;
	Q   = arg.Q  ;
}


Tmatrix<Interval> EllipsoidalIntegrator::evalC( const Tmatrix<double> &C, double h ) const{

	Tmatrix<Interval> r(nx,nx);
	
	for( int i=0; i<nx*nx; i++ ){
	  
		double r_i;
		r_i = C(i);
		for( int j=0; j<N; j++ ) r_i += ::pow(h,j+1)*C(i+(j+1)*nx*nx);
		r(i) = r_i;
	}
	return r;
}


Tmatrix<double> EllipsoidalIntegrator::evalC2( const Tmatrix<double> &C, double h ) const{

	Tmatrix<double> r(nx,nx);
	
	for( int i=0; i<nx*nx; i++ ){
	  
		double r_i;
		r_i = C(i);
		for( int j=0; j<N; j++ ) r_i += ::pow(h,j+1)*C(i+(j+1)*nx*nx);
		r(i) = r_i;
	}
	return r;
}

double EllipsoidalIntegrator::scale( const Interval &E, const Interval &X ) const{

    double TOL,ATOL;
    get( INTEGRATOR_TOLERANCE, TOL  );
    get( ABSOLUTE_TOLERANCE  , ATOL );

    return 0.5*acadoMax( ::fabs(E.l()), ::fabs(E.u()) )/
           ( TOL*acadoMax( ::fabs(X.l()), ::fabs(X.u()) ) + ATOL );
}


double EllipsoidalIntegrator::norm ( const Tmatrix<Interval> &E, Tmatrix<Interval> &X ) const{
  
    ASSERT( E.getDim() == X.getDim() );

    double sum = 0.0;
	
	for( int i=0; i < (int) E.getDim(); i++ ) sum += scale(E(i),X(i));
	
	return sum/((double) E.getDim());
}


void EllipsoidalIntegrator::updateQ( Tmatrix<double> C, Tmatrix<Interval> R ){

	Tmatrix<double> CT(nx,nx);
	
	for( int i=0; i<nx; i++ )
		for( int j=0;j<nx;j++)
			CT(i,j) = C(j,i);
	
	Q = C*Q*CT;
	
	double trQ = 0.0;
	for( int i=0; i<nx; i++ ) trQ += Q(i,i)/(Q(i,i)+1e-8);
	trQ = ::sqrt(trQ);
	
	DVector sqrR(nx);
	for( int i=0; i<nx; i++ ) sqrR(i) = acadoMax(::fabs(R(i).l()),::fabs(R(i).u()))/::sqrt(Q(i,i)+1e-8);
	
	double kappa = trQ;
	for( int i=0; i<nx; i++ ) kappa += sqrR(i);
	
	Q *= kappa/(trQ+EPS);
	for( int i=0; i<nx; i++ ){
		double tmp = acadoMax(::fabs(R(i).l()),::fabs(R(i).u()));
		tmp *= ::sqrt(kappa/(sqrR(i)+EPS));
		Q(i,i) += tmp*tmp+EPS;
	}
}

Tmatrix<Interval> EllipsoidalIntegrator::boundQ() const{

	Tmatrix<Interval> R(nx);
	for( int i=0; i<nx; i++ ) R(i) = Interval(-1.0,1.0)*::sqrt(Q(i,i)+EPS*EPS);
	return R;
}

BooleanType EllipsoidalIntegrator::isIncluded( const Tmatrix<Interval> &A, const Tmatrix<Interval> &B ) const{

	ASSERT( A.getDim() == B.getDim() );

	BooleanType result = BT_TRUE;

	for( int i=0; i < (int) A.getDim(); i++ ){
		if( A(i).l() < B(i).l() || A(i).u() > B(i).u() ){ result = BT_FALSE; }
	}

	return result;
}


void EllipsoidalIntegrator::setInfinity(){ Q *= INFINITY; }


returnValue EllipsoidalIntegrator::setupOptions( ){
 
	addOption( MAX_NUM_INTEGRATOR_STEPS    , defaultMaxNumSteps             );
	addOption( INTEGRATOR_TOLERANCE        , defaultIntegratorTolerance     );
	addOption( ABSOLUTE_TOLERANCE          , defaultAbsoluteTolerance       );
	addOption( MIN_INTEGRATOR_STEPSIZE     , defaultMinStepsize             );
	addOption( MAX_INTEGRATOR_STEPSIZE     , defaultMaxStepsize             );
	addOption( INTEGRATOR_PRINTLEVEL       , defaultIntegratorPrintlevel    );
	addOption( PRINT_INTEGRATOR_PROFILE    , defaultprintIntegratorProfile  );
	addOption( STEPSIZE_TUNING             , defaultStepsizeTuning          );

	return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO

// end of file.
