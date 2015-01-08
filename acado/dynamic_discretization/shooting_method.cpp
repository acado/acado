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
 *    \file src/dynamic_discretization/shooting_method.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *
 */


#include <acado/dynamic_discretization/shooting_method.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//


ShootingMethod::ShootingMethod() : DynamicDiscretization( ){

    integrator = 0;
}


ShootingMethod::ShootingMethod( UserInteraction* _userInteraction )
               :DynamicDiscretization( _userInteraction ){

    integrator = 0;
}

ShootingMethod::ShootingMethod ( const ShootingMethod& arg ) : DynamicDiscretization( arg ){

    ShootingMethod::copy( arg );
}


ShootingMethod::~ShootingMethod( ){

    ShootingMethod::deleteAll();
}


ShootingMethod& ShootingMethod::operator=( const ShootingMethod& arg ){

    if ( this != &arg ){
        ShootingMethod::deleteAll();
        DynamicDiscretization::operator=(arg);
        ShootingMethod::copy( arg );
    }

    return *this;
}


void ShootingMethod::copy( const ShootingMethod &arg ){

    int run1;
    if( arg.integrator != 0 ){
        integrator = (Integrator**)calloc(N,sizeof(Integrator*));
        for( run1 = 0; run1 < N; run1++ ){
            if( arg.integrator[run1] != 0 ) integrator[run1] = (arg.integrator[run1])->clone();
            else                            integrator[run1] = 0                              ;
        }
    }
    else integrator = 0;

    breakPoints = arg.breakPoints;
}

DynamicDiscretization* ShootingMethod::clone( ) const{

    return new ShootingMethod(*this);
}


returnValue ShootingMethod::addStage( const DynamicSystem  &dynamicSystem_,
                                      const Grid           &stageIntervals,
                                      const IntegratorType &integratorType_ ){


    // LOAD THE DIFFERENTIAL EQUATION FROM THE DYNAMIC SYSTEM:
    // -------------------------------------------------------
    DifferentialEquation differentialEquation_ = dynamicSystem_.getDifferentialEquation( );
    int integratorTypeTmp = integratorType_;


    // AUTOMATICALLY SWITCH TO DISCRETE TIME INTEGRATOR IF NECESSARY:
    // --------------------------------------------------------------
    if( differentialEquation_.isDiscretized() == BT_TRUE ){
         if( integratorTypeTmp != INT_DISCRETE )
             integratorTypeTmp = INT_DISCRETE;
    }
    else{
         if( integratorTypeTmp == INT_DISCRETE )
             return ACADOERROR( RET_CANNOT_TREAT_CONTINUOUS_DE );
    }


    // CONSTRUCT THE APPROPRIATE INTEGRATOR BASED ON THE OPTIONS:
    // ----------------------------------------------------------
    int run1 = N;
    unionGrid = unionGrid & stageIntervals;
    N         = unionGrid.getNumIntervals();

    integrator = (Integrator**)realloc(integrator,N*sizeof(Integrator*));

    while( run1 < N ){
        allocateIntegrator( run1, (IntegratorType) integratorTypeTmp );
        integrator[run1]->init( differentialEquation_ );
        run1++;
    }

    // STORE THE INFORMATION ABOUT STAGE-BREAK POINTS AND START/END TIMES:
    // -------------------------------------------------------------------
    int tmp = 0;
    if( breakPoints.getNumRows() > 0 ){
        addOptionsList( );
        tmp = (int) breakPoints( breakPoints.getNumRows()-1, 0 );
    }

    DMatrix stageIndices(1,5);

    stageIndices(0,0) = stageIntervals.getNumIntervals() + tmp;
    stageIndices(0,1) = differentialEquation_.getStartTimeIdx();
    stageIndices(0,2) = differentialEquation_.getEndTimeIdx();
    stageIndices(0,3) = differentialEquation_.getStartTime();
    stageIndices(0,4) = differentialEquation_.getEndTime();

    breakPoints.appendRows(stageIndices);

    return SUCCESSFUL_RETURN;
}


returnValue ShootingMethod::allocateIntegrator( uint idx, IntegratorType type_ ){

    switch( type_ ){

         case INT_DISCRETE: integrator[idx] = new IntegratorDiscretizedODE(); break;
         case INT_RK12    : integrator[idx] = new IntegratorRK12          (); break;
         case INT_RK23    : integrator[idx] = new IntegratorRK23          (); break;
         case INT_RK45    : integrator[idx] = new IntegratorRK45          (); break;
         case INT_RK78    : integrator[idx] = new IntegratorRK78          (); break;
         case INT_BDF     : integrator[idx] = new IntegratorBDF           (); break;
         case INT_UNKNOWN : integrator[idx] = new IntegratorBDF           (); break;
         case INT_LYAPUNOV45 : integrator[idx] = new IntegratorLYAPUNOV45          (); break;

         default: return ACADOERROR( RET_UNKNOWN_BUG ); break;
    }
    return SUCCESSFUL_RETURN;
}



returnValue ShootingMethod::addTransition( const Transition& transition_ ){

    if( transition_.getNXA() != 0 ) return ACADOERROR( RET_TRANSITION_DEPENDS_ON_ALGEBRAIC_STATES );
    integrator[N-1]->setTransition( transition_ );

    return SUCCESSFUL_RETURN;
}


returnValue ShootingMethod::clear(){

    deleteAllSeeds();
    ShootingMethod::deleteAll();
    integrator = 0;
    breakPoints.init(0,0);

    return SUCCESSFUL_RETURN;
}



returnValue ShootingMethod::evaluate(	OCPiterate &iter
										)
{
// 	iter.print(); ASSERT( 1==0 );

    // INTRODUCE SOME AUXILIARY VARIABLES:
    // -----------------------------------
    ASSERT( iter.x != 0 );

    uint run1;
    double tStart, tEnd;

    DVector x ;  nx = iter.getNX ();
    DVector xa;  na = iter.getNXA();
    DVector p ;  np = iter.getNP ();
    DVector u ;  nu = iter.getNU ();
    DVector w ;  nw = iter.getNW ();

	VariablesGrid xAll;
	VariablesGrid xaAll;

    residuum = *(iter.x);
    residuum.setAll( 0.0 );

    iter.getInitialData( x, xa, p, u, w );
// 	iter.x->print( "x" );
// 	iter.u->print( "u" );

    // RUN A LOOP OVER ALL INTERVALS OF THE UNION GRID:
    // ------------------------------------------------

// 	printf("unionGrid:\n");
// 	unionGrid.print();

    for( run1 = 0; run1 < unionGrid.getNumIntervals(); run1++ ){

        integrator[run1]->setOptions( getOptions( 0 ) );  // ??

		int freezeIntegrator;
		get( FREEZE_INTEGRATOR, freezeIntegrator );

		if ( (BooleanType)freezeIntegrator == BT_TRUE )
			integrator[run1]->freezeAll();

        tStart = unionGrid.getTime( run1   );
        tEnd   = unionGrid.getTime( run1+1 );
		
// 		printf("tStart = %e,   tEnd = %e\n",tStart,tEnd );
// 		x.print("x");
// 		u.print("u");
// 		p.print("p");

// 		integrator[run1]->set( INTEGRATOR_PRINTLEVEL, MEDIUM );

		Grid evaluationGrid;
		iter.x->getSubGrid( tStart,tEnd,evaluationGrid );
		
        Grid outputGrid;
		if ( acadoIsNegative( integrator[run1]->getDifferentialEquationSampleTime( ) ) == BT_TRUE )
			outputGrid.init( tStart,tEnd,getNumEvaluationPoints() );
		else
			outputGrid.init( tStart,tEnd, 1+acadoRound( (tEnd-tStart)/integrator[run1]->getDifferentialEquationSampleTime() ) );

// 		printf("evaluationGrid:\n");
// 		evaluationGrid.print();
// 		printf("outputGrid:\n");
// 		outputGrid.print();
// 		printf("evaluationGrid+outputGrid:\n");
// 		(outputGrid&evaluationGrid).print();
// 		printf("\n");
// 		u.print("u before");
// 		x.print("x before");
// 		w.print("w");
        if ( integrator[run1]->integrate( outputGrid&evaluationGrid, x, xa, p, u, w ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_INTEGRATE_SYSTEM );

		
		DVector xOld;
		DVector pOld = p;
		
		if ( evaluationGrid.getNumPoints( ) <= 2 )
		{
			integrator[run1]->getX ( x  );
			integrator[run1]->getXA( xa );
// 			x.print("x after2");
			xOld = x;
			
			iter.updateData( tEnd, x, xa, p, u, w );
		}
		else
		{
			integrator[run1]->getX (  xAll );
			integrator[run1]->getXA( xaAll );

			xOld = xAll.getLastVector( );
// 			xOld.print("x after");
			
			for( uint run2=1; run2<outputGrid.getNumPoints(); ++run2 )
			{
				if ( evaluationGrid.hasTime( outputGrid.getTime(run2) ) == BT_TRUE )
				{
					x  =  xAll.getVector(run2);
					xa = xaAll.getVector(run2);
					iter.updateData( outputGrid.getTime(run2), x, xa, p, u, w );
				}
			}
		}

		if ( iter.isInSimulationMode( ) == BT_FALSE )
			p = pOld;  // should be changed later...

        residuum.setVector( run1, xOld - x );
// 		(xOld - x).print("residuum");
    }

    // LOG THE RESULTS:
    // ----------------
    return logTrajectory( iter );
}



returnValue ShootingMethod::differentiateBackward( const int    &idx ,
                                                   const DMatrix &seed,
                                                         DMatrix &Gx  ,
                                                         DMatrix &Gp  ,
                                                         DMatrix &Gu  ,
                                                         DMatrix &Gw    ){

    uint run1;

    Gx.init( seed.getNumRows(), nx );
    Gp.init( seed.getNumRows(), np );
    Gu.init( seed.getNumRows(), nu );
    Gw.init( seed.getNumRows(), nw );

    for( run1 = 0; run1 < seed.getNumRows(); run1++ ){

         DVector tmp = seed.getRow( run1 );
         DVector tmpX( nx );
         DVector tmpP( np );
         DVector tmpU( nu );
         DVector tmpW( nw );

         ACADO_TRY( integrator[idx]->setBackwardSeed( 1, tmp )                              );
         ACADO_TRY( integrator[idx]->integrateSensitivities( )                              );
         ACADO_TRY( integrator[idx]->getBackwardSensitivities( tmpX, tmpP, tmpU, tmpW , 1 ) );

         Gx.setRow( run1, tmpX );
         Gp.setRow( run1, tmpP );
         Gu.setRow( run1, tmpU );
         Gw.setRow( run1, tmpW );
    }

    return SUCCESSFUL_RETURN;
}



returnValue ShootingMethod::differentiateForward(  const int     &idx,
                                                   const DMatrix  &dX ,
                                                   const DMatrix  &dP ,
                                                   const DMatrix  &dU ,
                                                   const DMatrix  &dW ,
                                                         DMatrix  &D    ){

    int run1;
    int n = 0;

    n = acadoMax( n, dX.getNumCols() );
    n = acadoMax( n, dP.getNumCols() );
    n = acadoMax( n, dU.getNumCols() );
    n = acadoMax( n, dW.getNumCols() );

    D.init( nx, n );

    for( run1 = 0; run1 < n; run1++ ){

         DVector tmp;

         DVector tmpX; if( dX.isEmpty() == BT_FALSE ) tmpX = dX.getCol( run1 );
         DVector tmpP; if( dP.isEmpty() == BT_FALSE ) tmpP = dP.getCol( run1 );
         DVector tmpU; if( dU.isEmpty() == BT_FALSE ) tmpU = dU.getCol( run1 );
         DVector tmpW; if( dW.isEmpty() == BT_FALSE ) tmpW = dW.getCol( run1 );

         ACADO_TRY( integrator[idx]->setForwardSeed( 1, tmpX, tmpP, tmpU, tmpW ) );
         ACADO_TRY( integrator[idx]->integrateSensitivities( )                   );
         ACADO_TRY( integrator[idx]->getForwardSensitivities( tmp, 1 )           );

         D.setCol( run1, tmp );
    }

    return SUCCESSFUL_RETURN;
}


returnValue ShootingMethod::differentiateForwardBackward( const int     &idx ,
                                                          const DMatrix  &dX  ,
                                                          const DMatrix  &dP  ,
                                                          const DMatrix  &dU  ,
                                                          const DMatrix  &dW  ,
                                                          const DMatrix  &seed,
                                                                DMatrix  &D   ,
                                                                DMatrix  &ddX ,
                                                                DMatrix  &ddP ,
                                                                DMatrix  &ddU ,
                                                                DMatrix  &ddW   ){

    int run1;
    int n = 0;

    n = acadoMax( n, dX.getNumCols() );
    n = acadoMax( n, dP.getNumCols() );
    n = acadoMax( n, dU.getNumCols() );
    n = acadoMax( n, dW.getNumCols() );

    D.init( nx, n );

    ddX.init( n, nx );
    ddP.init( n, np );
    ddU.init( n, nu );
    ddW.init( n, nw );

    for( run1 = 0; run1 < n; run1++ ){

         DVector tmp;

         DVector tmpX; if( dX.isEmpty() == BT_FALSE ) tmpX = dX.getCol( run1 );
         DVector tmpP; if( dP.isEmpty() == BT_FALSE ) tmpP = dP.getCol( run1 );
         DVector tmpU; if( dU.isEmpty() == BT_FALSE ) tmpU = dU.getCol( run1 );
         DVector tmpW; if( dW.isEmpty() == BT_FALSE ) tmpW = dW.getCol( run1 );

         ACADO_TRY( integrator[idx]->setForwardSeed( 1, tmpX, tmpP, tmpU, tmpW ) );
         ACADO_TRY( integrator[idx]->integrateSensitivities( )                   );
         ACADO_TRY( integrator[idx]->getForwardSensitivities( tmp, 1 )           );

         D.setCol( run1, tmp );

         DVector tmp2 = seed.getCol(0);

         DVector tmpX2( nx );
         DVector tmpP2( np );
         DVector tmpU2( nu );
         DVector tmpW2( nw );

         ACADO_TRY( integrator[idx]->setBackwardSeed( 2, tmp2 )                                 );
         ACADO_TRY( integrator[idx]->integrateSensitivities( )                                  );
         ACADO_TRY( integrator[idx]->getBackwardSensitivities( tmpX2, tmpP2, tmpU2, tmpW2 , 2 ) );

         ddX.setRow( run1, tmpX2 );
         ddP.setRow( run1, tmpP2 );
         ddU.setRow( run1, tmpU2 );
         ddW.setRow( run1, tmpW2 );

         ACADO_TRY( integrator[idx]->deleteAllSeeds() );
    }

    return SUCCESSFUL_RETURN;
}



returnValue ShootingMethod::evaluateSensitivities(){

    int i;

    // COMPUTATION OF BACKWARD SENSITIVITIES:
    // --------------------------------------

    if( bSeed.isEmpty() == BT_FALSE ){

        dBackward.init( N, 5 );

        for( i = 0; i < N; i++ ){

             DMatrix seed, X, P, U, W;
             bSeed.getSubBlock( 0, i, seed );

             ACADO_TRY( differentiateBackward( i, seed, X, P, U, W ) );

             if( nx > 0 ) dBackward.setDense( i, 0, X );
             if( np > 0 ) dBackward.setDense( i, 2, P );
             if( nu > 0 ) dBackward.setDense( i, 3, U );
             if( nw > 0 ) dBackward.setDense( i, 4, W );
        }
        return SUCCESSFUL_RETURN;
    }


    // COMPUTATION OF FORWARD SENSITIVITIES:
    // -------------------------------------

    dForward.init( N, 5 );

    for( i = 0; i < N; i++ ){

        DMatrix X, P, U, W, D, E;

        if( xSeed.isEmpty() == BT_FALSE ) xSeed.getSubBlock( i, 0, X );
        if( pSeed.isEmpty() == BT_FALSE ) pSeed.getSubBlock( i, 0, P );
        if( uSeed.isEmpty() == BT_FALSE ) uSeed.getSubBlock( i, 0, U );
        if( wSeed.isEmpty() == BT_FALSE ) wSeed.getSubBlock( i, 0, W );

        if( nx > 0 ){ ACADO_TRY( differentiateForward( i, X, E, E, E, D )); dForward.setDense( i, 0, D ); }
        if( np > 0 ){ ACADO_TRY( differentiateForward( i, E, P, E, E, D )); dForward.setDense( i, 2, D ); }
        if( nu > 0 ){ ACADO_TRY( differentiateForward( i, E, E, U, E, D )); dForward.setDense( i, 3, D ); }
        if( nw > 0 ){ ACADO_TRY( differentiateForward( i, E, E, E, W, D )); dForward.setDense( i, 4, D ); }
    }
    return SUCCESSFUL_RETURN;
}




returnValue ShootingMethod::update( DMatrix &G, const DMatrix &A, const DMatrix &B ){

    if( B.getNumCols() == 0 ) return SUCCESSFUL_RETURN;

    DMatrix E = eye<double>(B.getNumCols());
    E *= 1e-10;

    DMatrix S = ((A.transpose().eval() * A)+E).inverse();
    G += (B-G*A)*(S*A.transpose());
    return SUCCESSFUL_RETURN;
}



returnValue ShootingMethod::evaluateSensitivitiesLifted( ){

    int i,j;

    dForward.init( N, 5 );
    DMatrix Gx, *Gu, b, d, D, E, X, P, U, W, A, B;
    Gu = new DMatrix[N];

    for( i = 0; i < N; i++ ){

        if( xSeed.isEmpty() == BT_FALSE ) xSeed.getSubBlock( i, 0, X );
        if( pSeed.isEmpty() == BT_FALSE ) pSeed.getSubBlock( i, 0, P );
        if( uSeed.isEmpty() == BT_FALSE ) uSeed.getSubBlock( i, 0, U );
        if( wSeed.isEmpty() == BT_FALSE ) wSeed.getSubBlock( i, 0, W );

        if( np > 0 ){ D.init( nx, np ); D.setZero(); dForward.setDense( i, 2, D ); }
        if( nw > 0 ){ D.init( nx, nw ); D.setZero(); dForward.setDense( i, 4, D ); }

        if( nu > 0 ){ ACADO_TRY( differentiateForward( i, E, E, U, E, D )); dForward.setDense( i, 3, D ); Gu[i] = D; }
    }

    if( nu > 0 ){

        d.init(nx,1);
        d.setZero();

        for( i = 0; i < N; i++ ){
            A.init(0,0);
            B.init(0,0);
            for( j = 0; j < i; j++ ){
                differentiateForward( i, Gu[j], E, E, E, D );
                A.appendCols( Gu[j] );
                B.appendCols( D     );
                Gu[j] = D;
            }

            if( i > 0 ){
                differentiateForward( i, d, E, E, E, b );
                A.appendCols( d );
                B.appendCols( b );
                d = b;
            }
            d += residuum.getMatrix(i);
            Gx = eye<double>(nx);
            Gx *= 0.001;
            update( Gx, A, B );
            dForward.setDense( i, 0, Gx );
        }
    }
    delete[] Gu;

    return SUCCESSFUL_RETURN;
}



returnValue ShootingMethod::evaluateSensitivities( const BlockMatrix &seed, BlockMatrix &hessian ){

    const int NN = N+1;
    dForward.init( N, 5 );
    int i;

    for( i = 0; i < N; i++ ){

        DMatrix X, P, U, W, D, E, HX, HP, HU, HW, S;

        if( xSeed.isEmpty() == BT_FALSE ) xSeed.getSubBlock( i, 0, X );
        if( pSeed.isEmpty() == BT_FALSE ) pSeed.getSubBlock( i, 0, P );
        if( uSeed.isEmpty() == BT_FALSE ) uSeed.getSubBlock( i, 0, U );
        if( wSeed.isEmpty() == BT_FALSE ) wSeed.getSubBlock( i, 0, W );

        seed.getSubBlock( i, 0, S, nx, 1 );

        if( nx > 0 ){

            ACADO_TRY( differentiateForwardBackward( i, X, E, E, E, S, D, HX, HP, HU, HW ));
            dForward.setDense( i, 0, D );

            if( nx > 0 ) hessian.addDense( i,      i, HX );
            if( np > 0 ) hessian.addDense( i, 2*NN+i, HP );
            if( nu > 0 ) hessian.addDense( i, 3*NN+i, HU );
            if( nw > 0 ) hessian.addDense( i, 4*NN+i, HW );
        }

        if( np > 0 ){

            ACADO_TRY( differentiateForwardBackward( i, E, P, E, E, S, D, HX, HP, HU, HW ));
            dForward.setDense( i, 2, D );

            if( nx > 0 ) hessian.addDense( 2*NN+i,      i, HX );
            if( np > 0 ) hessian.addDense( 2*NN+i, 2*NN+i, HP );
            if( nu > 0 ) hessian.addDense( 2*NN+i, 3*NN+i, HU );
            if( nw > 0 ) hessian.addDense( 2*NN+i, 4*NN+i, HW );
        }

        if( nu > 0 ){

            ACADO_TRY( differentiateForwardBackward( i, E, E, U, E, S, D, HX, HP, HU, HW ));
            dForward.setDense( i, 3, D );

            if( nx > 0 ) hessian.addDense( 3*NN+i,      i, HX );
            if( np > 0 ) hessian.addDense( 3*NN+i, 2*NN+i, HP );
            if( nu > 0 ) hessian.addDense( 3*NN+i, 3*NN+i, HU );
            if( nw > 0 ) hessian.addDense( 3*NN+i, 4*NN+i, HW );
        }

        if( nw > 0 ){

            ACADO_TRY( differentiateForwardBackward( i, E, E, E, W, S, D, HX, HP, HU, HW ));
            dForward.setDense( i, 4, D );

            if( nx > 0 ) hessian.addDense( 4*NN+i,      i, HX );
            if( np > 0 ) hessian.addDense( 4*NN+i, 2*NN+i, HP );
            if( nu > 0 ) hessian.addDense( 4*NN+i, 3*NN+i, HU );
            if( nw > 0 ) hessian.addDense( 4*NN+i, 4*NN+i, HW );
        }
    }
    return SUCCESSFUL_RETURN;
}


returnValue ShootingMethod::unfreeze(){

    int run1;
    for( run1 = 0; run1 < (int) unionGrid.getNumIntervals(); run1++ )
         integrator[run1]->unfreeze();

    return SUCCESSFUL_RETURN;
}


returnValue ShootingMethod::deleteAllSeeds(){

    DynamicDiscretization::deleteAllSeeds();

    uint i;
    for( i = 0; i < unionGrid.getNumIntervals(); i++ )
        integrator[i]->deleteAllSeeds();

    return SUCCESSFUL_RETURN;
}


BooleanType ShootingMethod::isAffine( ) const{

	for( int run1 = 0; run1 < N; ++run1 )
		if ( integrator[run1]->isDifferentialEquationAffine( ) == BT_FALSE )
			return BT_FALSE;

    return BT_TRUE;
}


returnValue ShootingMethod::deleteAll(){

    int run1;
    if( integrator != 0 ){
        for( run1 = 0; run1 < N; run1++ )
            if( integrator[run1] != 0 )
                delete integrator[run1];
        free(integrator);
        integrator = 0;
    }

	unionGrid.init();
	DynamicDiscretization::initializeVariables( );

    return SUCCESSFUL_RETURN;
}


returnValue ShootingMethod::logTrajectory( const OCPiterate &iter ){

    if( integrator == 0 ) return SUCCESSFUL_RETURN;

    int i, j;
    double T = 0.0;
	double t1 = 0.0, t2 = 0.0;
	double h = 0.0;
	BooleanType needToRescale = BT_FALSE;

    VariablesGrid logX, logXA, logP, logU, logW, logI, tmp,tmp2;

    DMatrix intervalPoints(N+1,1);
    intervalPoints(0,0) = 0.0;

    j = 0;
    for( i = 0; i < N; i++ ){

        if( (int) breakPoints(j,0) <= i ) j++;

        int i1 = (int) breakPoints(j,1);
        int i2 = (int) breakPoints(j,2);

        if( i1 >= 0 )  t1 = iter.p->operator()(0,i1);
        else           t1 = breakPoints(j,3);

        if( i2 >= 0 )  t2 = iter.p->operator()(0,i2);
        else           t2 = breakPoints(j,4);

        if( i == 0 ) T = t1;

        integrator[i]->getX( tmp );

        intervalPoints(i+1,0) = intervalPoints(i,0) + tmp.getNumPoints();

        if ( ( i1 >= 0 ) || ( i2 >= 0 ) )
		{
			if ( iter.isInSimulationMode() == BT_FALSE )
			{
				h = t2-t1;
				needToRescale = BT_TRUE;
			}
		}
        else
		{
			h = 1.0;
			needToRescale = BT_FALSE;
		}

        if( nx > 0 ){ if ( needToRescale == BT_TRUE ) rescale( &tmp, T, h );
						logX .appendTimes( tmp );
                    }
        if( na > 0 ){ integrator[i]->getXA( tmp );
                      if ( needToRescale == BT_TRUE ) rescale( &tmp, T, h );
                      logXA.appendTimes( tmp );
                    }
        if( np > 0 ){ tmp2.init( np, tmp.getFirstTime(),tmp.getLastTime(),2 );
					  if ( iter.isInSimulationMode( ) == BT_FALSE )
						tmp2.setAllVectors( iter.p->getVector(0) );
					  else
						  tmp2.setAllVectors( iter.p->getVector(i) );
                      logP .appendTimes( tmp2 );
                    }
        if( nu > 0 ){ tmp2.init( nu, tmp.getFirstTime(),tmp.getLastTime(),2 );
                      tmp2.setAllVectors(iter.u->getVector(i));
                      logU .appendTimes( tmp2 );
                    }
        if( nw > 0 ){ tmp.init( nw, tmp );
                      tmp.setAllVectors(iter.w->getVector(i));
                      logW .appendTimes( tmp );
                    }
                      integrator[i]->getI( tmp );
                      if ( needToRescale == BT_TRUE ) rescale( &tmp, T, h );
                      logI .appendTimes( tmp );
        T = tmp.getLastTime();
    }


    // WRITE DATA TO THE LOG COLLECTION:
    // ---------------------------------
    if( nx > 0 ) setLast( LOG_DIFFERENTIAL_STATES, logX   );
    if( na > 0 ) setLast( LOG_ALGEBRAIC_STATES   , logXA  );
    if( np > 0 ) setLast( LOG_PARAMETERS         , logP   );
    if( nu > 0 ) setLast( LOG_CONTROLS           , logU   );
    if( nw > 0 ) setLast( LOG_DISTURBANCES       , logW   );

    setLast( LOG_INTERMEDIATE_STATES     , logI           );
    setLast( LOG_DISCRETIZATION_INTERVALS, intervalPoints );


    return SUCCESSFUL_RETURN;
}


returnValue ShootingMethod::rescale(	VariablesGrid* trajectory,
										double tEndNew,
										double newIntervalLength
										) const
{
	trajectory->shiftTimes( -trajectory->getTime(0) );
	trajectory->scaleTimes( newIntervalLength );
	trajectory->shiftTimes( tEndNew  );
	
	return SUCCESSFUL_RETURN;
}


CLOSE_NAMESPACE_ACADO

// end of file.
