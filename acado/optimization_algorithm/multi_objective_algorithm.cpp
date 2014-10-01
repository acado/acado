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
 *    \file src/optimization_algorithm/multi_objective_algorithm.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2009
 */


#include <acado/optimization_algorithm/multi_objective_algorithm.hpp>
#include <acado/ocp/ocp.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


MultiObjectiveAlgorithm::MultiObjectiveAlgorithm()
                        :OptimizationAlgorithm(){

    m                          = 0;
    N                          = 0;
    count                      = 0;
    totalNumberOfSQPiterations = 0;
    totalCPUtime               = 0;

    xResults  = 0;
    xaResults = 0;
    pResults  = 0;
    uResults  = 0;
    wResults  = 0;
	
	setupOptions( );
}


MultiObjectiveAlgorithm::MultiObjectiveAlgorithm( const OCP& ocp_ )
                        :OptimizationAlgorithm( ocp_ ){

    m            = ocp->getNumberOfMayerTerms();
    N            = 0;
    count        = 0;
    totalNumberOfSQPiterations = 0;
    totalCPUtime               = 0;

    xResults  = 0;
    xaResults = 0;
    pResults  = 0;
    uResults  = 0;
    wResults  = 0;
	
	setupOptions( );
}


MultiObjectiveAlgorithm::MultiObjectiveAlgorithm( const MultiObjectiveAlgorithm& arg )
                        :OptimizationAlgorithm( arg )
{

    m = arg.m;
    N = arg.N;

    vertices = arg.vertices;
    result   = arg.result  ;

    count = arg.count;

    totalNumberOfSQPiterations = arg.totalNumberOfSQPiterations;
    totalCPUtime               = arg.totalCPUtime              ;

    xResults  = 0;
    xaResults = 0;
    pResults  = 0;
    uResults  = 0;
    wResults  = 0;
}


MultiObjectiveAlgorithm::~MultiObjectiveAlgorithm( ){

    if( xResults  != 0 ) delete[] xResults ;
    if( xaResults != 0 ) delete[] xaResults;
    if( pResults  != 0 ) delete[] pResults ;
    if( uResults  != 0 ) delete[] uResults ;
    if( wResults  != 0 ) delete[] wResults ;
}


MultiObjectiveAlgorithm& MultiObjectiveAlgorithm::operator=( const MultiObjectiveAlgorithm& arg ){

    if( this != &arg ){

        OptimizationAlgorithm         ::operator=(arg);

        m = arg.m;
        N = arg.N;

        vertices = arg.vertices;
        result   = arg.result  ;
        count = arg.count;

        totalNumberOfSQPiterations = arg.totalNumberOfSQPiterations;
        totalCPUtime               = arg.totalCPUtime              ;

        xResults  = 0;
        xaResults = 0;
        pResults  = 0;
        uResults  = 0;
        wResults  = 0;
    }
    return *this;
}


returnValue MultiObjectiveAlgorithm::solveSingleObjective( const int &number_ ){


    int         run1       ;
    returnValue returnvalue;

    ASSERT( ocp != 0 );
    ASSERT( number_ < m );
    if( N == 0 ) get( PARETO_FRONT_DISCRETIZATION, N );

    int N_pow_m_minus_1 = 1; 
    int i;
    for(i=0; i<m-1; i++)
      N_pow_m_minus_1 *= N;

    Expression **arg = 0;
    arg = new Expression*[m];
    for( run1 = 0; run1 < m; run1++ )
        ocp->getObjective( run1, &arg[run1] );

    Grid tmp_grid;
    ocp->getGrid( tmp_grid );
    Objective  tmp(tmp_grid);
    tmp.addMayerTerm(*arg[number_]);
    ocp->setObjective( tmp );

    setStatus( BS_NOT_INITIALIZED );

    printf("\n\n Optimization of individual objective %d out of %d \n\n",number_ +1, m );
    totalCPUtime = -acadoGetTime();
    returnvalue = OptimizationAlgorithm::solve();
    totalCPUtime += acadoGetTime();

    int index=0;
    DMatrix Weights = getWeights();

    for( run1 = 0; run1 < (int) Weights.getNumCols(); run1++ ){
        if( fabs( Weights( number_, run1 ) - 1.0 ) < 1000.0*EPS )
            index = run1;
    }

    if( xResults  == 0 ) xResults  = new VariablesGrid[Weights.getNumCols()];
    if( xaResults == 0 ) xaResults = new VariablesGrid[Weights.getNumCols()];
    if( pResults  == 0 ) pResults  = new VariablesGrid[Weights.getNumCols()];
    if( uResults  == 0 ) uResults  = new VariablesGrid[Weights.getNumCols()];
    if( wResults  == 0 ) wResults  = new VariablesGrid[Weights.getNumCols()];

    getDifferentialStates( xResults[index]  );
    getAlgebraicStates   ( xaResults[index] );
    getParameters        ( pResults[index]  );
    getControls          ( uResults[index]  );
    getDisturbances      ( wResults[index]  );


    if( returnvalue != SUCCESSFUL_RETURN )
        return ACADOERROR(returnvalue);

    if( nlpSolver != 0 )
        totalNumberOfSQPiterations = nlpSolver->getNumberOfSteps();

    int hotstart;
    get( PARETO_FRONT_HOTSTART, hotstart );

    int *indices = new int[m];
    for( run1 = 0; run1 < m; run1++ )
        indices[run1] = number_;

    VariablesGrid xd_tmp, xa_tmp, p_tmp, u_tmp, w_tmp;

    if( hotstart == BT_TRUE ){
        getDifferentialStates( *userInit.x );
        getAlgebraicStates   ( *userInit.xa );
        getParameters        ( *userInit.p );
        getControls          ( *userInit.u );
        getDisturbances      ( *userInit.w );
        xd_tmp = *userInit.x;
        xa_tmp = *userInit.xa;
        p_tmp  = *userInit.p;
        u_tmp  = *userInit.u;
        w_tmp  = *userInit.w;
    }
    else{
        getDifferentialStates( xd_tmp );
        getAlgebraicStates   ( xa_tmp );
        getParameters        ( p_tmp  );
        getControls          ( u_tmp  );
        getDisturbances      ( w_tmp  );
    }

    VariablesGrid *_xd = 0;
    VariablesGrid *_xa = 0;
    VariablesGrid *_p  = 0;
    VariablesGrid *_u  = 0;
    VariablesGrid *_w  = 0;

    if( xd_tmp.isEmpty() == BT_FALSE ) _xd = new VariablesGrid(xd_tmp);
    if( xa_tmp.isEmpty() == BT_FALSE ) _xa = new VariablesGrid(xa_tmp);
    if( p_tmp.isEmpty()  == BT_FALSE ) _p  = new VariablesGrid(p_tmp );
    if( u_tmp.isEmpty()  == BT_FALSE ) _u  = new VariablesGrid(u_tmp );
    if( w_tmp.isEmpty()  == BT_FALSE ) _w  = new VariablesGrid(w_tmp );

    if( vertices.getDim() == 0 ){
        vertices.init(m,m);
        vertices.setAll(-INFTY);
    }

    Objective **obj = new Objective*[m];
    for( run1 = 0; run1 < m; run1++ ){
        obj[run1] = new Objective( tmp_grid );
        obj[run1]->addMayerTerm(*arg[run1]);
        OCPiterate xx( _xd, _xa, _p, _u, _w );
        obj[run1]->evaluate( xx );
        obj[run1]->getObjectiveValue( vertices(number_,run1) );
    }

    if( _xd != 0 ) delete _xd;
    if( _xa != 0 ) delete _xa;
    if( _p  != 0 ) delete _p ;
    if( _u  != 0 ) delete _u ;
    if( _w  != 0 ) delete _w ;


    delete[] indices;

    for( run1 = 0; run1 < m; run1++ )
        delete arg[run1];
    delete[] arg;

    for( run1 = 0; run1 < m; run1++ )
        delete obj[run1];
    delete[] obj;

    return SUCCESSFUL_RETURN;
}


returnValue MultiObjectiveAlgorithm::solve( ){

    int           run1,run2;
    returnValue returnvalue;

    ASSERT( ocp != 0 );
    ASSERT( m >= 2 );
    if( N == 0 ) get( PARETO_FRONT_DISCRETIZATION, N );

    int paretoGeneration;
    get( PARETO_FRONT_GENERATION, paretoGeneration );

    int hotstart;
    get( PARETO_FRONT_HOTSTART, hotstart );

    Expression **arg = 0;
    arg = new Expression*[m];

    for( run1 = 0; run1 < m; run1++ )
        ocp->getObjective( run1, &arg[run1] );

    Constraint tmp_con;

    double *idx = new double[m];

    WeightGeneration generator;
    DMatrix Weights;
    DVector formers;
    DVector  lb(m);
    DVector  ub(m);
    lb.setZero();
    ub.setAll(1.0);

    generator.getWeights( m, N, lb, ub, Weights, formers );

    result.init( Weights.getNumCols(), m );
    count = 0;

    if( xResults  == 0 ) xResults  = new VariablesGrid[Weights.getNumCols()];
    if( xaResults == 0 ) xaResults = new VariablesGrid[Weights.getNumCols()];
    if( pResults  == 0 ) pResults  = new VariablesGrid[Weights.getNumCols()];
    if( uResults  == 0 ) uResults  = new VariablesGrid[Weights.getNumCols()];
    if( wResults  == 0 ) wResults  = new VariablesGrid[Weights.getNumCols()];

    totalNumberOfSQPiterations = 0;
    totalCPUtime               = -acadoGetTime();

    run1 = 0;
    while( run1 < (int) Weights.getNumCols() ){


        // PRINT THE ITERATION NUMBER:
        // ---------------------------
    	printf("\n\n Multi-objective point: %d out of %d \n\n",run1+1, (int) Weights.getNumCols() );


        ocp->getConstraint( tmp_con );

        for( run2 = 0; run2 < (int) Weights.getNumRows(); run2++ )
            idx[run2] = Weights( run2, run1 );


        // THIS PART OF THE CODE WILL NOT RUN YET FOR GENERAL WEIGHTS

        int vertex = -1;
        for( run2 = 0; run2 < m; run2++ ){
            if( fabs( idx[run2]-1.0 ) < 100.0*EPS )
                vertex = run2;
        }
        // ----------------------------------------------------------


        if( vertex == -1 || paretoGeneration == PFG_WEIGHTED_SUM ){

            formulateOCP( idx, ocp, arg );
            setStatus( BS_NOT_INITIALIZED );
            returnvalue = OptimizationAlgorithm::solve();

            if( nlpSolver != 0 )
                totalNumberOfSQPiterations += nlpSolver->getNumberOfSteps();

            ocp->setConstraint( tmp_con );
            set( PRINT_COPYRIGHT, BT_FALSE );

            if( returnvalue != SUCCESSFUL_RETURN ){
                ACADOERROR(returnvalue);
            }
            else{

               getDifferentialStates( xResults[run1]  );
               getAlgebraicStates   ( xaResults[run1] );
               getParameters        ( pResults[run1]  );
               getControls          ( uResults[run1]  );
               getDisturbances      ( wResults[run1]  );

               if( hotstart == BT_TRUE ){
                    getDifferentialStates( *userInit.x );
                    getAlgebraicStates   ( *userInit.xa );
                    getParameters        ( *userInit.p  );
                    getControls          ( *userInit.u );
                    getDisturbances      ( *userInit.w  );
                    evaluateObjectives( *userInit.x, *userInit.xa, *userInit.p, *userInit.u, *userInit.w, arg );
                }
                else{
                   VariablesGrid xd_tmp, xa_tmp, p_tmp, u_tmp, w_tmp;
                   getDifferentialStates( xd_tmp );
                   getAlgebraicStates   ( xa_tmp );
                   getParameters        ( p_tmp  );
                   getControls          ( u_tmp  );
                   getDisturbances      ( w_tmp  );
                   evaluateObjectives( xd_tmp, xa_tmp, p_tmp, u_tmp, w_tmp, arg );
               }
            }
        }
        else{
        	printf(" Result from single objective optimization is adopted. \n\n" );
            for( run2 = 0; run2 < m; run2++ ){
                result(count,run2) = vertices(vertex,run2);
            }
            count++;
        }
        run1++;
    }
    totalCPUtime += acadoGetTime();

    for( run1 = 0; run1 < m; run1++ )
        delete arg[run1];
    delete[] arg;

    delete[] idx;

    return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue MultiObjectiveAlgorithm::setupOptions( )
{
    addOption( PARETO_FRONT_DISCRETIZATION  , defaultParetoFrontDiscretization );
    addOption( PARETO_FRONT_GENERATION      , defaultParetoFrontGeneration     );
    addOption( PARETO_FRONT_HOTSTART        , defaultParetoFrontHotstart       );

	// add optimization algorithm options
	//OptimizationAlgorithm::setupOptions( );

	return SUCCESSFUL_RETURN;
}


returnValue MultiObjectiveAlgorithm::initializeNlpSolver( const OCPiterate& _userInit )
{
	return OptimizationAlgorithm::initializeNlpSolver( _userInit );
}


returnValue MultiObjectiveAlgorithm::initializeObjective(	Objective* F
														)
{
	return SUCCESSFUL_RETURN;
}




returnValue MultiObjectiveAlgorithm::formulateOCP( double *idx, OCP *ocp_, Expression **arg ){

    int run1, run2;
    int paretoGeneration;
    double factor;

    get( PARETO_FRONT_GENERATION, paretoGeneration );
    if( paretoGeneration == PFG_UNKNOWN ) paretoGeneration = PFG_WEIGHTED_SUM;

    Grid tmp_grid;
    ocp_->getGrid( tmp_grid );
    Objective  tmp(tmp_grid);


    // WEIGTHED SUM:
    // -----------------------------------------------------------------------------

    if( paretoGeneration == PFG_WEIGHTED_SUM ){

        Expression sum(1);

        for( run1 = 0; run1 < m; run1++ ){   // loop over the number of objectives
             factor = idx[run1];             // determines the weight factor
             sum = sum + arg[run1][0]*factor;
        }
        tmp.addMayerTerm(sum);               // add the new objective as a Meyer Term
        ocp_->setObjective(tmp);             // replace (overwrite) the objective in the ocp
        return SUCCESSFUL_RETURN;
    }


    // NORMALIZED NORMAL CONSTRAINT:
    // -----------------------------------------------------------------------------

    if( paretoGeneration == PFG_NORMALIZED_NORMAL_CONSTRAINT ){ 
	// Normalization based on Messac et al 2004

        //tmp.addMayerTerm( *arg[m-1] );			// Select last (i.e., m-th) objective function
        //ocp_->setObjective( tmp );			// replace (overwrite) the objective in the ocp

        DMatrix P  = getNormalizedPayOffMatrix();
        DMatrix NK = getUtopiaPlaneVectors    ();

        DVector W(m);
        for( run1 = 0; run1 < m; run1++ )
            W(run1) = idx[run1];

        DVector PW = P*W;

        DVector U = getUtopiaVector();
        DVector L = getNormalizationVector();

        Expression *Fnorm;
        Fnorm = new Expression[m];

        for( run2 = 0; run2 < m; run2++ ){

             Fnorm[run2] = ( *arg[run2] - U(run2) ) / L(run2);
        }

        tmp.addMayerTerm( Fnorm[m-1] );
        ocp_->setObjective( tmp );

        for( run1 = 0; run1 < m-1; run1++ ){
            Expression sum(1);
            for( run2 = 0; run2 < m; run2++ ){

                sum = sum + NK(run2,run1)*( Fnorm[run2] - PW(run2) );
            }
            ocp_->subjectTo( AT_END, sum <= 0.0 );
        }
        delete[] Fnorm;

        return SUCCESSFUL_RETURN;
    }


    // ENHANCED NORMALIZED NORMAL CONSTRAINT:
    // -----------------------------------------------------------------------------

    if( paretoGeneration == PFG_ENHANCED_NORMALIZED_NORMAL_CONSTRAINT ){
	// Normalization based on Sanchis et al 2008

        DMatrix P  = getPayOffMatrix();
        DMatrix PHI_N(m,m);

        int run3, run4;
        for( run3 = 0; run3 < m; run3++ ){
            for( run4 = 0; run4 < m; run4++ ){
                PHI_N(run3,run4) = 1.0;
            }
        }
        for( run3 = 0; run3 < m; run3++ )
            PHI_N(run3,run3) = 0.0;

        DMatrix T;
        T = PHI_N * P.inverse();

        DMatrix NK(m,m-1);
        NK.setZero();

        for( run3 = 0; run3 < m-1; run3++ )
             NK(run3,run3) = 1.0;

        for( run3 = 0; run3 < m-1; run3++ )
             NK(m-1,run3) = -1.0;


        DVector W(m);
        for( run1 = 0; run1 < m; run1++ )
            W(run1) = idx[run1];

        DVector PW = PHI_N*W;

        DVector U = getUtopiaVector();

        Expression *Fnorm;
        Fnorm = new Expression[m];

        for( run2 = 0; run2 < m; run2++ ){
            Expression tmp3(1);
            for( run3 = 0; run3 < m; run3++ ){
                tmp3 = tmp3 + T(run2,run3)*( *arg[run3]- U(run3) );
            }
            Fnorm[run2] = tmp3;
        }

        tmp.addMayerTerm( Fnorm[m-1] );
        ocp_->setObjective( tmp );

        for( run1 = 0; run1 < m-1; run1++ ){

            Expression sum(1);

            for( run2 = 0; run2 < m; run2++ ){

                sum =  sum + NK(run2,run1)*( Fnorm[run2] - PW(run2) );
            }
            ocp_->subjectTo( AT_END, sum <= 0.0 );
        }
        delete[] Fnorm;

        return SUCCESSFUL_RETURN;
    }


    // NORMAL BOUNDARY INTERSECTION:
    // -----------------------------------------------------------------------------

    if( paretoGeneration == PFG_NORMAL_BOUNDARY_INTERSECTION ){

        DVector W(m);
        for( run1 = 0; run1 < m; run1++ )
            W(run1) = idx[run1];

        DMatrix P  = getPayOffMatrix();
        DVector U  = getUtopiaVector();
        DVector V  = P*W + U;
        W.setAll( 1.0 );
        DVector X  = P*W;

        Expression lambda;

        lambda = ( *arg[m-1] - V(m-1) )/ X(m-1);

        tmp.addMayerTerm( *arg[m-1] );
        ocp_->setObjective( tmp );

        for( run1 = 0; run1 < m-1; run1++ )
             ocp->subjectTo( AT_END, *arg[run1] - lambda*X(run1) == V(run1) );

        return SUCCESSFUL_RETURN;
    }
    return SUCCESSFUL_RETURN;
}


returnValue MultiObjectiveAlgorithm::evaluateObjectives( VariablesGrid    &xd_ ,
                                                         VariablesGrid    &xa_ ,
                                                         VariablesGrid    &p_  ,
                                                         VariablesGrid    &u_  ,
                                                         VariablesGrid    &w_  ,
                                                         Expression      **arg   ){

    int run1;
    Grid tmp_grid;
    ocp->getGrid( tmp_grid );

    VariablesGrid *_xd = 0;
    VariablesGrid *_xa = 0;
    VariablesGrid *_p  = 0;
    VariablesGrid *_u  = 0;
    VariablesGrid *_w  = 0;

    if( xd_.isEmpty() == BT_FALSE ) _xd = new VariablesGrid(xd_);
    if( xa_.isEmpty() == BT_FALSE ) _xa = new VariablesGrid(xa_);
    if( p_.isEmpty()  == BT_FALSE ) _p  = new VariablesGrid(p_ );
    if( u_.isEmpty()  == BT_FALSE ) _u  = new VariablesGrid(u_ );
    if( w_.isEmpty()  == BT_FALSE ) _w  = new VariablesGrid(w_ );

    Objective *obj;
    for( run1 = 0; run1 < m; run1++ ){
        obj = new Objective( tmp_grid );
        obj->addMayerTerm(*arg[run1]);
        OCPiterate xx( _xd, _xa, _p, _u, _w );
        obj->evaluate( xx );
        obj->getObjectiveValue( result(count,run1) );
        delete obj;
    }
    count++;

    if( _xd != 0 ) delete _xd;
    if( _xa != 0 ) delete _xa;
    if( _p  != 0 ) delete _p ;
    if( _u  != 0 ) delete _u ;
    if( _w  != 0 ) delete _w ;

    return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
