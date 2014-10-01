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
 *    \file src/objective/objective.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *
 */


#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/objective/objective.hpp>
#include <acado/function/ocp_iterate.hpp>

#include <algorithm>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//


Objective::Objective( )
          :LagrangeTerm(){

    lsqTerm    = 0;
    lsqEndTerm = 0;
    mayerTerm  = 0;

    nLSQ       = 0;
    nEndLSQ    = 0;
    nMayer     = 0;
}


Objective::Objective( const Grid &grid_ ):LagrangeTerm(){

    lsqTerm    = 0;
    lsqEndTerm = 0;
    mayerTerm  = 0;

    nLSQ       = 0;
    nEndLSQ    = 0;
    nMayer     = 0;

    init( grid_ );
}



returnValue Objective::init( const Grid &grid_ ){

    return LagrangeTerm::init( grid_ );
}



Objective::Objective( const Objective& rhs )
          :LagrangeTerm(rhs){

    uint run1;

    nLSQ       = rhs.nLSQ   ;
    nEndLSQ    = rhs.nEndLSQ;
    nMayer     = rhs.nMayer ;

    if( rhs.lsqTerm != 0 ){
        lsqTerm = (LSQTerm**)calloc(nLSQ,sizeof(LSQTerm*));
        for( run1 = 0; run1 < nLSQ; run1++ )
            lsqTerm[run1] = new LSQTerm(*rhs.lsqTerm[run1]);
    }
    else lsqTerm = 0;

    if( rhs.lsqEndTerm != 0 ){
        lsqEndTerm = (LSQEndTerm**)calloc(nEndLSQ,sizeof(LSQEndTerm*));
        for( run1 = 0; run1 < nEndLSQ; run1++ )
            lsqEndTerm[run1] = new LSQEndTerm(*rhs.lsqEndTerm[run1]);
    }
    else lsqEndTerm = 0;

    if( rhs.mayerTerm != 0 ){
        mayerTerm = (MayerTerm**)calloc(nMayer,sizeof(MayerTerm*));
        for( run1 = 0; run1 < nMayer; run1++ )
            mayerTerm[run1] = new MayerTerm(*rhs.mayerTerm[run1]);
    }
    else mayerTerm = 0;

    cgLsqElements = rhs.cgLsqElements;
    cgLsqEndTermElements = rhs.cgLsqEndTermElements;

    cgExternLsqElements = rhs.cgExternLsqElements;
    cgExternLsqEndTermElements = rhs.cgExternLsqEndTermElements;

    cgLsqLinearElements = rhs.cgLsqLinearElements;
}


Objective::~Objective( ){

    uint run1;

    if( lsqTerm != 0 ){
        for( run1 = 0; run1 < nLSQ; run1++ )
            delete lsqTerm[run1];
        free(lsqTerm);
    }

    if( lsqEndTerm != 0 ){
        for( run1 = 0; run1 < nEndLSQ; run1++ )
            delete lsqEndTerm[run1];
        free(lsqEndTerm);
    }

    if( mayerTerm != 0 ){
        for( run1 = 0; run1 < nMayer; run1++ )
            delete mayerTerm[run1];
        free(mayerTerm);
    }

    cgExternLsqElements.clear();
    cgLsqEndTermElements.clear();

    cgExternLsqElements.clear();
    cgExternLsqEndTermElements.clear();

    cgLsqLinearElements.clear();
}

returnValue Objective::addLSQ( const MatrixVariablesGrid *S_,
                                      const Function&            h ,
                                      const VariablesGrid       *r_  )
{
    nLSQ++;
    lsqTerm = (LSQTerm**)realloc(lsqTerm,nLSQ*sizeof(LSQTerm*));
    lsqTerm[nLSQ-1] = new LSQTerm(S_, h, r_);
    lsqTerm[nLSQ-1]->setGrid(grid);

    DMatrix temp = S_->getMatrix( 0 );
    if (temp.isPositiveSemiDefinite() == BT_FALSE)
    	return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

    cgLsqElements.push_back(LsqData(temp, h));

    return SUCCESSFUL_RETURN;
}



returnValue Objective::addLSQEndTerm( const DMatrix   & S,
                                             const Function & m,
                                             const DVector   & r  )
{

    nEndLSQ++;
    lsqEndTerm = (LSQEndTerm**)realloc(lsqEndTerm,nEndLSQ*sizeof(LSQEndTerm*));
    lsqEndTerm[nEndLSQ-1] = new LSQEndTerm(grid, S, m, r);
    lsqEndTerm[nEndLSQ-1]->setGrid(grid);

    cgLsqEndTermElements.push_back(LsqData(S, m));

    return SUCCESSFUL_RETURN;
}

Objective& Objective::operator=( const Objective& rhs ){

    uint run1;

    if( this != &rhs ){

        if( lsqTerm != 0 ){
            for( run1 = 0; run1 < nLSQ; run1++ )
                delete lsqTerm[run1];
            free(lsqTerm);
        }

        if( lsqEndTerm != 0 ){
            for( run1 = 0; run1 < nEndLSQ; run1++ )
                delete lsqEndTerm[run1];
            free(lsqEndTerm);
        }

        if( mayerTerm != 0 ){
            for( run1 = 0; run1 < nMayer; run1++ )
                delete mayerTerm[run1];
            free(mayerTerm);
        }

        LagrangeTerm::operator=(rhs);

        nLSQ       = rhs.nLSQ   ;
        nEndLSQ    = rhs.nEndLSQ;
        nMayer     = rhs.nMayer ;

        if( rhs.lsqTerm != 0 ){
            lsqTerm = (LSQTerm**)calloc(nLSQ,sizeof(LSQTerm*));
            for( run1 = 0; run1 < nLSQ; run1++ )
                lsqTerm[run1] = new LSQTerm(*rhs.lsqTerm[run1]);
        }
        else lsqTerm = 0;

        if( rhs.lsqEndTerm != 0 ){
           lsqEndTerm = (LSQEndTerm**)calloc(nEndLSQ,sizeof(LSQEndTerm*));
            for( run1 = 0; run1 < nEndLSQ; run1++ )
                lsqEndTerm[run1] = new LSQEndTerm(*rhs.lsqEndTerm[run1]);
        }
        else lsqEndTerm = 0;

        if( rhs.mayerTerm != 0 ){
            mayerTerm = (MayerTerm**)calloc(nMayer,sizeof(MayerTerm*));
            for( run1 = 0; run1 < nMayer; run1++ )
                mayerTerm[run1] = new MayerTerm(*rhs.mayerTerm[run1]);
        }
        else mayerTerm = 0;

        cgLsqElements = rhs.cgLsqElements;
        cgLsqEndTermElements = rhs.cgLsqEndTermElements;

        cgExternLsqElements = rhs.cgExternLsqElements;
        cgExternLsqEndTermElements = rhs.cgExternLsqEndTermElements;

        cgLsqLinearElements = rhs.cgLsqLinearElements;
    }
    return *this;
}



returnValue Objective::evaluate( const OCPiterate &x ){

    uint run1;

    for( run1 = 0; run1 < nLSQ; run1++ )
        ACADO_TRY( lsqTerm[run1]->evaluate( x ) );

    for( run1 = 0; run1 < nEndLSQ; run1++ )
        ACADO_TRY( lsqEndTerm[run1]->evaluate( x ) );

    for( run1 = 0; run1 < nMayer; run1++ )
        ACADO_TRY( mayerTerm[run1]->evaluate( x ) );

    return SUCCESSFUL_RETURN;
}


returnValue Objective::evaluateSensitivities(){

    returnValue returnvalue;
    uint run1;

    for( run1 = 0; run1 < nLSQ; run1++ ){
        returnvalue = lsqTerm[run1]->evaluateSensitivities( 0 );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    for( run1 = 0; run1 < nEndLSQ; run1++ ){
        returnvalue = lsqEndTerm[run1]->evaluateSensitivities( 0 );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    for( run1 = 0; run1 < nMayer; run1++ ){
        returnvalue = mayerTerm[run1]->evaluateSensitivities( 0 );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    return SUCCESSFUL_RETURN;
}


returnValue Objective::evaluateSensitivities( BlockMatrix &hessian ){

    returnValue returnvalue;
    uint run1;

    for( run1 = 0; run1 < nLSQ; run1++ ){
        returnvalue = lsqTerm[run1]->evaluateSensitivities( &hessian );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    for( run1 = 0; run1 < nEndLSQ; run1++ ){
        returnvalue = lsqEndTerm[run1]->evaluateSensitivities( &hessian );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    for( run1 = 0; run1 < nMayer; run1++ ){
        returnvalue = mayerTerm[run1]->evaluateSensitivities( &hessian );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    return SUCCESSFUL_RETURN;
}


returnValue Objective::evaluateSensitivitiesGN( BlockMatrix &hessian ){

    returnValue returnvalue;
    uint run1;

	hessian.setZero();
	if( nMayer != 0 )
		return ACADOERROR(RET_GAUSS_NEWTON_APPROXIMATION_NOT_SUPPORTED);

	for( run1 = 0; run1 < nLSQ; run1++ ){
        returnvalue = lsqTerm[run1]->evaluateSensitivitiesGN( &hessian );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }
    for( run1 = 0; run1 < nEndLSQ; run1++ ){
        returnvalue = lsqEndTerm[run1]->evaluateSensitivitiesGN( &hessian );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    return SUCCESSFUL_RETURN;
}


returnValue Objective::setForwardSeed(  BlockMatrix *xSeed_ ,
                                        BlockMatrix *xaSeed_,
                                        BlockMatrix *pSeed_ ,
                                        BlockMatrix *uSeed_ ,
                                        BlockMatrix *wSeed_ ,
                                        int          order    ){

    returnValue returnvalue;
    uint run1;

    for( run1 = 0; run1 < nLSQ; run1++ ){
        returnvalue = lsqTerm[run1]->setForwardSeed( xSeed_, xaSeed_, pSeed_, uSeed_, wSeed_, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    for( run1 = 0; run1 < nEndLSQ; run1++ ){
        returnvalue = lsqEndTerm[run1]->setForwardSeed( xSeed_, xaSeed_, pSeed_, uSeed_, wSeed_, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    for( run1 = 0; run1 < nMayer; run1++ ){
        returnvalue = mayerTerm[run1]->setForwardSeed( xSeed_, xaSeed_, pSeed_, uSeed_, wSeed_, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    return SUCCESSFUL_RETURN;
}


returnValue Objective::setBackwardSeed( BlockMatrix *seed,  int order ){

    returnValue returnvalue;
    uint run1;

    for( run1 = 0; run1 < nLSQ; run1++ ){
        returnvalue = lsqTerm[run1]->setBackwardSeed( seed, 1 );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    for( run1 = 0; run1 < nEndLSQ; run1++ ){
        returnvalue = lsqEndTerm[run1]->setBackwardSeed( seed, 1 );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    for( run1 = 0; run1 < nMayer; run1++ ){
        returnvalue = mayerTerm[run1]->setBackwardSeed( seed, 1 );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
    }

    return SUCCESSFUL_RETURN;
}


returnValue Objective::setUnitBackwardSeed( ){

    BlockMatrix seed(1,1);
    seed.setIdentity(0,0,1);
    return setBackwardSeed(&seed,1);
}


returnValue Objective::getObjectiveValue( double &objectiveValue ){

    double obj;

    returnValue returnvalue;
    uint run1;

    objectiveValue = 0.0;

    for( run1 = 0; run1 < nLSQ; run1++ ){
        returnvalue = lsqTerm[run1]->getObjectiveValue( obj );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        objectiveValue += obj;
    }

    for( run1 = 0; run1 < nEndLSQ; run1++ ){
        returnvalue = lsqEndTerm[run1]->getObjectiveValue( obj );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        objectiveValue += obj;
    }

    for( run1 = 0; run1 < nMayer; run1++ ){
        returnvalue = mayerTerm[run1]->getObjectiveValue( obj );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        objectiveValue += obj;
    }

    return SUCCESSFUL_RETURN;
}


returnValue Objective::getForwardSensitivities( BlockMatrix &D, int order ){

    BlockMatrix DD, *sum;

    returnValue returnvalue;
    uint run1;

    sum = 0;

    for( run1 = 0; run1 < nLSQ; run1++ ){
        returnvalue = lsqTerm[run1]->getForwardSensitivities( &DD, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        if( sum != 0 ) sum[0] += DD;
        else           sum     = new BlockMatrix(DD);
    }

    for( run1 = 0; run1 < nEndLSQ; run1++ ){
        returnvalue = lsqEndTerm[run1]->getForwardSensitivities( &DD, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        if( sum != 0 ) sum[0] += DD;
        else           sum     = new BlockMatrix(DD);
    }

    for( run1 = 0; run1 < nMayer; run1++ ){
        returnvalue = mayerTerm[run1]->getForwardSensitivities( &DD, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        if( sum != 0 ) sum[0] += DD;
        else           sum     = new BlockMatrix(DD);
    }

    if( sum != 0 ){
        D = sum[0];
        delete sum;
    }

    return SUCCESSFUL_RETURN;
}


returnValue Objective::getBackwardSensitivities( BlockMatrix &D, int order ){

    BlockMatrix DD, *sum;

    returnValue returnvalue;
    uint run1;

    sum = 0;

    for( run1 = 0; run1 < nLSQ; run1++ ){
        returnvalue = lsqTerm[run1]->getBackwardSensitivities( &DD, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        if( sum != 0 ) sum[0] += DD;
        else           sum     = new BlockMatrix(DD);
    }

    for( run1 = 0; run1 < nEndLSQ; run1++ ){
        returnvalue = lsqEndTerm[run1]->getBackwardSensitivities( &DD, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        if( sum != 0 ) sum[0] += DD;
        else           sum     = new BlockMatrix(DD);
    }

    for( run1 = 0; run1 < nMayer; run1++ ){
        returnvalue = mayerTerm[run1]->getBackwardSensitivities( &DD, order );
        if( returnvalue != SUCCESSFUL_RETURN )  return returnvalue;
        if( sum != 0 ) sum[0] += DD;
        else           sum     = new BlockMatrix(DD);
    }

    if( sum != 0 ){
        D = sum[0];
        delete sum;
    }

    return SUCCESSFUL_RETURN;
}


returnValue Objective::init( const int              nStages     ,
                             const int              nTransitions,
                             DifferentialEquation **fcn         ,
                             Transition            *transitions ,
                             Constraint            *constraint_   ){

    if( lagrangeFcn != 0 ){

        int run1;

        if( fcn == 0 ){
            fcn = new DifferentialEquation*[nStages];
            for( run1 = 0; run1 < nStages; run1++ )
                fcn[run1] = new DifferentialEquation();
        }

        DifferentialState xAugment;

        for( run1 = 0; run1 < nTransitions; run1++ )
            transitions[run1] << xAugment == xAugment;

        for( run1 = 0; run1 < nStages; run1++ ){

            if( fcn[run1]->isDiscretized() == BT_FALSE ){
                if( nLagrangeTerms == 1 ){
                    fcn[run1][0] << dot( xAugment ) == *lagrangeFcn[0];
                }
                else{
                    if( lagrangeFcn[run1] == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);
                    fcn[run1][0] << dot( xAugment ) == (*lagrangeFcn[run1]);
                }
            }
            else{
                if( nLagrangeTerms == 1 ){

                    Expression tmp =  xAugment + fcn[run1]->getStepLength()*(*lagrangeFcn[0]);

                    fcn[run1][0] << next( xAugment ) == tmp;
                }
                else{
                    if( lagrangeFcn[run1] == 0 ) return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

                    Expression tmp = xAugment + fcn[run1]->getStepLength()*(*lagrangeFcn[run1]);

                    fcn[run1][0] << next( xAugment ) == tmp;
                }
            }
        }

		constraint_[0].add(0, 0.0, xAugment, 0.0);

        nMayer++;
        mayerTerm = (MayerTerm**)realloc(mayerTerm,nMayer*sizeof(MayerTerm*));
        mayerTerm[nMayer-1] = new MayerTerm(grid, xAugment );
    }

    return SUCCESSFUL_RETURN;
}


BooleanType Objective::isEmpty() const{

    if( nLSQ == 0 && nEndLSQ == 0 && nMayer == 0 && lagrangeFcn == 0 )
        return BT_TRUE;

    return BT_FALSE;
}

returnValue Objective::getLSQTerms( LsqElements& _elements ) const
{
	_elements = cgLsqElements;

	return SUCCESSFUL_RETURN;
}

returnValue Objective::getLSQEndTerms( LsqElements& _elements ) const
{
	_elements = cgLsqEndTermElements;

	return SUCCESSFUL_RETURN;
}

returnValue Objective::getLSQTerms( LsqExternElements& _elements ) const
{
	_elements = cgExternLsqElements;

	return SUCCESSFUL_RETURN;
}

returnValue Objective::getLSQEndTerms( LsqExternElements& _elements ) const
{
	_elements = cgExternLsqEndTermElements;

	return SUCCESSFUL_RETURN;
}

returnValue Objective::getLSQLinearTerms(LsqLinearElements& _elements) const
{
	_elements = cgLsqLinearElements;

	return SUCCESSFUL_RETURN;
}

uint Objective::getNumMayerTerms( ) const
{

	return nMayer;
}

returnValue Objective::getMayerTerm(uint index, Function& _mayerTerm) const
{
	mayerTerm[index]->getFunction(_mayerTerm);

	return SUCCESSFUL_RETURN;
}

uint Objective::getNumLagrangeTerms( ) const
{

	return nLagrangeTerms;
}

returnValue Objective::getLagrangeTerm(uint index, Function& _lagrangeTerm) const
{
	_lagrangeTerm << *lagrangeFcn[index];

	return SUCCESSFUL_RETURN;
}

//
// Code generation related functions
//

returnValue Objective::addLSQ(const DMatrix& S, const Function& h)
{
	cgLsqElements.push_back(LsqData(S, h));

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQEndTerm(const DMatrix& S, const Function& h)
{
	cgLsqEndTermElements.push_back(LsqData(S, h));

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQ(const DMatrix& S, const std::string& h)
{
	cgExternLsqElements.push_back(LsqExternData(S, h));

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQEndTerm(const DMatrix& S, const std::string& h)
{
	cgExternLsqEndTermElements.push_back(LsqExternData(S, h));

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQ(const BMatrix& S, const Function& h)
{
	cgLsqElements.push_back(LsqData(S.cast<double>(), h, false));

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQEndTerm(const BMatrix& S, const Function& h)
{
	cgLsqEndTermElements.push_back(LsqData(S.cast<double>(), h, false));

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQ(const BMatrix& S, const std::string& h)
{
	cgExternLsqElements.push_back(LsqExternData(S.cast<double>(), h, false));

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQEndTerm(const BMatrix& S, const std::string& h)
{
	cgExternLsqEndTermElements.push_back(LsqExternData(S.cast<double>(), h, false));

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQLinearTerms(const DVector& Slx, const DVector& Slu)
{
	cgLsqLinearElements.push_back( LsqLinearData(Slx, Slu, true) );

	return SUCCESSFUL_RETURN;
}

returnValue Objective::addLSQLinearTerms(const BVector& Slx, const BVector& Slu)
{
	cgLsqLinearElements.push_back( LsqLinearData(Slx.cast<double>(), Slu.cast<double>(), false) );

	return SUCCESSFUL_RETURN;
}

CLOSE_NAMESPACE_ACADO

// end of file.
