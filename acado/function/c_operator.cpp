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
*    \file src/symbolic_operator/c_operator.cpp
*    \author Boris Houska, Hans Joachim Ferreau
*    \date 2008
*/


#include <acado/utils/acado_utils.hpp>
#include <acado/function/c_function.hpp>
#include <acado/function/c_operator.hpp>



BEGIN_NAMESPACE_ACADO


int COperator::counter = 0;

COperator::COperator( ) :SmoothOperator( )
{
    result       =  0;
    d_result     =  0;
    cresult      =  0;
    d_cresult    =  0;
    component    = -1;
    bufferSize   =  0;
    idx          =  0;

    first        = BT_FALSE;
    globalTypeID = counter ;

    nCount = 0;
}


COperator::COperator( const CFunction &fcn, const Expression &arg, int component_ ) :SmoothOperator( )
{

    uint run1;

    cFunction = fcn;
    argument  = arg;

    component = component_;

    first         = BT_FALSE;
    globalTypeID  =  counter;

    idx =  new int[cFunction.getDim()];

    bufferSize = 1;

    result   = (double**)calloc(bufferSize,sizeof(double*));
    d_result = (double**)calloc(bufferSize,sizeof(double*));

    for( run1 = 0; run1 < bufferSize; run1++ ){
        result  [run1] = new double[argument.getDim()];
        d_result[run1] = new double[argument.getDim()];
    }

    cresult   = (double**)calloc(bufferSize,sizeof(double*));
    d_cresult = (double**)calloc(bufferSize,sizeof(double*));

    for( run1 = 0; run1 < bufferSize; run1++ ){
        cresult  [run1] = new double[cFunction.getDim()];
        d_cresult[run1] = new double[cFunction.getDim()];
    }

    nCount = 0;
}


COperator::COperator( const COperator &arg ){ copy(arg); }

COperator::~COperator(){ deleteAll(); }


COperator& COperator::operator=( const COperator &arg ){

    if( this != &arg ){

        deleteAll(   );
        copy     (arg);
    }
    return *this;
}


int COperator::increaseID(){ return counter++; }


void COperator::copy( const COperator &arg ){

    uint run1, run2;

    bufferSize     = arg.bufferSize   ;
    cFunction      = arg.cFunction    ;
    argument       = arg.argument     ;
    component      = arg.component    ;

    first          = arg.first        ;
    globalTypeID   = arg.globalTypeID ;

    idx = new int[cFunction.getDim()];
    for( run1 = 0; run1 < cFunction.getDim(); run1++ )
         idx[run1] = arg.idx[run1];

    result   = (double**)calloc(bufferSize,sizeof(double*));
    d_result = (double**)calloc(bufferSize,sizeof(double*));

    for( run1 = 0; run1 < bufferSize; run1++ ){

        result  [run1] = new double[argument.getDim()];
        d_result[run1] = new double[argument.getDim()];

        for( run2 = 0; run2 < argument.getDim(); run2++ ){
              result[run1][run2] = arg.result  [run1][run2];
            d_result[run1][run2] = arg.d_result[run1][run2];
        }
    }

    cresult   = (double**)calloc(bufferSize,sizeof(double*));
    d_cresult = (double**)calloc(bufferSize,sizeof(double*));

    for( run1 = 0; run1 < bufferSize; run1++ ){

        cresult  [run1] = new double[cFunction.getDim()];
        d_cresult[run1] = new double[cFunction.getDim()];

        for( run2 = 0; run2 < cFunction.getDim(); run2++ ){
              cresult[run1][run2] = arg.cresult  [run1][run2];
            d_cresult[run1][run2] = arg.d_cresult[run1][run2];
        }
    }

    nCount = 0;
}


void COperator::deleteAll(){

    uint run1;

    for( run1 = 0; run1 < bufferSize; run1++ ){
        delete[] result  [run1];
        delete[] d_result[run1];
    }

    free( result   );
    free( d_result );

    for( run1 = 0; run1 < bufferSize; run1++ ){
        delete[] cresult  [run1];
        delete[] d_cresult[run1];
    }

    free( cresult   );
    free( d_cresult );

    delete[] idx;
}


returnValue COperator::evaluate( int number, double *x, double *result_ ){

    uint run1;

    ASSERT( component >= 0 );

    if( first == BT_FALSE ){
        result_[0] = x[idx[component]];
    }
    else{

        if( number >= (int) bufferSize ){

            int oldSize = bufferSize;
            bufferSize += number;
            result    = (double**)realloc(result   ,bufferSize*sizeof(double*));
            d_result  = (double**)realloc(d_result ,bufferSize*sizeof(double*));
            cresult   = (double**)realloc(cresult  ,bufferSize*sizeof(double*));
            d_cresult = (double**)realloc(d_cresult,bufferSize*sizeof(double*));

            for( run1 = oldSize; run1 < bufferSize; run1++ ){
                result   [run1] = new double[argument.getDim() ];
                d_result [run1] = new double[argument.getDim() ];
                cresult  [run1] = new double[cFunction.getDim()];
                d_cresult[run1] = new double[cFunction.getDim()];
            }
        }

        for( run1 = 0; run1 < argument.getDim(); run1++ )
            argument.element[run1]->evaluate( number, x , &result[number][run1] );
        cFunction.evaluate( number, result[number], cresult[number] );
        result_[0] = cresult[number][component];
        for( run1 = 0; run1 < cFunction.getDim(); run1++ )
            x[idx[run1]] = cresult[number][run1];
    }
    return SUCCESSFUL_RETURN;
}


returnValue COperator::evaluate( EvaluationBase *x ){
 
    ASSERT( 1 == 0 );
    return ACADOERROR(RET_ASSERTION);
}



Operator* COperator::differentiate( int index ){

    ASSERT( 1 == 0 );
    ACADOERROR(RET_ASSERTION);
    return 0;
}



Operator* COperator::AD_forward( int dim,
                                     VariableType *varType,
                                     int *component_,
                                     Operator **seed,
                                     int &nNewIS,
                                     TreeProjection ***newIS ){

    ASSERT( 1 == 0 );
    ACADOERROR(RET_ASSERTION);
    return 0;
}


returnValue COperator::AD_backward( int           dim      , /**< number of directions  */
                                        VariableType *varType  , /**< the variable types    */
                                        int          *component_, /**< and their components  */
                                        Operator     *seed     , /**< the backward seed     */
                                        Operator    **df       , /**< the result            */
                                        int           &nNewIS  , /**< the number of new IS  */
                                        TreeProjection ***newIS_  /**< the new IS-pointer    */ ){

    ASSERT( 1 == 0 );
    return ACADOERROR(RET_ASSERTION);
}


returnValue COperator::AD_symmetric( int            dim       , /**< number of directions  */
                                        VariableType  *varType   , /**< the variable types    */
                                        int           *component_, /**< and their components  */
                                        Operator      *l         , /**< the backward seed     */
                                        Operator     **S         , /**< forward seed matrix   */
                                        int            dimS      , /**< dimension of forward seed             */
                                        Operator     **dfS       , /**< first order foward result             */
                                        Operator     **ldf       , /**< first order backward result           */
                                        Operator     **H         , /**< upper trianglular part of the Hessian */
                                      int            &nNewLIS  , /**< the number of newLIS  */
                                      TreeProjection ***newLIS , /**< the new LIS-pointer   */
                                      int            &nNewSIS  , /**< the number of newSIS  */
                                      TreeProjection ***newSIS , /**< the new SIS-pointer   */
                                      int            &nNewHIS  , /**< the number of newHIS  */
                                      TreeProjection ***newHIS   /**< the new HIS-pointer   */ ){
  
    ASSERT( 1 == 0 );
    return ACADOERROR(RET_ASSERTION);
}


Operator* COperator::substitute( int index, const Operator *sub ){

    ACADOERROR( RET_NOT_IMPLEMENTED_YET );
    return 0;
}


NeutralElement COperator::isOneOrZero() const{

    return NE_NEITHER_ONE_NOR_ZERO;
}



BooleanType COperator::isDependingOn( VariableType var ) const{

    uint run1;

    for( run1 = 0; run1 < argument.getDim(); run1++ )
        if( argument.element[run1]->isDependingOn(var) == BT_TRUE )
            return BT_TRUE;
    return BT_FALSE;
}


BooleanType COperator::isDependingOn( int dim,
                                        VariableType *varType,
                                        int *component_,
                                        BooleanType   *implicit_dep ){

    uint run1;

    for( run1 = 0; run1 < argument.getDim(); run1++ )
        if( argument.element[run1]->isDependingOn( dim, varType, component_, implicit_dep ) == BT_TRUE )
            return BT_TRUE;
    return BT_FALSE;
}


BooleanType COperator::isLinearIn( int dim,
                                     VariableType *varType,
                                     int *component_,
                                     BooleanType   *implicit_dep ){

    return BT_FALSE;
}


BooleanType COperator::isPolynomialIn( int dim,
                                         VariableType *varType,
                                         int *component_,
                                         BooleanType   *implicit_dep ){

    return BT_FALSE;
}


BooleanType COperator::isRationalIn( int dim,
                                       VariableType *varType,
                                       int *component_,
                                       BooleanType   *implicit_dep ){

    return BT_FALSE;
}


MonotonicityType COperator::getMonotonicity( ){

    return MT_NONMONOTONIC;
}


CurvatureType COperator::getCurvature( ){

    return CT_UNKNOWN;
}


returnValue COperator::setMonotonicity( MonotonicityType monotonicity_ ){

    return ACADOERROR( RET_UNKNOWN_BUG );
}


returnValue COperator::setCurvature( CurvatureType curvature_ ){

    return ACADOERROR( RET_UNKNOWN_BUG );
}


returnValue COperator::AD_forward( int number, double *x, double *seed,
                                     double *f, double *df ){

    uint run1;

    ASSERT( component >= 0 );

    if( first == BT_FALSE ){
         f[0] =    x[idx[component]];
        df[0] = seed[idx[component]];
    }
    else{

        if( number >= (int) bufferSize ){

            int oldSize = bufferSize;
            bufferSize += number;
            result    = (double**)realloc(result   ,bufferSize*sizeof(double*));
            d_result  = (double**)realloc(d_result ,bufferSize*sizeof(double*));
            cresult   = (double**)realloc(cresult  ,bufferSize*sizeof(double*));
            d_cresult = (double**)realloc(d_cresult,bufferSize*sizeof(double*));

            for( run1 = oldSize; run1 < bufferSize; run1++ ){
                result   [run1] = new double[argument.getDim() ];
                d_result [run1] = new double[argument.getDim() ];
                cresult  [run1] = new double[cFunction.getDim()];
                d_cresult[run1] = new double[cFunction.getDim()];
            }
        }

        for( run1 = 0; run1 < argument.getDim(); run1++ )
            argument.element[run1]->AD_forward( number, x, seed, &result[number][run1], &d_result[number][run1] );
        cFunction.AD_forward( number, result[number], d_result[number], cresult[number], d_cresult[number] );
         f[0] =   cresult[number][component];
        df[0] = d_cresult[number][component];
        for( run1 = 0; run1 < cFunction.getDim(); run1++ ){
            x   [idx[run1]] =   cresult[number][run1];
            seed[idx[run1]] = d_cresult[number][run1];
        }
    }
    return SUCCESSFUL_RETURN;
}



returnValue COperator::AD_forward( int number, double *seed, double *df ){

    uint run1;

    ASSERT( component >= 0 );

    if( first == BT_FALSE ){
        df[0] = seed[idx[component]];
    }
    else{
        for( run1 = 0; run1 < argument.getDim(); run1++ )
            argument.element[run1]->AD_forward( number, seed, &d_result[number][run1] );
        cFunction.AD_forward( number, d_result[number], d_cresult[number] );
        df[0] = d_cresult[number][component];
        for( run1 = 0; run1 < cFunction.getDim(); run1++ )
            seed[idx[run1]] = d_cresult[number][run1];
    }
    return SUCCESSFUL_RETURN;
}


returnValue COperator::AD_backward( int number, double seed, double *df ){

    return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
}


returnValue COperator::AD_forward2( int number, double *seed, double *dseed,
                                      double *df, double *ddf ){

    uint run1;

    ASSERT( component >= 0 );

    if( first == BT_FALSE ){
         df[0] =  seed[idx[component]];
        ddf[0] = dseed[idx[component]];
    }
    else{

        double *d_result2  = new double[argument.getDim() ];
        double *d_cresult2 = new double[cFunction.getDim()];

        double *dd_result  = new double[argument.getDim() ];
        double *dd_cresult = new double[cFunction.getDim()];

        for( run1 = 0; run1 < argument.getDim(); run1++ )
            argument.element[run1]->AD_forward2( number, seed, dseed, &d_result2[run1], &dd_result[run1] );

        cFunction.AD_forward2( number, d_result2, dd_result, d_cresult2, dd_cresult );

         df[0] = d_cresult2[component];
        ddf[0] = dd_cresult[component];

        for( run1 = 0; run1 < cFunction.getDim(); run1++ ){
             seed[idx[run1]] = d_cresult2[run1];
            dseed[idx[run1]] = dd_cresult[run1];
        }

        delete[] d_result2  ;
        delete[] d_cresult2 ;
        delete[] dd_result  ;
        delete[] dd_cresult ;
    }
    return SUCCESSFUL_RETURN;
}


returnValue COperator::AD_backward2( int number, double seed1, double seed2,
                                       double *df, double *ddf ){

    return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
}


std::ostream& COperator::print( std::ostream& stream ) const
{
    return stream << "C functions can not be printed";
}



Operator* COperator::clone() const{

    return new COperator(*this);
}


returnValue COperator::clearBuffer(){

    if( bufferSize > 1 ){
        bufferSize = 1;
        result    = (double**)realloc(result   ,bufferSize*sizeof(double*));
        d_result  = (double**)realloc(d_result ,bufferSize*sizeof(double*));
        cresult   = (double**)realloc(cresult  ,bufferSize*sizeof(double*));
        d_cresult = (double**)realloc(d_cresult,bufferSize*sizeof(double*));
    }
    return cFunction.clearBuffer();
}


returnValue COperator::enumerateVariables( SymbolicIndexList *indexList ){

    uint run1;
    returnValue returnvalue;

    for( run1 = 0; run1 < argument.getDim(); run1++ ){

        returnvalue = argument.element[run1]->enumerateVariables( indexList );
        if( returnvalue != SUCCESSFUL_RETURN ) return returnvalue;
    }

    first = indexList->determineCExpressionIndices( cFunction.getDim(), globalTypeID, idx );

    return SUCCESSFUL_RETURN;
}


OperatorName COperator::getName(){

    return ON_CEXPRESSION;
}


BooleanType COperator::isVariable( VariableType &varType, int &component_ ) const{

    return BT_FALSE;
}


returnValue COperator::loadIndices( SymbolicIndexList *indexList ){

    uint run1;
    returnValue returnvalue;

    for( run1 = 0; run1 < argument.getDim(); run1++ ){
        returnvalue = argument.element[run1]->loadIndices( indexList );
        if( returnvalue != SUCCESSFUL_RETURN ) return returnvalue;
    }

    return SUCCESSFUL_RETURN;
}


BooleanType COperator::isSymbolic() const{

    return BT_FALSE;
}


CLOSE_NAMESPACE_ACADO

// end of file.
