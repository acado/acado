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
 *    \file src/curve/curve.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 */


#include <acado/curve/curve.hpp>



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

Curve::Curve( ){

    nIntervals       = 0;
    dim              = 0;
    parameterization = 0;
    grid             = 0;
}

Curve::Curve( const Curve& arg ){

    uint run1;

    nIntervals       = arg.nIntervals;
    dim              = arg.dim       ;

    parameterization = (Function**)calloc(nIntervals,sizeof(Function*));

    for( run1 = 0; run1 < nIntervals; run1++ ){
        if( arg.parameterization[run1] != 0 ) parameterization[run1] = new Function(*arg.parameterization[run1]);
        else                                  parameterization[run1] = 0;
    }

    if( arg.grid != 0 )  grid = new Grid(*arg.grid);
    else                 grid = 0;
}


Curve::~Curve( ){

    uint run1;

    for( run1 = 0; run1 < nIntervals; run1++ )
        if( parameterization[run1] != 0 )
            delete parameterization[run1];

     if( parameterization != 0 ) free(parameterization);
     if( grid != 0 ) delete grid;
}


Curve& Curve::operator=( const Curve& arg ){

    uint run1;

    if ( this != &arg ){

        for( run1 = 0; run1 < nIntervals; run1++ )
            if( parameterization[run1] != 0 )
                delete parameterization[run1];

        if( parameterization != 0 ) free(parameterization);
        if( grid != 0 ) delete grid;

        nIntervals       = arg.nIntervals;
        dim              = arg.dim       ;
        parameterization = (Function**)calloc(nIntervals,sizeof(Function*));

        for( run1 = 0; run1 < nIntervals; run1++ ){
            if( arg.parameterization[run1] != 0 ) parameterization[run1] = new Function(*arg.parameterization[run1]);
            else                                  parameterization[run1] = 0;
        }

        if( arg.grid != 0 )  grid = new Grid(*arg.grid);
        else                 grid = 0;
    }
    return *this;
}


Curve Curve::operator()(	uint idx
							) const
{
	Curve tmp;

	if ( (int)idx >= getDim() )
	{
		ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );
		return tmp;
	}

    uint run1;

	tmp.nIntervals       = nIntervals;
	tmp.dim              = dim         ; // -> 1!
	tmp.parameterization = (Function**)calloc(nIntervals,sizeof(Function*));

	for( run1 = 0; run1 < nIntervals; run1++ ){
		if( parameterization[run1] != 0 ) tmp.parameterization[run1] = new Function( parameterization[run1]->operator()( idx ) );
		else                              tmp.parameterization[run1] = 0;
	}

	if( grid != 0 )  tmp.grid = new Grid(*grid);
	else             tmp.grid = 0;

	return tmp;
}


returnValue Curve::add( double tStart, double tEnd, const DVector constant ){

    uint     run1;
    Function tmp ;

    // CONSTRUCT SIMPLY A CONSTANT FUNCTION AND ADD IT:
    // ------------------------------------------------

    for( run1 = 0; run1 < constant.getDim(); run1++ )
         tmp << constant(run1);

    return add( tStart, tEnd, tmp );
}


returnValue Curve::add( const VariablesGrid& sampledData, InterpolationMode mode ){

    uint         run1,run2  ;
    returnValue  returnvalue;
    Function    *tmp        ;
    TIME         t          ;
    double       m,b        ;

    switch( mode ){

        case IM_CONSTANT:
             for( run1 = 0; run1 < sampledData.getNumPoints()-1; run1++ ){
                 tmp = new Function();
                 for( run2 = 0; run2 < sampledData.getNumRows(); run2++ )
                     tmp->operator<<( sampledData(run1,run2) );
                 returnvalue = add( sampledData.getTime(run1), sampledData.getTime(run1+1), *tmp );
                 if( returnvalue != SUCCESSFUL_RETURN )
                     ACADOERROR(returnvalue);
                 delete tmp;
             }
             return SUCCESSFUL_RETURN;


        case IM_LINEAR:
             for( run1 = 0; run1 < sampledData.getNumPoints()-1; run1++ ){
                 tmp = new Function();
                 for( run2 = 0; run2 < sampledData.getNumRows(); run2++ ){

                     m = (sampledData(run1+1,run2)    - sampledData(run1,run2)   )/
                         (sampledData.getTime(run1+1) - sampledData.getTime(run1));

                     b = sampledData(run1,run2) - m*sampledData.getTime(run1);

                     tmp->operator<<( m*t + b );
                 }
                 returnvalue = add( sampledData.getTime(run1), sampledData.getTime(run1+1), *tmp );
                 if( returnvalue != SUCCESSFUL_RETURN )
                     ACADOERROR(returnvalue);
                 delete tmp;
             }
             return SUCCESSFUL_RETURN;


        case IM_QUADRATIC:
             return ACADOERROR(RET_NOT_IMPLEMENTED_YET);

        case IM_CUBIC:
             return ACADOERROR(RET_NOT_IMPLEMENTED_YET);

        default:
             return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
    }
    return SUCCESSFUL_RETURN;
}



returnValue Curve::add( double tStart, double tEnd, const Function &parameterization_ ){

    uint run1;

    // CHECK WHETHER  "tStart < tEnd": 
    // ------------------------------------------------
    if( acadoIsStrictlyGreater( tStart,tEnd ) == BT_TRUE )
        return ACADOERROR(RET_TIME_INTERVAL_NOT_VALID);


    // CHECK WHETHER THE INPUT VECTOR IS EMPTY: 
    // ------------------------------------------------
    if( parameterization_.getDim() == 0 )
        return ACADOERROR(RET_INPUT_DIMENSION_MISMATCH);



    if( isEmpty() == BT_FALSE ){

        // IF THE CURVE IS NOT EMPTY THE DIMENSIONS MUST BE CHECKED:
        // ---------------------------------------------------------
        if( getDim() != (int) parameterization_.getDim() )
            return ACADOERROR(RET_INPUT_DIMENSION_MISMATCH);

        // CHECK WHETHER THE CURVE HAS NO GAP's:
        // ---------------------------------------------------------
        if( acadoIsEqual( tStart,grid->getLastTime() ) == BT_FALSE )
		{
			ASSERT(1==0);
            return ACADOERROR(RET_TIME_INTERVAL_NOT_VALID);
		}

        // APPEND THE NEW TIME INTERVAL TO THE GRID:
        // ---------------------------------------------------------

        double *times = new double[nIntervals+2];
        for( run1 = 0; run1 < nIntervals+1; run1++ )        
            times[run1] = grid->getTime(run1);
        times[nIntervals+1] = tEnd;

        delete grid;
        grid = new Grid( nIntervals+2, times );

        nIntervals++;
        delete[] times;

        // ---------------------------------------------------------
    }
    else{

        // SETUP A NEW GRID:
        // ----------------------------------------------------

        grid = new Grid( tStart, tEnd, 2 );
        nIntervals++;

        // ----------------------------------------------------
    }


    // CHECK WHETHER THE FUNCTION ITSELF IS VALID:
    // -------------------------------------------
    if( parameterization_.getNX () != 0 ||
        parameterization_.getNXA() != 0 ||
        parameterization_.getNP () != 0 ||
        parameterization_.getNPI() != 0 ||
        parameterization_.getNU () != 0 ||
        parameterization_.getNUI() != 0 ||
        parameterization_.getNW()  != 0 ){
            return ACADOERROR(RET_INPUT_DIMENSION_MISMATCH);
    }


    // SET THE DIMENSION OF THE CURVE:
    // (the correctness of the dimension has already been cecked)
    // ----------------------------------------------------------

    dim = parameterization_.getDim();


    // ALLOCATE MEMORY FOR THE NEW PIECE OF CURVE:
    // -------------------------------------------

    parameterization = (Function**)realloc(parameterization,nIntervals*sizeof(Function*));


    // MAKE A DEEP COPY OF THE PARAMETERIZATION:
    // -----------------------------------------

    parameterization[nIntervals-1] = new Function(parameterization_);


    // RETURN:
    // -----------------------------------------

    return SUCCESSFUL_RETURN;
}


returnValue Curve::evaluate( double t, double *result ) const{

    uint        idx        ;
    returnValue returnvalue;

    // CHECK WHETHER THE CURVE IS EMPTY:
    // ---------------------------------
    if( isEmpty() == BT_TRUE )
        return ACADOERROR(RET_MEMBER_NOT_INITIALISED);


    // CHECK WHETHER THE TIME  t  IS IN THE DOMAIN OF THE CURVE: 
    // ---------------------------------------------------------
    if( (t > grid->getLastTime() + 100.0*EPS) || (t < grid->getFirstTime() - 100.0*EPS) )
        return ACADOERROR(RET_INVALID_TIME_POINT);


    // CHECK WHETHER THE ARGUMENT IS THE NULL POINTER:
    // -----------------------------------------------
    if( result == 0 )
        return ACADOERROR(RET_INVALID_ARGUMENTS);


    // OBTAIN THE INTERVAL INDEX:
    // --------------------------

    idx = grid->getFloorIndex(t);
    if( idx == nIntervals ) idx--;


    // EVALUATE THE FUNCTION ASSOCIATED WITH THIS INTERVAL:
    // ----------------------------------------------------
    double tt[1] = { t };
    returnvalue = parameterization[idx]->evaluate(0,tt,result);


    if( returnvalue != SUCCESSFUL_RETURN )
        return ACADOERROR(returnvalue);

    return SUCCESSFUL_RETURN;
}


returnValue Curve::evaluate( double t, DVector &result ) const{

    uint        run1       ;
    returnValue returnvalue;

    double *tmp = new double[dim];

    returnvalue = evaluate( t, tmp );

    if( returnvalue != SUCCESSFUL_RETURN )
        return returnvalue;

    result.init(dim);
    for( run1 = 0; run1 < dim; run1++ )
        result(run1) = tmp[run1];

     delete[] tmp;

    return SUCCESSFUL_RETURN;
}


returnValue Curve::evaluate( double tStart, double tEnd, VariablesGrid &result ) const
{
	// determine sub grid of intervals with given time horizon [tStart,tEnd]
	Grid intervalsSubGrid;

	if ( grid->getSubGrid( tStart,tEnd,intervalsSubGrid ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNKNOWN_BUG );

	return discretize( intervalsSubGrid,result );
}



returnValue Curve::discretize( const Grid &discretizationGrid, VariablesGrid &result ) const{

    uint        run1       ;
    returnValue returnvalue;
    DVector      tmp        ;

    result.init( dim, discretizationGrid );

    for( run1 = 0; run1 < discretizationGrid.getNumPoints(); run1++ ){
        returnvalue = evaluate( discretizationGrid.getTime(run1), tmp );
        if( returnvalue != SUCCESSFUL_RETURN )
            return returnvalue;
        result.setVector(run1,tmp);
    }
    return SUCCESSFUL_RETURN;
}


returnValue Curve::getTimeDomain( double tStart, double tEnd ) const
{
    // CHECK WHETHER THE CURVE IS EMPTY:
    // ---------------------------------
    if( isEmpty() == BT_TRUE )
        return ACADOERROR(RET_MEMBER_NOT_INITIALISED);

    tStart = grid->getFirstTime();
    tEnd   = grid->getLastTime() ;

    return SUCCESSFUL_RETURN;
}


returnValue Curve::getTimeDomain( const uint &idx, double tStart, double tEnd ) const
{
    // CHECK WHETHER THE CURVE IS EMPTY:
    // ---------------------------------
    if( isEmpty() == BT_TRUE )
        return ACADOERROR(RET_MEMBER_NOT_INITIALISED);


    // CHECK WHETHER THE INDEX IS IN THE PERMISSIBLE RANGE:
    // ----------------------------------------------------
    if( idx >= grid->getNumIntervals() )
        return ACADOERROR(RET_INDEX_OUT_OF_BOUNDS);


    tStart = grid->getTime(idx  );
    tEnd   = grid->getTime(idx+1);

    return SUCCESSFUL_RETURN;
}


BooleanType Curve::isInTimeDomain(	double t
									) const
{
	if( isEmpty( ) == BT_TRUE )
		return BT_FALSE;

	if ( ( acadoIsGreater( t,grid->getFirstTime() ) == BT_TRUE ) &&
		 ( acadoIsSmaller( t,grid->getLastTime()  ) == BT_TRUE ) )
		return BT_TRUE;
	else
		return BT_FALSE;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


CLOSE_NAMESPACE_ACADO

/*
 *    end of file
 */
