/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
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

#ifndef ACADO_CMAKE_BUILD

/**
 *    \file src/control_law/exported_rti_scheme.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 20.08.2008
 */


#include <acado/control_law/exported_rti_scheme.hpp>


#ifdef __MATLAB__
extern "C"
{
	void preparationStep( ) {};
	void feedbackStep( real_t* ) {};
	void shiftControls( real_t* ) {};
	void shiftStates( real_t* ) {};
	
	real_t* getAcadoVariablesX( ) { return 0; };
	real_t* getAcadoVariablesU( ) { return 0; };
	real_t* getAcadoVariablesXRef( ) { return 0; };
	real_t* getAcadoVariablesURef( ) { return 0; };
} // extern "C"
#endif // __MATLAB__



BEGIN_NAMESPACE_ACADO



//
// PUBLIC MEMBER FUNCTIONS:
//

ExportedRTIscheme::ExportedRTIscheme( ) : ControlLaw( )
{
	setStatus( BS_NOT_INITIALIZED );
}


ExportedRTIscheme::ExportedRTIscheme(	uint _nX,
										uint _nU,
										uint _nPH,
										double _samplingTime
										) : ControlLaw( _samplingTime )
{
	nX  = _nX;
	nU  = _nU;
	nPH = _nPH;

	setStatus( BS_NOT_INITIALIZED );
}


ExportedRTIscheme::ExportedRTIscheme( const ExportedRTIscheme& rhs ) : ControlLaw( rhs )
{
	nX  = rhs.nX;
	nU  = rhs.nU;
	nPH = rhs.nPH;
}


ExportedRTIscheme::~ExportedRTIscheme( )
{
}


ExportedRTIscheme& ExportedRTIscheme::operator=( const ExportedRTIscheme& rhs )
{
	if ( this != &rhs )
	{
		ControlLaw::operator=( rhs );

		nX = rhs.nX;
		nU = rhs.nU;
		nPH = rhs.nPH;
	}

    return *this;
}


ControlLaw* ExportedRTIscheme::clone( ) const
{
	return new ExportedRTIscheme( *this );
}



returnValue ExportedRTIscheme::initializeControls( const VariablesGrid& _u_init )
{
	if ( _u_init.getNumValues( ) != getNU() )
		return ACADOERROR( RET_INPUT_DIMENSION_MISMATCH );
	
	u.init( getNU() );

	for( uint j=0; j<getNU( ); ++j )
		u(j) = _u_init( 0,j );
	
	return SUCCESSFUL_RETURN;
}


returnValue ExportedRTIscheme::initializeControls( const char* fileName )
{
	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}



returnValue ExportedRTIscheme::init(	double startTime,
										const Vector& _x,
										const Vector& _p,
										const VariablesGrid& _yRef
										)
{
// 	printf( "sizeof(acadoVariables) = %d\n",sizeof(acadoVariables) );
// 	printf( "sizeof(acadoWorkspace) = %d\n",sizeof(acadoWorkspace) );
// 	printf( "address acadoVariables = %p\n",&acadoVariables );
// 	printf( "address acadoWorkspace = %p\n",&acadoWorkspace );

	Grid refGrid( _yRef.getFirstTime(),_yRef.getLastTime(),nPH+1 );
	VariablesGrid yRefTmp = _yRef.getRefinedGrid( refGrid );
	//yRefTmp.print( );

   // INTRODUCE AUXILIARY VAIRABLES:
   // ------------------------------
	uint i, j;

   // INITIALIZE THE STATES AND CONTROLS:
   // ----------------------------------------
	for( i = 0; i < getNX(); ++i ) 
		getAcadoVariablesX()[i] = (real_t)_x(i);

// 	_x.print("xInit");
// 	yRefTmp.print("yRefInit");
	
	if ( u.isEmpty( ) == BT_TRUE )
	{
		u.init( getNU() );
		u.setZero( );
	}

	for( i = 0; i < nPH; i++ ) 
	{
		for( j = 0; j < getNU( ); ++j )
			getAcadoVariablesU()[i*getNU()+j] = (real_t)u(j);
	}

   // INITIALIZE THE STATES AND CONTROL REFERENCE:
   // --------------------------------------------
    for( i = 0; i < nPH; i++ )
	{
		for( j = 0; j < getNX( ); ++j )
			getAcadoVariablesXRef()[i*getNX()+j] = (real_t)yRefTmp( i,j );

		for( j = 0; j < getNU( ); ++j )
			getAcadoVariablesURef()[i*getNU()+j] = (real_t)yRefTmp( i,getNX()+j );
	}


	if ( preparationStep( startTime,_yRef ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );

	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}



returnValue ExportedRTIscheme::step(	double currentTime,
										const Vector& _x,
										const Vector& _p,
										const VariablesGrid& _yRef
										)
{
	if ( feedbackStep( currentTime,_x,_p ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );

	if ( preparationStep( currentTime,_yRef ) != SUCCESSFUL_RETURN )// TODO: change to nextTime later!
		return ACADOERROR( RET_CONTROLLAW_STEP_FAILED );

	return SUCCESSFUL_RETURN;
}


returnValue ExportedRTIscheme::feedbackStep(	double currentTime,
												const Vector &_x,
												const Vector &_p,
												const VariablesGrid& _yRef
												)
{
	uint i;
	
// 	_x.print("x");


	if ( getStatus( ) != BS_READY )
		return ACADOERROR( RET_BLOCK_NOT_READY );

	if ( _x.getDim( ) != getNX() )
		return ACADOERROR( RET_INVALID_ARGUMENTS );

	if ( _p.getDim( ) != getNP() )
		return ACADOERROR( RET_INVALID_ARGUMENTS );


	real_t* measurement = new real_t[getNX()];
	for( i=0; i<getNX(); ++i )
		measurement[i] = (real_t)_x(i);

	::feedbackStep( measurement );

	delete[] measurement;

	for( i=0; i<getNU(); ++i )
		u(i) = (real_t)getAcadoVariablesU()[i];

// 	u.print("u");

// 	printf("=================================================================\n\n" );
// 	printf("    Current Real-Time Iteration:  KKT Tolerance = %.3e\n", getKKT() );
// 	printf("\n=================================================================\n" );

	//shift( );

	return SUCCESSFUL_RETURN;
}


returnValue ExportedRTIscheme::preparationStep(	double nextTime,
												const VariablesGrid& _yRef
												)
{
	uint i,j;

	Grid refGrid( _yRef.getFirstTime(),_yRef.getLastTime(),nPH+1 );
	VariablesGrid yRefTmp = _yRef.getRefinedGrid( refGrid );

    for( i = 0; i < nPH; ++i )
	{
		for( j = 0; j < getNX( ); ++j )
			getAcadoVariablesXRef()[i*getNX()+j] = (real_t)yRefTmp( i,j );

		for( j = 0; j < getNU( ); ++j )
			getAcadoVariablesURef()[i*getNU()+j] = (real_t)yRefTmp( i,getNX()+j );
	}

	::preparationStep( );

	return SUCCESSFUL_RETURN;
}



returnValue ExportedRTIscheme::shift( )
{
// 	shiftControls( getAcadoVariablesURef() );
// 	shiftStates  ( getAcadoVariablesXRef() );

	return SUCCESSFUL_RETURN;
}



returnValue ExportedRTIscheme::setReference( const VariablesGrid &ref )
{
    if( getStatus() != BS_READY )
		return ACADOERROR( RET_OPTALG_INIT_FAILED );

    return ACADOERROR(RET_NOT_IMPLEMENTED_YET);
}



uint ExportedRTIscheme::getNX( ) const
{
	return nX;
}


uint ExportedRTIscheme::getNXA( ) const
{
	return 0;
}


uint ExportedRTIscheme::getNU( ) const
{
	return nU;
}


uint ExportedRTIscheme::getNP( ) const
{
	return 0;
}


uint ExportedRTIscheme::getNW( ) const
{
	return 0;
}


uint ExportedRTIscheme::getNY( ) const
{
	return getNX( );
}



double ExportedRTIscheme::getLengthPredictionHorizon( ) const
{
	return (double)nPH * samplingTime;
}


double ExportedRTIscheme::getLengthControlHorizon( ) const
{
	return getLengthPredictionHorizon( );
}



BooleanType ExportedRTIscheme::isDynamic( ) const
{
	return BT_TRUE;
}


BooleanType ExportedRTIscheme::isStatic( ) const
{
	if ( isDynamic() == BT_TRUE )
		return BT_FALSE;
	else
		return BT_TRUE;
}


BooleanType ExportedRTIscheme::isInRealTimeMode( ) const
{
	return BT_TRUE;
}



//
// PROTECTED MEMBER FUNCTIONS:
//

CLOSE_NAMESPACE_ACADO

#endif // ACADO_CMAKE_BUILD

// end of file.
