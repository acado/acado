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



/**
 *    \file src/code_generation/gauss_newton_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/gauss_newton_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

GaussNewtonExport::GaussNewtonExport(	UserInteraction* _userInteraction,
										const String& _commonHeaderName
										) : ExportAlgorithm( _userInteraction,_commonHeaderName )
{
	condenser = 0;
}


GaussNewtonExport::GaussNewtonExport( const GaussNewtonExport& arg ) : ExportAlgorithm( arg )
{
	copy( arg );
}


GaussNewtonExport::~GaussNewtonExport( )
{
	clear( );
}


GaussNewtonExport& GaussNewtonExport::operator=( const GaussNewtonExport& arg )
{
	if( this != &arg )
	{
		clear( );
		ExportAlgorithm::operator=( arg );
		copy( arg );
	}
    
	return *this;
}



returnValue GaussNewtonExport::setup( )
{
	String fileName( "gauss_newton_method.c" );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "--> Preparing to export %s... ",fileName.getName() );

	uint run1, run2;

	Matrix lbValuesMatrix( getN()*getNU(),1 );
	Matrix ubValuesMatrix( getN()*getNU(),1 );

	for( run1=0; run1<getN(); ++run1 )
		for( run2=0; run2<getNU(); ++run2 )
		{
			lbValuesMatrix( run1*getNU()+run2,0 ) = uBounds.getLowerBound(run1,run2);
			ubValuesMatrix( run1*getNU()+run2,0 ) = uBounds.getUpperBound(run1,run2);
		}

	x.setup( "x",    (getN()+1)*getNX(),1, REAL,ACADO_VARIABLES );
	u.setup( "u",    getN()*getNU(),1,     REAL,ACADO_VARIABLES );
	ExportVariable xEnd( "xEnd",   getNX(),1 );
	ExportVariable uEnd( "uEnd",   getNU(),1 );
	
	ExportVariable x0 ( "x0",           getNX(),1 );
	deltaX0.setup( "deltaX0", getNX(),1, REAL,ACADO_WORKSPACE );
	
	H.setup(   "H",   getNumQPvars(),getNumQPvars(),       REAL,ACADO_PARAMS );
	A.setup(   "A",   getNumStateBounds( ),getNumQPvars(), REAL,ACADO_PARAMS );
	H00.setup( "H00", getNX(),getNX(),               REAL,ACADO_WORKSPACE );
	H01.setup( "H01", getNX(),getN()*getNU(),        REAL,ACADO_WORKSPACE );
	H11.setup( "H11", getN()*getNU(),getN()*getNU(), REAL,ACADO_WORKSPACE );

	d.setup( "d", getN() * getNX(), 1,               REAL,ACADO_WORKSPACE );
	C.setup( "C", getN()*getNX(),getNX(),        REAL,ACADO_WORKSPACE );
	E.setup( "E", getN()*getNX(),getN()*getNU(), REAL,ACADO_WORKSPACE );
	
	g.setup(  "g",  getNumQPvars(),1, REAL,ACADO_PARAMS );
	g0.setup( "g0", getNX(),1,        REAL,ACADO_WORKSPACE );
	g1.setup( "g1", getN()*getNU(),1, REAL,ACADO_WORKSPACE );
	
	lb.setup( "lb", getNumQPvars(),1, REAL,ACADO_PARAMS );
	ub.setup( "ub", getNumQPvars(),1, REAL,ACADO_PARAMS );
	ExportVariable lbValues( "lb", lbValuesMatrix );
	ExportVariable ubValues( "ub", ubValuesMatrix );
	
	lbA.setup( "lbA", getNumStateBounds(),1, REAL,ACADO_PARAMS );
	ubA.setup( "ubA", getNumStateBounds(),1, REAL,ACADO_PARAMS );
	lbAValues.setup( "lbA", getNumStateBounds(),1, REAL,ACADO_WORKSPACE );
	ubAValues.setup( "ubA", getNumStateBounds(),1, REAL,ACADO_WORKSPACE );
	
	xVars.setup( "x", getNumQPvars(),1, REAL,ACADO_VARS );
	yVars.setup( "y", getNumQPvars()+getNumStateBounds(),1, REAL,ACADO_VARS );
	
	ExportVariable tmp( "tmp", 1,1, REAL,ACADO_LOCAL,BT_TRUE );

	////////////////////////////////////////////////////////////////////////////
	//
	// Preparation step
	//
	////////////////////////////////////////////////////////////////////////////
	preparationStep.setup( "preparationStep" );
	
//	if ( condenser->performsSingleShooting() == BT_FALSE )
//		preparationStep.addFunctionCall( "expand" );

	preparationStep.addFunctionCall( "setupQP" );


	////////////////////////////////////////////////////////////////////////////
	//
	// Initial value embedding
	//
	////////////////////////////////////////////////////////////////////////////
	initialValueEmbedding.setup( "initialValueEmbedding" );

	initialValueEmbedding.addStatement( "real_t tmp;\n" );
	
	if ( condenser->performsFullCondensing() == BT_TRUE )
	{
		initialValueEmbedding.addStatement( lb == lbValues - u );
		initialValueEmbedding.addStatement( ub == ubValues - u );
	}
	else
	{
		if ( isInitialStateFixed( ) == BT_TRUE )
		{
			initialValueEmbedding.addStatement( lb.getRows( 0,getNX() ) == deltaX0 );
			initialValueEmbedding.addStatement( ub.getRows( 0,getNX() ) == deltaX0 );
		}
		else
		{
			initialValueEmbedding.addStatement( lb.getRows( 0,getNX() ) == (Matrix)condenser->getLowerBoundsX0() );
			initialValueEmbedding.addStatement( lb.getRows( 0,getNX() ) -= x.getRows( 0,getNX() ) );
			initialValueEmbedding.addStatement( ub.getRows( 0,getNX() ) == (Matrix)condenser->getUpperBoundsX0() );
			initialValueEmbedding.addStatement( ub.getRows( 0,getNX() ) -= x.getRows( 0,getNX() ) );
		}

		initialValueEmbedding.addStatement( lb.getRows( getNX(),getNumQPvars() ) == lbValues - u );
		initialValueEmbedding.addStatement( ub.getRows( getNX(),getNumQPvars() ) == ubValues - u );
	}

	
	if ( condenser->performsFullCondensing() == BT_TRUE )
	{
		initialValueEmbedding.addStatement( g == g1 + (H01^deltaX0) );
	}
	else
	{
		initialValueEmbedding.addStatement( g.getRows( 0,getNX() ) == g0 );
		initialValueEmbedding.addStatement( g.getRows( getNX(),getNumQPvars() ) == g1 );
	}

	initialValueEmbedding.addLinebreak( );

	if( getNumStateBounds( ) > 0 )
	{
		for( run1 = 0; run1 < getNumStateBounds( ); run1++ )
		{
			if ( condenser->performsFullCondensing() == BT_TRUE )
			{
				// TODO: CVXGEN might require transposed storage!
				initialValueEmbedding.addStatement( A.getRow(run1) == E.getRow( condenser->getStateBoundComponent(run1)-getNX() ) );
			}
			else
			{
				initialValueEmbedding.addStatement( A.getSubMatrix( run1,run1+1, 0,getNX() ) == C.getRow( condenser->getStateBoundComponent(run1)-getNX() ) );
				initialValueEmbedding.addStatement( A.getSubMatrix( run1,run1+1, getNX(),getNumQPvars() ) == E.getRow( condenser->getStateBoundComponent(run1)-getNX() ) );
			}
		}
		
		// shift constraint bounds by first interval
		for( run1 = 0; run1 < getNumStateBounds( ); run1++ )
		{
			if ( condenser->performsFullCondensing() == BT_TRUE )
			{
				if ( condenser->performsSingleShooting() == BT_TRUE )
				{
					initialValueEmbedding.addStatement( tmp == x.getRow( condenser->getStateBoundComponent(run1) ) + C.getRow( condenser->getStateBoundComponent(run1)-getNX() )*deltaX0 );
				}
				else
				{
					initialValueEmbedding.addStatement( tmp == x.getRow( condenser->getStateBoundComponent(run1) ) + C.getRow( condenser->getStateBoundComponent(run1)-getNX() )*deltaX0 );
					initialValueEmbedding.addStatement( tmp -= d.getRow( condenser->getStateBoundComponent(run1) - getNX() ) );
				}
				initialValueEmbedding.addStatement( lbA.getRow(run1) == lbAValues.getRow(run1) - tmp );
				initialValueEmbedding.addStatement( ubA.getRow(run1) == ubAValues.getRow(run1) - tmp );
			}
			else
			{
				if ( condenser->performsSingleShooting() == BT_TRUE )
					initialValueEmbedding.addStatement( tmp == x.getRow( condenser->getStateBoundComponent(run1) ) );
				else
					initialValueEmbedding.addStatement( tmp == x.getRow( condenser->getStateBoundComponent(run1) ) - d.getRow( condenser->getStateBoundComponent(run1) - getNX() ) );
				initialValueEmbedding.addStatement( lbA.getRow(run1) == lbAValues.getRow(run1) - tmp );
				initialValueEmbedding.addStatement( ubA.getRow(run1) == ubAValues.getRow(run1) - tmp );
			}
		}
		
		initialValueEmbedding.addLinebreak( );
	}

	if ( condenser->performsFullCondensing() == BT_TRUE )
	{
		initialValueEmbedding.addStatement( H == H11 );
	}
	else
	{
		initialValueEmbedding.addStatement( H.getSubMatrix( 0,getNX(), 0,getNX() ) == H00 );
		initialValueEmbedding.addStatement( H.getSubMatrix( 0,getNX(), getNX(),getNumQPvars() ) == H01 );
		initialValueEmbedding.addStatement( H.getSubMatrix( getNX(),getNumQPvars(), 0,getNX() ) == H01.accessTransposed() );
		initialValueEmbedding.addStatement( H.getSubMatrix( getNX(),getNumQPvars(), getNX(),getNumQPvars() ) == H11 );
	}

	////////////////////////////////////////////////////////////////////////////
	//
	// Feedback step
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable returnValueFeedbackStep("retVal", 1, 1, INT, ACADO_LOCAL, BT_TRUE);

	feedbackStep.setup( "feedbackStep",x0 );

	feedbackStep.setReturnValue(returnValueFeedbackStep);

//	feedbackStep.addStatement( "int retVal;\n\n" );


	feedbackStep.addStatement( deltaX0 == x0 - x.getRows( 0,getNX() ) );
	feedbackStep.addLinebreak( );
	
	feedbackStep.addFunctionCall( initialValueEmbedding );
	feedbackStep.addLinebreak( );

//	feedbackStep.addFunctionCall( "solve" );
//	feedbackStep.addStatement( "retVal = (int) solve();\n" );
	feedbackStep.addLinebreak( );

	if ( condenser->performsFullCondensing() == BT_TRUE )
	{
		feedbackStep.addStatement( u += xVars );
		feedbackStep.addStatement( x.getRows( 0,getNX() ) += deltaX0 );
	}
	else
	{
		feedbackStep.addStatement( u += xVars.getRows( getNX(),getNumQPvars() ) );
		feedbackStep.addStatement( x.getRows( 0,getNX() ) += xVars.getRows( 0,getNX() ) );
	}

	// TODO: This is a quick hack... To be discussed where should be placed.
	if ( condenser->performsSingleShooting() == BT_FALSE )
		feedbackStep.addFunctionCall( "expand" );


	////////////////////////////////////////////////////////////////////////////
	//
	// Shift controls
	//
	////////////////////////////////////////////////////////////////////////////
	shiftControls.setup( "shiftControls",uEnd );
	
	for(run1 = 1; run1 < getN(); run1++)
		shiftControls.addStatement(u.getRows((run1 - 1) * getNU(), run1 * getNU()) ==
				u.getRows(run1 * getNU(), (run1 + 1) * getNU()));

	shiftControls.addLinebreak( );
	shiftControls.addStatement( "if (uEnd != NULL)\n{" );
	shiftControls.addStatement(u.getRows((run1 - 1) * getNU(), run1 * getNU()) == uEnd);
	shiftControls.addStatement( "}" );

	////////////////////////////////////////////////////////////////////////////
	//
	// Shift states
	// TODO: Check this once again! This seems to me like expanding routine. Anyway, this routine should be called after expanding routine...
	//
	////////////////////////////////////////////////////////////////////////////
	shiftStates.setup("shiftStates", xEnd);

	unsigned i;
	for ( i = 0; i < getN() ; ++i )
		shiftStates.addStatement(x.getRows(getNX() * i, getNX() * (i + 1)) ==
				x.getRows(getNX() * (i + 1), getNX() * (i + 2)));

	shiftStates.addLinebreak( );
	shiftStates.addStatement( "if (xEnd != NULL)\n{" );
	shiftStates.addStatement(x.getRows(getNX() * i, getNX() * (i + 1)) == xEnd );
	shiftStates.addStatement( "}" );

	////////////////////////////////////////////////////////////////////////////
	//
	// Get KKT value
	//
	////////////////////////////////////////////////////////////////////////////
	getKKT.setup( "getKKT" );
	getKKT.setReturnValue( tmp );
        
//	getKKT.addStatement( "real_t tmp;\n" );

	if ( condenser->performsFullCondensing() == BT_TRUE )
		getKKT.addStatement( tmp == (g1^xVars) );
	else
		getKKT.addStatement( tmp == (g1^xVars.getRows( getNX(),getNumQPvars() )) );

	getKKT.addStatement( "tmp = fabs( tmp );\n" );
	
// 	if ( performsSingleShooting() == BT_FALSE )
// 	{
// 		for( run1 = 0; run1 < getN(); ++run1 )
// 			getKKT.addStatement( "tmp += fabs( )" );
// 	}
	
// 	acadoFPrintf( file, "\n");
/*	acadoFPrintf( file, "    int run1;\n");
    acadoFPrintf( file, "    for( run1 = 0; run1 < %d; run1++ ){\n",getN()*getNU() );
	acadoFPrintf( file, "        if ( vars.y[run1] > %e )\n", 1.0/INFTY );
	acadoFPrintf( file, "            tmp += fabs( (acadoVariables.u[run1]-params.lb[run1])*vars.y[run1] );\n");
	acadoFPrintf( file, "        if ( vars.y[run1] < %e )\n", -1.0/INFTY );
	acadoFPrintf( file, "            tmp += fabs( (acadoVariables.u[run1]-params.ub[run1])*vars.y[run1] );\n");
	acadoFPrintf( file, "    }\n");
	acadoFPrintf( file, "\n");*/
	if ( getNumStateBounds() > 0 )
	{
    /*acadoFPrintf( file, "    for( run1 = 0; run1 < %d; run1++ ){\n",getN()*getNX() );
	acadoFPrintf( file, "        if ( vars.y[%d+run1] > %e )\n",getN()*getNU(), 1.0/INFTY );
	acadoFPrintf( file, "            tmp += fabs( (acadoVariables.x[run1]-acadoWorkspace.lbA[run1])*vars.y[%d+run1] );\n",getN()*getNU() );
	acadoFPrintf( file, "        if ( vars.y[%d+run1] < %e )\n",getN()*getNU(), -1.0/INFTY );
	acadoFPrintf( file, "            tmp += fabs( (acadoVariables.x[run1]-acadoWorkspace.ubA[run1])*vars.y[%d+run1] );\n",getN()*getNU() );
	acadoFPrintf( file, "    }\n");
	acadoFPrintf( file, "\n");
	*/}
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}



returnValue GaussNewtonExport::setCondensingExport( CondensingExport* const _condenser )
{
	condenser = _condenser;
	return SUCCESSFUL_RETURN;
}



returnValue GaussNewtonExport::setControlBounds(	const VariablesGrid& _uBounds
													)
{
	BooleanType isFinite = BT_FALSE;
	Vector lbTmp;
	Vector ubTmp;

	for( uint i=0; i<_uBounds.getNumPoints(); ++i )
	{
		lbTmp = _uBounds.getLowerBounds(i);
		ubTmp = _uBounds.getUpperBounds(i);

		if ( (ubTmp-lbTmp).isPositive() == BT_FALSE )
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		
		if ( ( lbTmp.isFinite( ) == BT_TRUE ) || ( ubTmp.isFinite( ) == BT_TRUE ) )
			isFinite = BT_TRUE;
	}

	if ( isFinite == BT_TRUE )
		uBounds = _uBounds;
	else
		uBounds.init();

	return SUCCESSFUL_RETURN;
}



returnValue GaussNewtonExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	// ACADO_PARAMS
	declarations.addDeclaration( H,dataStruct );
	declarations.addDeclaration( A,dataStruct );
	declarations.addDeclaration( g,dataStruct );
	declarations.addDeclaration( lb,dataStruct );
	declarations.addDeclaration( ub,dataStruct );
	declarations.addDeclaration( lbA,dataStruct );
	declarations.addDeclaration( ubA,dataStruct );

	// ACADO_VARS
	declarations.addDeclaration( xVars,dataStruct );
	declarations.addDeclaration( yVars,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue GaussNewtonExport::getFunctionDeclarations(	ExportStatementBlock& declarations
														) const
{
	declarations.addDeclaration( preparationStep );
	declarations.addDeclaration( initialValueEmbedding );
	declarations.addDeclaration( feedbackStep );
	declarations.addDeclaration( shiftControls );
	declarations.addDeclaration( shiftStates );
	declarations.addDeclaration( getKKT );

	return SUCCESSFUL_RETURN;
}


returnValue GaussNewtonExport::getCode(	ExportStatementBlock& code
										)
{
// 	int printLevel;
// 	get( PRINTLEVEL,printLevel );
// 
// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "--> Exporting %s... ",fileName.getName() );

	code.addFunction( preparationStep );
	code.addFunction( initialValueEmbedding );
	code.addFunction( feedbackStep );
	code.addFunction( shiftControls );
	code.addFunction( shiftStates );
	code.addFunction( getKKT );

// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}



BooleanType GaussNewtonExport::isInitialStateFixed( ) const
{
	return condenser->isInitialStateFixed( );
}



uint GaussNewtonExport::getNumQPvars( ) const
{
	return condenser->getNumQPvars();
}


uint GaussNewtonExport::getNumStateBounds( ) const
{
	return condenser->getNumStateBounds();
}



// PROTECTED:


returnValue GaussNewtonExport::copy(	const GaussNewtonExport& arg
										)
{
	condenser = arg.condenser;
	uBounds   = arg.uBounds;

	// ExportVariables
	x = arg.x;
	u = arg.u;

	deltaX0   = arg.deltaX0;
	H00       = arg.H00;
	H01       = arg.H01;
	H11       = arg.H11;
	d         = arg.d;
	C         = arg.C;
	E         = arg.E;
	g0        = arg.g0;
	g1        = arg.g1;
	lbAValues = arg.lbAValues;
	ubAValues = arg.ubAValues;

	H   = arg.H;
	A   = arg.A;
	g   = arg.g;
	lb  = arg.lb;
	ub  = arg.ub;
	lbA = arg.lbA;
	ubA = arg.ubA;

	xVars = arg.xVars;
	yVars = arg.yVars;
	
	// ExportFunctions
	preparationStep       = arg.preparationStep;
	feedbackStep          = arg.feedbackStep;
	initialValueEmbedding = arg.initialValueEmbedding;
	shiftControls         = arg.shiftControls;
	shiftStates           = arg.shiftStates;
	getKKT                = arg.getKKT;

	return SUCCESSFUL_RETURN;
}


returnValue GaussNewtonExport::clear( )
{
	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
