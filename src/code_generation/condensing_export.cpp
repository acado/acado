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
 *    \file src/code_generation/condensing_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska
 *    \date 2010-2011
 */

#include <acado/code_generation/condensing_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

CondensingExport::CondensingExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExportAlgorithm( _userInteraction,_commonHeaderName )
{
	integrator = 0;
	
	levenbergMarquardt    = 0.0;

	xBoundsIdx = 0;
	nxBounds = 0;
}


CondensingExport::CondensingExport(	const CondensingExport& arg
									) : ExportAlgorithm( arg )
{
	copy( arg );
}


CondensingExport::~CondensingExport( )
{
	clear( );
}


CondensingExport& CondensingExport::operator=(	const CondensingExport& arg
												)
{
	if( this != &arg )
	{
		clear( );
		ExportAlgorithm::operator=( arg );
		copy( arg );
	}

	return *this;
}



returnValue CondensingExport::setup( )
{
	String fileName( "condensing.c" );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "--> Preparing to export %s... ",fileName.getName() );

	Q.setDataStruct( ACADO_VARIABLES );
	R.setDataStruct( ACADO_VARIABLES );
	QF.setDataStruct( ACADO_VARIABLES );
	QS.setDataStruct( ACADO_VARIABLES );
	QS2.setDataStruct( ACADO_VARIABLES );

	setupQQF( );

	// set up in this order to ensure correct initialization!
	setupMultiplicationRoutines( );
	setupCondensing( );
	setupEvaluation( );
	
	
	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}



returnValue CondensingExport::setIntegratorExport(	IntegratorExport* const _integrator
													)
{
	integrator = _integrator;
	return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::setWeightingMatrices(	const ExportVariable& _Q,
													const ExportVariable& _R,
													const ExportVariable& _QF,
													const ExportVariable& _QS,
													const ExportVariable& _QS2
													)
{
	QS  = _QS;
	QS2 = _QS2;
    Q   = _Q;
    QF  = _QF;

    R = _R;

    return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::setStateBounds(	const VariablesGrid& _xBounds
												)
{
	BooleanType isFinite = BT_FALSE;
	Vector lbTmp = _xBounds.getLowerBounds(0);
	Vector ubTmp = _xBounds.getUpperBounds(0);
	
	if ( xBoundsIdx != 0 )
		delete[] xBoundsIdx;
	xBoundsIdx = new int[_xBounds.getDim()+1];

	for( uint j=0; j<lbTmp.getDim(); ++j )
	{
		if ( acadoIsGreater( ubTmp(j),lbTmp(j) ) == BT_FALSE )
			return ACADOERROR( RET_INVALID_ARGUMENTS );
			
		if ( ( acadoIsFinite( ubTmp(j) ) == BT_TRUE ) || ( acadoIsFinite( lbTmp(j) ) == BT_TRUE ) )
			isFinite = BT_TRUE;
	}

	for( uint i=1; i<_xBounds.getNumPoints(); ++i )
	{
		lbTmp = _xBounds.getLowerBounds(i);
		ubTmp = _xBounds.getUpperBounds(i);

		for( uint j=0; j<lbTmp.getDim(); ++j )
		{
			if ( acadoIsGreater( ubTmp(j),lbTmp(j) ) == BT_FALSE )
				return ACADOERROR( RET_INVALID_ARGUMENTS );
			
			if ( ( acadoIsFinite( ubTmp(j) ) == BT_TRUE ) || ( acadoIsFinite( lbTmp(j) ) == BT_TRUE ) )
			{
				xBoundsIdx[nxBounds] = i*lbTmp.getDim()+j;
				++nxBounds;
				isFinite = BT_TRUE;
			}
		}
	}

	xBoundsIdx[nxBounds] = -1;

	if ( isFinite == BT_TRUE )
		xBounds = _xBounds;
	else
		xBounds.init();

	return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::setLevenbergMarquardt(	double _levenbergMarquardt
														)
{
	if ( _levenbergMarquardt < 0.0 )
	{
		ACADOWARNING( RET_INVALID_ARGUMENTS );
		levenbergMarquardt = 0.0;
	}
	else
	{
		levenbergMarquardt = _levenbergMarquardt;
	}

	return SUCCESSFUL_RETURN;
}



uint CondensingExport::getNumQPvars( ) const
{
	if ( performsFullCondensing() == BT_TRUE )
		return getNU()*getN();
	else
		return getNX() + getNU()*getN();
}


uint CondensingExport::getNumStateBounds( ) const
{
	return nxBounds;
}


int CondensingExport::getStateBoundComponent(	uint idx
												) const
{
	if( idx >= getNumStateBounds() )
		return -ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return xBoundsIdx[idx];
}


Vector CondensingExport::getLowerBoundsX0( ) const
{
	if ( xBounds.isEmpty() == BT_FALSE )
		return xBounds.getLowerBounds(0);
	else
	{
		Vector tmp( getNX() );
		tmp.setAll( -INFTY );
		return tmp;
	}
}


Vector CondensingExport::getUpperBoundsX0( ) const
{
	if ( xBounds.isEmpty() == BT_FALSE )
		return xBounds.getUpperBounds(0);
	else
	{
		Vector tmp( getNX() );
		tmp.setAll( INFTY );
		return tmp;
	}
}



BooleanType CondensingExport::performsFullCondensing( ) const
{
	int sparseQPsolution;
	get( SPARSE_QP_SOLUTION,sparseQPsolution );

	if ( (SparseQPsolutionMethods)sparseQPsolution == CONDENSING )
		return BT_FALSE;
	else
		return BT_TRUE;
}


BooleanType CondensingExport::performsSingleShooting( ) const
{
	int discretizationType;
	get( DISCRETIZATION_TYPE,discretizationType );

	if ( discretizationType == SINGLE_SHOOTING )
		return BT_TRUE;
	else
		return BT_FALSE;
}


BooleanType CondensingExport::isInitialStateFixed( ) const
{
	int fixInitialState;
	get( FIX_INITIAL_STATE,fixInitialState );

	return (BooleanType)fixInitialState;
}



returnValue CondensingExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	// ACADO_VARIABLES
	declarations.addDeclaration( x,dataStruct );
	declarations.addDeclaration( u,dataStruct );
	declarations.addDeclaration( p,dataStruct );
	declarations.addDeclaration( xRef,dataStruct );
	declarations.addDeclaration( uRef,dataStruct );
	declarations.addDeclaration( x0Ref,dataStruct );
	declarations.addDeclaration( x0Ref2,dataStruct );

	if ( Q.isGiven() == BT_FALSE )
		declarations.addDeclaration( Q,dataStruct );
	
	if ( R.isGiven() == BT_FALSE )
		declarations.addDeclaration( R,dataStruct );
	
	if ( QF.isGiven() == BT_FALSE )
		declarations.addDeclaration( QF,dataStruct );
	
	if ( QS.isGiven() == BT_FALSE )
		declarations.addDeclaration( QS,dataStruct );

	if ( QS2.isGiven() == BT_FALSE )
		declarations.addDeclaration( QS2,dataStruct );


	// ACADO_WORKSPACE
	if ( QQF.isGiven() == BT_FALSE )
		declarations.addDeclaration( QQF,dataStruct );

	declarations.addDeclaration( state,dataStruct );
	declarations.addDeclaration( residuum,dataStruct );
	declarations.addDeclaration( g0,dataStruct );
	declarations.addDeclaration( g1,dataStruct );
	declarations.addDeclaration( H00,dataStruct );
	declarations.addDeclaration( H01,dataStruct );
	declarations.addDeclaration( H11,dataStruct );
	declarations.addDeclaration( lbA,dataStruct );
	declarations.addDeclaration( ubA,dataStruct );
	declarations.addDeclaration( d,dataStruct );
	declarations.addDeclaration( deltaX0,dataStruct );
	declarations.addDeclaration( C,dataStruct );
	declarations.addDeclaration( QC,dataStruct );
	declarations.addDeclaration( Gx,dataStruct );
	declarations.addDeclaration( E,dataStruct );
	declarations.addDeclaration( QE,dataStruct );
	declarations.addDeclaration( Gu,dataStruct );
	declarations.addDeclaration( Dx0,dataStruct );
	declarations.addDeclaration( Dx0b,dataStruct );
	declarations.addDeclaration( Dx,dataStruct );
	declarations.addDeclaration( QDx,dataStruct );
	declarations.addDeclaration( Du,dataStruct );
	declarations.addDeclaration( RDu,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::getFunctionDeclarations(	ExportStatementBlock& declarations
														) const
{
	declarations.addDeclaration( condense1 );
	declarations.addDeclaration( condense2 );
	declarations.addDeclaration( expand );
	declarations.addDeclaration( setupQP );
	
	declarations.addDeclaration( multiplyQC1 );
	declarations.addDeclaration( multiplyQE1 );
	declarations.addDeclaration( multiplyQDX1 );
	declarations.addDeclaration( multiplyRDU1 );
	declarations.addDeclaration( multiplyQC2 );
	declarations.addDeclaration( multiplyQE2 );
	declarations.addDeclaration( multiplyQDX2 );

	declarations.addDeclaration( multiplyC );
	declarations.addDeclaration( multiplyE );
	declarations.addDeclaration( multiplyG1 );
	declarations.addDeclaration( multiplyCD1 );
	declarations.addDeclaration( multiplyEU1 );
	declarations.addDeclaration( multiplyG0 );
	declarations.addDeclaration( multiplyH00 );
	declarations.addDeclaration( multiplyH01 );
	declarations.addDeclaration( multiplyH11 );

	declarations.addDeclaration( getObjectiveValue );

	return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::getCode(	ExportStatementBlock& code
										)
{
// 	int printLevel;
// 	get( PRINTLEVEL,printLevel );
// 
// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "--> Exporting %s... ",fileName.getName() );

	code.addFunction( multiplyQC1 );
	code.addFunction( multiplyQE1 );
	code.addFunction( multiplyQDX1 );
	code.addFunction( multiplyRDU1 );
	code.addFunction( multiplyQC2 );
	code.addFunction( multiplyQE2 );
	code.addFunction( multiplyQDX2 );
	code.addFunction( multiplyC );
	code.addFunction( multiplyE );
	code.addFunction( multiplyCD1 );
	code.addFunction( multiplyEU1 );
	code.addFunction( multiplyG0 );
	code.addFunction( multiplyG1 );
	code.addFunction( multiplyH00 );
	code.addFunction( multiplyH01 );
	code.addFunction( multiplyH11 );

	code.addFunction( condense1 );
	code.addFunction( condense2 );
	code.addFunction( expand );
	code.addFunction( setupQP );

	code.addFunction( getObjectiveValue );

// 	if ( (PrintLevel)printLevel >= HIGH ) 
// 		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}



//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue CondensingExport::copy(	const CondensingExport& arg
									)
{
	integrator = arg.integrator;

	levenbergMarquardt = arg.levenbergMarquardt;

	xBounds = arg.xBounds;
	
	if ( arg.xBoundsIdx != 0 )
	{
		xBoundsIdx = new int[arg.nxBounds];
		for( uint i=0; i<arg.nxBounds; ++i )
			xBoundsIdx[i] = arg.xBoundsIdx[i];
	}
	else
		xBoundsIdx = 0;

	nxBounds  = arg.nxBounds;

	// ExportVariables
	Q   = arg.Q;
	R   = arg.R;
	QF  = arg.QF;
	QQF = arg.QQF;
	QS  = arg.QS;
	QS2 = arg.QS2;

	x = arg.x;
	u = arg.u;
	p = arg.p;
	xRef   = arg.xRef;
	uRef   = arg.uRef;
	x0Ref  = arg.x0Ref;
	x0Ref2 = arg.x0Ref2;

	state    = arg.state;
	residuum = arg.residuum;
	g0       = arg.g0;
	g1       = arg.g1;
	H00      = arg.H00;
	H01      = arg.H01;
	H11      = arg.H11;
	lbA      = arg.lbA;
	ubA      = arg.ubA;
	d        = arg.d;
	deltaX0  = arg.deltaX0;
	C        = arg.C;
	QC       = arg.QC;
	Gx       = arg.Gx;
	Gu       = arg.Gu;
	E        = arg.E;
	QE       = arg.QE;
	Dx0      = arg.Dx0;
	Dx0b     = arg.Dx0b;
	Dx       = arg.Dx;
	QDx      = arg.QDx;
	Du       = arg.Du;
	RDu      = arg.RDu;

	deltaU = arg.deltaU;

	// ExportFunctions
	condense1 = arg.condense1;
	condense2 = arg.condense2;
	expand    = arg.expand;
	setupQP   = arg.setupQP;

	multiplyQC1  = arg.multiplyQC1;
	multiplyQE1  = arg.multiplyQE1;
	multiplyQDX1 = arg.multiplyQDX1;
	multiplyRDU1 = arg.multiplyRDU1;
	multiplyQC2  = arg.multiplyQC2;
	multiplyQE2  = arg.multiplyQE2;
	multiplyQDX2 = arg.multiplyQDX2;

	multiplyC    = arg.multiplyC;
	multiplyE    = arg.multiplyE;
	multiplyCD1  = arg.multiplyCD1;
	multiplyEU1  = arg.multiplyEU1;
	multiplyG0   = arg.multiplyG0;
	multiplyG1   = arg.multiplyG1;
	multiplyH00  = arg.multiplyH00;
	multiplyH01  = arg.multiplyH01;
	multiplyH11  = arg.multiplyH11;

	getObjectiveValue  = arg.getObjectiveValue;

	return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::clear( )
{
	if ( xBoundsIdx != 0 )
		delete[] xBoundsIdx;

	return SUCCESSFUL_RETURN;
}



returnValue CondensingExport::setupMultiplicationRoutines( )
{
	ExportVariable QQ = deepcopy( Q );
	QQ.setDataStruct( ACADO_LOCAL );

	ExportVariable QC1( "QC1",  getNX(),getNX() );
	ExportVariable QE1( "QE1",  getNX(),getNU()*getN() );
	ExportVariable Dx1( "Dx1",  getNX(),1 );
	ExportVariable QDx1("QDx1", getNX(),1 );
	ExportVariable Du1( "Du1",  getNU(),1 );
	ExportVariable RDu1("RDu1", getNU(),1 );
	ExportVariable Gx ( "Gx",   getNX(),getNX() );
	ExportVariable Gu ( "Gu",   getNX(),getNU() );
	ExportVariable C1 ( "C1",   getNX(),getNX() );
	ExportVariable E1 ( "E1",   getNX(),getNU()*getN() );
	ExportVariable d1 ( "d1",   getNX(),1 );
	ExportVariable u1 ( "u1",   getNU(),1 );
	ExportVariable C  ( "C",    getNX()*getN(),getNX() );
	ExportVariable QC ( "QC",   getNX()*getN(),getNX() );
	ExportVariable E  ( "E",    getNX()*getN(),getNU()*getN() );
	ExportVariable QE ( "QE",   getNX()*getN(),getNU()*getN() );
	ExportVariable QDx( "QDx",  getNX()*getN(),1 );
	ExportVariable g0 ( "g0",   getNX(),1 );
	ExportVariable g1 ( "g1",   getNU()*getN(),1 );
	ExportVariable H00( "H00",  getNX(),getNX() );
	ExportVariable H01( "H01",  getNX(),getNU()*getN() );
	ExportVariable H11( "H11",  getNU()*getN(),getNU()*getN() );

	multiplyQC1.setup( "multiplyQC1", QQ,C1,QC1 );
	multiplyQC1.addStatement( QC1 == QQ*C1 );

	multiplyQE1.setup( "multiplyQE1", QQ,E1,QE1 );
	multiplyQE1.addStatement( QE1 == QQ*E1 );

	multiplyQDX1.setup( "multiplyQDX1", QQ,Dx1,QDx1 );
	multiplyQDX1.addStatement( QDx1 == QQ*Dx1 );

	multiplyRDU1.setup( "multiplyRDU1", R,Du1,RDu1 );
	multiplyRDU1.addStatement( RDu1 == R*Du1 );

	multiplyQC2.setup( "multiplyQC2", QQF,C1,QC1 );
	multiplyQC2.addStatement( QC1 == QQF*C1 );

	multiplyQE2.setup( "multiplyQE2", QQF,E1,QE1 );
	multiplyQE2.addStatement( QE1 == QQF*E1 );

	multiplyQDX2.setup( "multiplyQDX2", QQF,Dx1,QDx1 );
	multiplyQDX2.addStatement( QDx1 == QQF*Dx1 );

	multiplyC.setup( "multiplyC", Gx,C1,C1("C1_new") );
	multiplyC.addStatement( C1("C1_new") == Gx*C1 );

	multiplyE.setup( "multiplyE", Gx,E1,E1("E1_new") );
	multiplyE.addStatement( E1("E1_new") == Gx*E1 );
	
	if ( performsSingleShooting() == BT_FALSE )
	{
		multiplyCD1.setup( "multiplyCD1", Gx,d1,d1("d1_new") );
		multiplyCD1.addStatement( d1("d1_new") == Gx*d1 );

		multiplyEU1.setup( "multiplyEU1", Gu,u1,d1("d1_new") );
		multiplyEU1.addStatement( d1("d1_new") += Gu*u1 );
	}

	if ( performsFullCondensing() == BT_FALSE )
	{
		multiplyG0.setup( "multiplyG0", C,QDx,g0 );
		multiplyG0.addStatement( g0 == (C^QDx) );
	}
	
	multiplyG1.setup( "multiplyG1", E,QDx,g1 );
	multiplyG1.addStatement( g1 == (E^QDx) );

	if ( performsFullCondensing() == BT_FALSE )
	{
		multiplyH00.setup( "multiplyH00", C,QC,H00 );
		multiplyH00.addStatement( H00 == (C^QC) );
	}

	multiplyH01.setup( "multiplyH01", C,QE,H01 );
	multiplyH01.addStatement( H01 == (C^QE) );

	multiplyH11.setup( "multiplyH11", E,QE,H11 );
	multiplyH11.addStatement( (H11 == (E^QE)) );

	return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::setupCondensing( )
{
    uint run1;

	x.setup(    "x",     getN()+1, getNX(), REAL,ACADO_VARIABLES );
	xRef.setup( "xRef",  getN(), getNX(),   REAL,ACADO_VARIABLES );
	uRef.setup( "uRef",  getN(), getNU(),   REAL,ACADO_VARIABLES );

	Gx.setup( "Gx",  getNX(), getNX(), REAL,ACADO_WORKSPACE );
	Gu.setup( "Gu",  getNX(), getNU(), REAL,ACADO_WORKSPACE );
	
	ExportVariable yy( "yy", 1, getNX()*(getNX()+getNU()+1)+getNU() );

	ExportVariable Q0 = QS;
	ExportVariable Q0b = QS2;

	if ( performsSingleShooting() == BT_FALSE )
	{
		residuum.setup( "residuum", getN(), getNX(), REAL,ACADO_WORKSPACE );
		d.setup(        "d",        getN(), getNX(), REAL,ACADO_WORKSPACE );
	}

	deltaX0.setup( "deltaX0", getNX(),       1, REAL,ACADO_WORKSPACE );

	C.setup   ( "C",    getN()*getNX(), getNX(), REAL,ACADO_WORKSPACE );
	QC.setup  ( "QC",   getN()*getNX(), getNX(), REAL,ACADO_WORKSPACE );
	E.setup   ( "E",    getN()*getNX(), getN()*getNU(), REAL,ACADO_WORKSPACE );
	QE.setup  ( "QE",   getN()*getNX(), getN()*getNU(), REAL,ACADO_WORKSPACE );

	if ( isInitialStateFixed( ) == BT_FALSE )
	{
		Dx0.setup ( "Dx0",  1,              getNX(), REAL,ACADO_WORKSPACE );
		Dx0b.setup( "Dx0b", 1,              getNX(), REAL,ACADO_WORKSPACE );
	}
	
	Dx.setup  ( "Dx",   getN(),         getNX(), REAL,ACADO_WORKSPACE );
	QDx.setup ( "QDx",  getN()*getNX(), 1, REAL,ACADO_WORKSPACE );
	Du.setup  ( "Du",   getN(),         getNU(), REAL,ACADO_WORKSPACE );
	RDu.setup ( "RDu",  getN(),         getNU(), REAL,ACADO_WORKSPACE );

	if ( performsFullCondensing() == BT_FALSE )
	{
		g0.setup  ( "g0",   getNX()       , 1, REAL,ACADO_WORKSPACE );
		H00.setup ( "H00",  getNX()       , getNX(), REAL,ACADO_WORKSPACE );
	}

	g1.setup  ( "g1",   getN()*getNU(), 1, REAL,ACADO_WORKSPACE );
	H01.setup ( "H01",  getNX(),        getN()*getNU(), REAL,ACADO_WORKSPACE );
	H11.setup ( "H11",  getN()*getNU(), getN()*getNU(), REAL,ACADO_WORKSPACE );

	deltaU.setup( "vars.x", getNumQPvars(),1 );

	ExportIndex index( String("index") );

	////////////////////////////////////////////////////////////////////////////
	//
	// First condensing routine
	//
	////////////////////////////////////////////////////////////////////////////
	condense1.setup( "condense1", index.makeArgument(),yy );
	
	// TO BE TESTED
	if ( performsSingleShooting() == BT_TRUE )
		condense1.addStatement( Dx.getRow(index) == yy.getCols( 0,getNX() ) - xRef.getRow(index) );
	else
		condense1.addStatement( Dx.getRow(index) == x.getRow( index + 1 ) - xRef.getRow(index) );
	
	uint uIdx = getNX()*(getNX()+getNU()+1);
	condense1.addStatement( Du.getRow(index) == yy.getCols( uIdx,uIdx+getNU() ) - uRef.getRow(index) );
	condense1.addStatement( Gx.makeRowVector() == yy.getCols( getNX(),getNX()+getNX()*getNX() ) );

	uIdx = getNX()*(getNX()+1);
	condense1.addStatement( Gu.makeRowVector() == yy.getCols( uIdx,uIdx+getNX()*getNU() ) );

    condense1.addStatement( "if( index != 0 ){\n" );

	if ( performsSingleShooting() == BT_FALSE )
	{
		condense1.addFunctionCall( multiplyCD1, Gx, d.getAddress(index - 1), d.getAddress(index) );
		// TODO: check this once again
//		condense1.addStatement( d.getRow( index ) += residuum.getRow( index-1 ) );
		condense1.addStatement( d.getRow( index ) += residuum.getRow( index ) );
	}
	condense1.addFunctionCall( multiplyC, Gx,C.getAddress((index-1)*getNX(),0), C.getAddress(index*getNX(),0) );
	condense1.addFunctionCall( multiplyE, Gx,E.getAddress((index-1)*getNX(),0), E.getAddress(index*getNX(),0) );
    
	condense1.addStatement( "}\nelse{\n" );
	
	if ( performsSingleShooting() == BT_FALSE )
	{
		condense1.addStatement( d.getRow( 0 ) == residuum.getRow( 0 ) );
	}
	condense1.addStatement( C.getRows( 0,getNX() ) == Gx );

	condense1.addStatement( "}\n" );

	condense1.addStatement( E.getSubMatrix( index*getNX(),(index+1)*getNX(), index*getNU(),(index+1)*getNU() ) == Gu );

	////////////////////////////////////////////////////////////////////////////
	//
	// Second condensing routine
	//
	////////////////////////////////////////////////////////////////////////////
	condense2.setup( "condense2" );

	//
	// Create matrices QC and QE, vectors QDx, RDu
	//
    for( run1 = 0; run1 < getN()-1; run1++ )
	{
		condense2.addFunctionCall( multiplyQC1, Q, C.getAddress(run1*getNX(),0),  QC.getAddress(run1*getNX(),0)  );
		condense2.addFunctionCall( multiplyQE1, Q, E.getAddress(run1*getNX(),0),  QE.getAddress(run1*getNX(),0)  );
		condense2.addFunctionCall( multiplyQDX1, Q, Dx.getAddress(run1), QDx.getAddress(run1*getNX()) );
		condense2.addFunctionCall( multiplyRDU1, R, Du.getAddress(run1), RDu.getAddress(run1) );
    }

	condense2.addFunctionCall( multiplyQC2, QQF, C.getAddress((getN()-1)*getNX(),0),  QC.getAddress((getN()-1)*getNX(),0)  );
	condense2.addFunctionCall( multiplyQE2, QQF, E.getAddress((getN()-1)*getNX(),0),  QE.getAddress((getN()-1)*getNX(),0)  );
	condense2.addFunctionCall( multiplyQDX2, QQF, Dx.getAddress(getN()-1), QDx.getAddress((getN()-1)*getNX()) );
	condense2.addFunctionCall( multiplyRDU1, R,  Du.getAddress(getN()-1), RDu.getAddress(getN()-1) );

	//
	// Condense gradient
	//
	if ( performsFullCondensing() == BT_FALSE )
	{
		condense2.addFunctionCall( multiplyG0, C, QDx, g0 );
		
		if ( isInitialStateFixed( ) == BT_FALSE )
			condense2.addStatement( g0 += Q0 * Dx0.makeColVector() );
		if ( isInitialStateFixed( ) == BT_FALSE )
			condense2.addStatement( g0 += Q0b * Dx0b.makeColVector() );
	}
	
	if ( performsSingleShooting() == BT_TRUE )
	{
		condense2.addFunctionCall( multiplyG1,  E, QDx, g1 );
	}
	else
	{
		condense2.addStatement( Dx == Dx + d );
		condense2.addFunctionCall(multiplyG1, QE, Dx, g1 );
	}

	condense2.addStatement( g1 += RDu.makeColVector() );

	//
	// Condense Hessian
	//
	if ( performsFullCondensing() == BT_FALSE )
	{
		condense2.addFunctionCall( multiplyH00, C,QC,H00 );

		Matrix regH = eye( getNX() );
		regH *= levenbergMarquardt;

		if ( isInitialStateFixed( ) == BT_FALSE )
		{
			condense2.addStatement( H00 += Q0 + regH );
			condense2.addStatement( H00 += Q0b );
		}
		else
			condense2.addStatement( H00 += regH );
	}
	
	condense2.addFunctionCall( multiplyH01, C, QE, H01 );
	condense2.addFunctionCall( multiplyH11, E, QE, H11 );

	Matrix regH = eye( getNU() );
	regH *= levenbergMarquardt;

	for( run1 = 0; run1 < getN(); run1++ )
		condense2.addStatement( H11.getSubMatrix( run1*getNU(),(run1+1)*getNU(), run1*getNU(),(run1+1)*getNU() ) += R + regH );


	////////////////////////////////////////////////////////////////////////////
	//
	// Expanding routine
	// TODO: create multiplication routines maybe...
	//
	////////////////////////////////////////////////////////////////////////////
	if ( performsSingleShooting() == BT_FALSE )
	{
		expand.setup( "expand" );

		if ( performsFullCondensing() == BT_TRUE )
		{
			// d += C * deltaX0
			expand.addStatement( d.makeColVector() += C * deltaX0 );
			// d += E * deltaU
			expand.addStatement( d.makeColVector() += E * deltaU );
		}
		else
		{
			// d += C * deltaX0
			expand.addStatement( d.makeColVector() += C * deltaU.getRows(0, getNX()) );
			// d += E * deltaU
			expand.addStatement( d.makeColVector() += E * deltaU.getRows(getNX(), getNumQPvars()) );
		}

		// x(:, 1: N + 1) += d
		expand.addStatement( x.getRows(1, getN() + 1) += d );
	}

	return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::setupEvaluation( )
{
	x.setup( "x",        (getN()+1), getNX(), REAL,ACADO_VARIABLES );
	u.setup( "u",        getN(), getNU(),     REAL,ACADO_VARIABLES );
	p.setup( "p",        1, getNP(),          REAL,ACADO_VARIABLES );

	state.setup   ( "state",    1,getNX()*(getNX()+getNU()+1) + getNU() +getNP(), REAL,ACADO_WORKSPACE );

	if ( isInitialStateFixed( ) == BT_FALSE )
	{
		x0Ref.setup ( "x0Ref",  1, getNX(), REAL,ACADO_VARIABLES );
		x0Ref2.setup( "x0Ref2", 1, getNX(), REAL,ACADO_VARIABLES );
	}

	E.setup  ( "E",   getN()*getNX(), getN()*getNU(), REAL,ACADO_WORKSPACE );
	lbA.setup( "lbA", getNumStateBounds(), 1, REAL,ACADO_WORKSPACE );
	ubA.setup( "ubA", getNumStateBounds(), 1, REAL,ACADO_WORKSPACE );

	Matrix zeroXU = zeros( getNX(),getNU() );
	Matrix idX    = eye( getNX() );

	//
	// setupQP
	//
	setupQP.setup( "setupQP" );
    
	if ( performsSingleShooting() == BT_TRUE )
		setupQP.addStatement( state.getCols( 0,getNX() ) == x.getRow(0) );

	// TODO: this part should be preinitialized. More specifically, E & QE matrices should be preinitializes to 0.
	uint run1, run2;
	for( run1 = 0; run1 < getN()-1; run1++ )
		for( run2 = 1+run1; run2 < getN(); run2++ )
			setupQP.addStatement( E.getSubMatrix( run1*getNX(),(run1+1)*getNX(), run2*getNU(),(run2+1)*getNU() ) == zeroXU );


    // Write state bounds to the file
	// TODO: Since the bounds are fixed in this case, they should be preinitialized
	if( getNumStateBounds( ) > 0 )
	{
		Vector xLowerBounds(nxBounds), xUpperBounds(nxBounds);
		for( run1 = 0; run1 < nxBounds; run1++ )
		{
			xLowerBounds(run1) = xBounds.getLowerBound( xBoundsIdx[run1]/getNX(),xBoundsIdx[run1]%getNX() );
			xUpperBounds(run1) = xBounds.getUpperBound( xBoundsIdx[run1]/getNX(),xBoundsIdx[run1]%getNX() );
		}
		
		setupQP.addStatement( lbA == xLowerBounds );
		setupQP.addStatement( ubA == xUpperBounds );
	}
	setupQP.addLinebreak( );


	if ( isInitialStateFixed( ) == BT_FALSE )
	{
		setupQP.addStatement( Dx0  == x.getRow(0) - x0Ref );
		setupQP.addStatement( Dx0b == x.getRow(0) - x0Ref2 );
		setupQP.addLinebreak( );
	}
	
	
	// compute QQF if necessary
	if ( QQF.isGiven( ) == BT_FALSE )
		setupQP.addStatement( QQF == Q + QF );

	setupQP.addStatement( "acadoWorkspace.rk_num = 0;\n" );

	ExportIndex run( String("run1") );
	ExportForLoop loop( run, 0,getN() );
	
	setupQP.addIndex( run );

	if ( performsSingleShooting() == BT_FALSE ) {
		loop.addStatement( "acadoWorkspace.rk_num = 0;\n" );
		loop.addStatement( state.getCols( 0,getNX() ) == x.getRow( run ) );
	}
	
	// no free parameters implemented yet!
	uint uIdx = getNX() * ( 1+getNX() );
	uint pIdx = getNX() * ( 1+getNX()+getNU() );
	uIdx = pIdx;
	pIdx = pIdx + getNU();
	loop.addStatement( state.getCols( uIdx,pIdx ) == u.getRow( run ) );
	loop.addStatement( state.getCols( pIdx,pIdx+getNP() ) == p );
	loop.addLinebreak( );
	
	if ( integrator->hasEquidistantGrid() )
	{
		loop.addFunctionCall( "integrate", state );
	}
	else
	{
		loop.addFunctionCall( "integrate", run.makeArgument(), state );
	}
	
	if ( performsSingleShooting() == BT_TRUE )
	{
		loop.addStatement( x.getRow( run+1 ) == state.getCols( 0,getNX() ) );
	}
	else
	{
		//
		// Multiple shooting
		// TODO: Check the sign here
		//
		loop.addStatement( residuum.getRow( run ) == state.getCols( 0,getNX() ) - x.getRow( run+1 ) );
	}
	
	loop.addLinebreak( );
	
	loop.addFunctionCall( condense1, run.makeArgument(),state );

    setupQP.addStatement( loop );
	setupQP.addLinebreak( );
		
	setupQP.addFunctionCall( condense2 );

	////////////////////////////////////////////////////////////////////////////
	//
	// Get objective value
	//
	////////////////////////////////////////////////////////////////////////////

	ExportVariable tmp("tmp", 1, 1, REAL, ACADO_LOCAL, BT_TRUE);
	ExportVariable tmpDx("tmpDx", 1, getNX(), REAL, ACADO_LOCAL );
	ExportVariable tmpDu("tmpDu", 1, getNU(), REAL, ACADO_LOCAL );

	getObjectiveValue.setup( "getObjectiveValue" );
	getObjectiveValue.setReturnValue( tmp );

	getObjectiveValue.addVariable( tmpDx );
	getObjectiveValue.addVariable( tmpDu );

	getObjectiveValue.addStatement( tmp == 0.0 );
	getObjectiveValue.addLinebreak( );

	for (unsigned i = 0; i < getN(); ++i)
	{
		getObjectiveValue.addStatement( tmpDx == Dx.getRow( i ) * Q );
		getObjectiveValue.addStatement( tmp += Dx.getRow( i ) * tmpDx.getTranspose() );
	}
	getObjectiveValue.addLinebreak( );

	for (unsigned i = 0; i < getN(); ++i)
	{
		getObjectiveValue.addStatement( tmpDu == Du.getRow( i ) * R );
		getObjectiveValue.addStatement( tmp += Du.getRow( i ) * tmpDu.getTranspose() );
	}
	getObjectiveValue.addLinebreak( );

	getObjectiveValue.addStatement( tmp == tmp * Matrix( 0.5 ) );

	return SUCCESSFUL_RETURN;
}


returnValue CondensingExport::setupQQF( )
{
	if ( ( Q.isGiven() == BT_TRUE ) && ( QF.isGiven() == BT_TRUE ) )
	{
		QQF = (Q.getGivenMatrix() + QF.getGivenMatrix());
	}
	else
	{
		QQF.setup( "QQF",Q.getNumRows(),Q.getNumCols() );
		QQF.setDataStruct( ACADO_WORKSPACE );
	}

	return SUCCESSFUL_RETURN;
}



CLOSE_NAMESPACE_ACADO

// end of file.
