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
 *    \file src/code_generation/irk_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/irk_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ImplicitRungeKuttaExport::ImplicitRungeKuttaExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : RungeKuttaExport( _userInteraction,_commonHeaderName )
{
	diffsDim = 0;
	inputDim = 0;
	numIts = 3; 		// DEFAULT value
	numItsInit = 0; 	// DEFAULT value
	REUSE = BT_TRUE;
	CONTINUOUS_OUTPUT = BT_FALSE;

	solver = 0;
}

ImplicitRungeKuttaExport::ImplicitRungeKuttaExport( const ImplicitRungeKuttaExport& arg ) : RungeKuttaExport( arg )
{
	diffsDim = 0;
	inputDim = 0;
	numIts = arg.numIts ;
	numItsInit = arg.numItsInit ;
    diffs_OUTPUTS = arg.diffs_OUTPUTS;
    OUTPUTS = arg.OUTPUTS;
	outputGrids = arg.outputGrids;
    solver = arg.solver;
	REUSE = arg.REUSE;;
	CONTINUOUS_OUTPUT = arg.CONTINUOUS_OUTPUT;
}


ImplicitRungeKuttaExport::~ImplicitRungeKuttaExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}


ImplicitRungeKuttaExport& ImplicitRungeKuttaExport::operator=( const ImplicitRungeKuttaExport& arg ){

    if( this != &arg ){

		RungeKuttaExport::operator=( arg );
		copy( arg );
    }
    return *this;
}


returnValue ImplicitRungeKuttaExport::setDifferentialEquation(	const Expression& rhs )
{
	Parameter         dummy0;
	Control           dummy1;
	DifferentialState dummy2;
	AlgebraicState 	  dummy3;
	DifferentialStateDerivative dummy4;
	dummy0.clearStaticCounters();
	dummy1.clearStaticCounters();
	dummy2.clearStaticCounters();
	dummy3.clearStaticCounters();
	dummy4.clearStaticCounters();

	x = DifferentialState(NX);
	dx = DifferentialStateDerivative(NDX);
	z = AlgebraicState(NXA);
	u = Control(NU);
	p = Parameter(NP);
	
//	uint i;
	
	if( NDX > 0 && NDX != NX ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	if( rhs.getNumRows() != (NX+NXA) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	
	DifferentialEquation f;
	f << rhs;
	
	DifferentialEquation g;
	g << forwardDerivative( rhs, x );
	g << forwardDerivative( rhs, z );
	g << forwardDerivative( rhs, u );
	g << forwardDerivative( rhs, dx );

	setup();
	return (ODE.init( f,"acado_rhs",NX,NXA,NU ) & diffs_ODE.init( g,"acado_diffs",NX,NXA,NU ) );
}


returnValue ImplicitRungeKuttaExport::setModel(	const String& _rhs, const String& _diffs_rhs ) {

	IntegratorExport::setModel( _rhs, _diffs_rhs );

	Parameter         dummy0;
	Control           dummy1;
	DifferentialState dummy2;
	AlgebraicState 	  dummy3;
	DifferentialStateDerivative dummy4;
	dummy0.clearStaticCounters();
	dummy1.clearStaticCounters();
	dummy2.clearStaticCounters();
	dummy3.clearStaticCounters();
	dummy4.clearStaticCounters();

	x = DifferentialState(NX);
	dx = DifferentialStateDerivative(NDX);
	z = AlgebraicState(NXA);
	u = Control(NU);
	p = Parameter(NP);

	setup();

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	solver->getDataDeclarations( declarations,dataStruct );
	
	if( EXPORT_RHS ) {
		ExportVariable max = ODE.getGlobalExportVariable();
		if( diffs_ODE.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_ODE.getGlobalExportVariable();
		}
		uint i;
		for( i = 0; i < OUTPUTS.size(); i++ ) {
			if( OUTPUTS[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
				max = OUTPUTS[i].getGlobalExportVariable();
			}
			if( diffs_OUTPUTS[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
				max = diffs_OUTPUTS[i].getGlobalExportVariable();
			}
		}
		declarations.addDeclaration( max,dataStruct );
	}

	declarations.addDeclaration( rk_ttt,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( rk_kkk,dataStruct );
	declarations.addDeclaration( rk_sol,dataStruct );
	declarations.addDeclaration( rk_A,dataStruct );
	declarations.addDeclaration( rk_b,dataStruct );
	declarations.addDeclaration( rk_rhsTemp,dataStruct );
	declarations.addDeclaration( rk_diffsTemp,dataStruct );
	declarations.addDeclaration( rk_num,dataStruct );
	declarations.addDeclaration( rk_diffsPrev,dataStruct );
	declarations.addDeclaration( rk_diffsNew,dataStruct );
	
	if( CONTINUOUS_OUTPUT ) {
		declarations.addDeclaration( rk_xPrev,dataStruct );
		declarations.addDeclaration( rk_rhsOutputTemp,dataStruct );
		declarations.addDeclaration( rk_diffsOutputTemp,dataStruct );
		declarations.addDeclaration( rk_outH,dataStruct );
		declarations.addDeclaration( rk_out2,dataStruct );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::getFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	declarations.addDeclaration( integrate );
	solver->getFunctionDeclarations( declarations );

	if( EXPORT_RHS ) {
		declarations.addDeclaration( ODE );
		declarations.addDeclaration( diffs_ODE );
	}
	else {
		Function tmpFun;
		ExportODEfunction tmpExport(tmpFun, getNameODE());
		declarations.addDeclaration( tmpExport );
		tmpExport = ExportODEfunction(tmpFun, getNameDiffsODE());
		declarations.addDeclaration( tmpExport );
	}
	uint i;
	if( EXPORT_RHS && CONTINUOUS_OUTPUT ) {
		for( i = 0; i < OUTPUTS.size(); i++ ) {
			declarations.addDeclaration( OUTPUTS[i] );
			declarations.addDeclaration( diffs_OUTPUTS[i] );
		}
	}
	else {
		for( i = 0; i < name_OUTPUTS.size(); i++ ) {
			Function tmpFun;
			ExportODEfunction tmpExport(tmpFun, getNameOUTPUT(i));
			declarations.addDeclaration( tmpExport );
			tmpExport = ExportODEfunction(tmpFun, getNameDiffsOUTPUT(i));
			declarations.addDeclaration( tmpExport );
		}
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::getCode(	ExportStatementBlock& code
									)
{
	if( EXPORT_RHS ) {
		code.addFunction( ODE );
		code.addStatement( "\n\n" );
		code.addFunction( diffs_ODE );
		code.addStatement( "\n\n" );

		if( CONTINUOUS_OUTPUT ) {
			uint i;
			for( i = 0; i < OUTPUTS.size(); i++ ) {
				code.addFunction( OUTPUTS[i] );
				code.addStatement( "\n\n" );
				code.addFunction( diffs_OUTPUTS[i] );
				code.addStatement( "\n\n" );
			}
		}
	}

	// export RK scheme
	uint run2, run3, run4, run5;
	String tempString;
	
	Matrix nul = zeros( 1,NX+NXA );
	initializeButcherTableau();
	initializeDDMatrix();
	initializeCoefficients();
	   
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();    

	ExportVariable Ah ( "Ah_mat",  AA*=h );
	ExportVariable Bh( "bb*h", Matrix( bb, BT_TRUE )*=h );
	ExportVariable DM( "DD", DD );
	ExportVariable zeroM( "nul", nul );
	
	ExportIndex i( "i" );
	ExportIndex j( "j" );
	ExportIndex k( "k" );
	ExportIndex run( "run" );
	ExportIndex run1( "run1" );
	
	code.addDeclaration( Ah );
	code.addLinebreak( 2 );

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !hasEquidistantGrid() ) {
		ExportVariable numStepsV( "numSteps", numSteps );
		code.addDeclaration( numStepsV );
		code.addLinebreak( 2 );
		integrate.addStatement( String( "int " ) << numInt.getName() << " = " << numStepsV.getName() << "[" << rk_index.getName() << "];\n" );
	}
	
	std::vector<ExportVariable> gridVariables;
	for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
		Vector gridV(outputGrids[run5].getNumIntervals());
		for( uint index = 1; index <= outputGrids[run5].getNumIntervals(); index++ ) {
			gridV( index-1 ) = outputGrids[run5].getTime( index );
		}
		ExportVariable gridVariable( (String)"gridOutput" << run5, gridV );
		code.addDeclaration( gridVariable );
		gridVariables.push_back( gridVariable );
	}

	integrate.addIndex( i );
	integrate.addIndex( j );
	integrate.addIndex( k );
	if( rk_outputs.size() > 0 ) {
		integrate.addDeclaration( polynEvalVar );
	}
	integrate.addIndex( run );
	integrate.addIndex( run1 );
	integrate.addStatement( rk_ttt == Matrix(grid.getFirstTime()) );

	// initialize sensitivities:
	Matrix idX    = eye( NX );
	integrate.addStatement( rk_eta.getCols( NX+NXA,NX+NXA+NX*NX ) == idX.makeVector().transpose() );
	if( NXA > 0 ) {
		Matrix zeroXA = zeros( NXA,NX );
		integrate.addStatement( rk_eta.getCols( NX+NXA+NX*NX,NX+NXA+NX*NX+NXA*NX ) == zeroXA.makeVector().transpose() );
	}
	if( NU > 0 ) {
		Matrix zeroXU = zeros( NX+NXA,NU );
		integrate.addStatement( rk_eta.getCols( (NX+NXA)*(1+NX),(NX+NXA)*(1+NX+NU) ) == zeroXU.makeVector().transpose() );
	}

	integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
	if( CONTINUOUS_OUTPUT ) integrate.addStatement( rk_xPrev.getCols( NX+NXA,inputDim-diffsDim ) == rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) );
	
	integrate.addLinebreak( );

    // integrator loop:
	ExportForLoop tmpLoop( run, 0, grid.getNumIntervals() );
	ExportStatementBlock *loop;
	if( hasEquidistantGrid() ) {
		loop = &tmpLoop;
	}
	else {
	    loop = &integrate;
		loop->addStatement( String("for(") << run.getName() << " = 0; " << run.getName() << " < " << numInt.getName() << "; " << run.getName() << "++ ) {\n" );
	}

	// Set rk_diffsPrev:
	ExportForLoop loopTemp( i,0,NX+NXA );
	loopTemp.addStatement( rk_diffsPrev.getSubMatrix( i,i+1,0,NX ) == rk_eta.getCols( i*NX+NX+NXA,i*NX+NX+NXA+NX ) );
	if( NU > 0 ) loopTemp.addStatement( rk_diffsPrev.getSubMatrix( i,i+1,NX,NX+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1),i*NU+(NX+NXA)*(NX+1)+NU ) );
	loop->addStatement( loopTemp );


	if( REUSE ) loop->addStatement( String( "if( " ) << rk_num.get(0,0) << " == 0 ) {\n" );
	// Initialization iterations:
	ExportForLoop loop1( i,0,numItsInit+1 ); // NOTE: +1 because 0 will lead to NaNs, so the minimum number of iterations is 1 at the initialization
	ExportForLoop loop11( run1,0,numStages );
	loop11.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
	if( NXA > 0 ) loop11.addStatement( rk_xxx.getCols( NX,NX+NXA ) == rk_kkk.getSubMatrix( run1,run1+1,NX,NX+NXA ) );
	if( NDX > 0 ) loop11.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );

	loop11.addFunctionCall( getNameODE(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
	loop11.addFunctionCall( getNameDiffsODE(), rk_xxx, rk_diffsTemp.getAddress(0,0) );
	// matrix rk_A: TODO: EXPORTINDEX STUFF FROM MILAN
	for( run2 = 0; run2 < NX+NXA; run2++ ) {
		for( run3 = 0; run3 < numStages; run3++ ) { // differential states
			if( NDX == 0 ) {
				loop11.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*NX,run3*NX+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,run2*NX,run2*NX+NX ) );
				loop11.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) " );
				loop11.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*NX+run2,run3*NX+run2+1 ) -= 1 );
			}
			else {
				loop11.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*NX,run3*NX+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,run2*NX,run2*NX+NX ) );
				loop11.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) {\n" );
				loop11.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*NX,run3*NX+NX ) += rk_diffsTemp.getSubMatrix( 0,1,(NX+NXA)*(NX+NXA+NU)+run2*NDX,(NX+NXA)*(NX+NXA+NU)+run2*NDX+NDX ) );
				loop11.addStatement( String( "}\n" ) );
			}
		}
		if( NXA > 0 ) {
			for( run3 = 0; run3 < numStages; run3++ ) { // algebraic states
				loop11.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) {\n" );
				loop11.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,numStages*NX+run3*NXA,numStages*NX+run3*NXA+NXA ) == rk_diffsTemp.getSubMatrix( 0,1,(NX+NXA)*NX+run2*NXA,(NX+NXA)*NX+run2*NXA+NXA ) );
				loop11.addStatement( String( "}\n else {\n" ) );
				loop11.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,numStages*NX+run3*NXA,numStages*NX+run3*NXA+NXA ) == zeroM.getCols( 0,NXA ) );
				loop11.addStatement( String( "}\n" ) );
			}
		}
	}
	// matrix rk_b:
	if( NDX == 0 ) {
		loop11.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) - rk_rhsTemp.getCols( 0,NX ) );
	}
	else {
		loop11.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == zeroM.getCols( 0,NX ) - rk_rhsTemp.getCols( 0,NX ) );
	}
	if( NXA > 0 ) loop11.addStatement( rk_b.getCols( run1*(NX+NXA)+NX,(run1+1)*(NX+NXA) ) == zeroM.getCols( 0,NXA ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
	loop1.addStatement( loop11 );

	loop1.addFunctionCall( solver->getNameSolveFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loopTemp = ExportForLoop( j,0,numStages );
	loopTemp.addStatement( rk_kkk.getSubMatrix( j,j+1,0,NX ) += rk_sol.getCols( j*NX,j*NX+NX ) );													// differential states
	if(NXA > 0) loopTemp.addStatement( rk_kkk.getSubMatrix( j,j+1,NX,NX+NXA ) += rk_sol.getCols( j*NXA+numStages*NX,j*NXA+numStages*NX+NXA ) );		// algebraic states
	loop1.addStatement( loopTemp );
	loop->addStatement( loop1 );
	if( REUSE ) loop->addStatement( String( "}\n" ) );


	// the rest (numIts) of the Newton iterations with reuse of the Jacobian (no evaluation or factorization needed)
	ExportForLoop loop2( i,0,numIts );
	ExportForLoop loop21( run1,0,numStages );
	loop21.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
	if( NXA > 0 ) loop21.addStatement( rk_xxx.getCols( NX,NX+NXA ) == rk_kkk.getSubMatrix( run1,run1+1,NX,NX+NXA ) );
	if( NDX > 0 ) loop21.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );

	loop21.addFunctionCall( getNameODE(), rk_xxx, rk_rhsTemp.getAddress(0,0) );

	// matrix rk_b:
	if( NDX == 0 ) {
		loop21.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) - rk_rhsTemp.getCols( 0,NX ) );
	}
	else {
		loop21.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == zeroM.getCols( 0,NX ) - rk_rhsTemp.getCols( 0,NX ) );
	}
	if( NXA > 0 ) {
		loop21.addStatement( rk_b.getCols( run1*(NX+NXA)+NX,(run1+1)*(NX+NXA) ) == zeroM.getCols( 0,NXA ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
	}
	loop2.addStatement( loop21 );
	loop2.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loopTemp = ExportForLoop( j,0,numStages );
	loopTemp.addStatement( rk_kkk.getSubMatrix( j,j+1,0,NX ) += rk_sol.getCols( j*NX,j*NX+NX ) );													// differential states
	if(NXA > 0) loopTemp.addStatement( rk_kkk.getSubMatrix( j,j+1,NX,NX+NXA ) += rk_sol.getCols( j*NXA+numStages*NX,j*NXA+numStages*NX+NXA ) );		// algebraic states
	loop2.addStatement( loopTemp );
	loop->addStatement( loop2 );

	// generate continuous output (except for the last grid point):
	for( run5 = 0; run5 < rk_outputs.size(); run5++ ) {
		ExportForLoop loop22( i,0,outputGrids[run5].getNumIntervals()-1 );
		evaluatePolynomial( loop22, rk_outH, gridVariables[run5], i, h );
		if( NXA > 0 || NDX > 0 ) evaluateDerivedPolynomial( loop22, rk_out2, gridVariables[run5], i );
		Vector dependencyX, dependencyZ, dependencyDX;
		if( EXPORT_RHS ) dependencyX = sumRow( outputExpressions[run5].getDependencyPattern( x ) );
		if( EXPORT_RHS && NXA > 0 ) dependencyZ = sumRow( outputExpressions[run5].getDependencyPattern( z ) );
		if( EXPORT_RHS && NDX > 0 ) dependencyDX = sumRow( outputExpressions[run5].getDependencyPattern( dx ) );
		for( run3 = 0; run3 < NX; run3++ ) {
			if( !EXPORT_RHS || (int)dependencyX(run3) != 0 ) {
				loop22.addStatement( rk_xxx.getCols( run3,run3+1 ) == rk_eta.getCols( run3,run3+1 ) + rk_outH*rk_kkk.getCol( run3 ) );
			}
		}
		for( run3 = 0; run3 < NXA; run3++ ) {
			if( !EXPORT_RHS || (int)dependencyZ(run3) != 0 ) {
				loop22.addStatement( rk_xxx.getCols( NX+run3,NX+run3+1 ) == rk_out2*rk_kkk.getCol( NX+run3 ) );
			}
		}
		for( run3 = 0; run3 < NDX; run3++ ) {
			if( !EXPORT_RHS || (int)dependencyDX(run3) != 0 ) {
				loop22.addStatement( rk_xxx.getCols( inputDim-diffsDim+run3,inputDim-diffsDim+run3+1 ) == rk_out2*rk_kkk.getCol( run3 ) );
			}
		}
		loop22.addFunctionCall( getNameOUTPUT( run5 ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run5 );
		uint outputDim = outputGrids[run5].getNumIntervals( )*numOutputs*(NX+NU+1);
		for( run3 = 0; run3 < numOutputs; run3++ ) { // TODO: EXPORTINDEX STUFF FROM MILAN
			loop22.addStatement( String( rk_outputs[run5].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run3 << "] = " << rk_rhsOutputTemp.get( 0,run3 ) << ";\n" );
		}
		loop->addStatement( loop22 );
	}

	// solution calculated --> evaluate and save the necessary derivatives in rk_diffsTemp and update the matrix rk_A:
	ExportForLoop loop3( run1,0,numStages );
	loop3.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
	if( NXA > 0 ) loop3.addStatement( rk_xxx.getCols( NX,NX+NXA ) == rk_kkk.getSubMatrix( run1,run1+1,NX,NX+NXA ) );
	if( NDX > 0 ) loop3.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );

	loop3.addFunctionCall( getNameDiffsODE(), rk_xxx, rk_diffsTemp.getAddress(run1,0) );

	// matrix rk_A: TODO: EXPORTINDEX STUFF FROM MILAN
	for( run2 = 0; run2 < NX+NXA; run2++ ) {
		for( run3 = 0; run3 < numStages; run3++ ) { // differential states
			if( NDX == 0 ) {
				loop3.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*NX,run3*NX+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( run1,run1+1,run2*NX,run2*NX+NX ) );
				loop3.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) " );
				loop3.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*NX+run2,run3*NX+run2+1 ) -= 1 );
			}
			else {
				loop3.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*NX,run3*NX+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( run1,run1+1,run2*NX,run2*NX+NX ) );
				loop3.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) {" );
				loop3.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,run3*NX,run3*NX+NX ) += rk_diffsTemp.getSubMatrix( run1,run1+1,(NX+NXA)*(NX+NXA+NU)+run2*NDX,(NX+NXA)*(NX+NXA+NU)+run2*NDX+NDX ) );
				loop3.addStatement( String( "}\n" ) );
			}
		}
		if( NXA > 0 ) {
			for( run3 = 0; run3 < numStages; run3++ ) { // algebraic states
				loop3.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) {\n" );
				loop3.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,numStages*NX+run3*NXA,numStages*NX+run3*NXA+NXA ) == rk_diffsTemp.getSubMatrix( run1,run1+1,(NX+NXA)*NX+run2*NXA,(NX+NXA)*NX+run2*NXA+NXA ) );
				loop3.addStatement( String( "}\n else {\n" ) );
				loop3.addStatement( rk_A.getSubMatrix( run1*(NX+NXA)+run2,run1*(NX+NXA)+run2+1,numStages*NX+run3*NXA,numStages*NX+run3*NXA+NXA ) == zeroM.getCols( 0,NXA ) );
				loop3.addStatement( String( "}\n" ) );
			}
		}
	}
	loop->addStatement( loop3 );

	// update rk_eta:
	if( CONTINUOUS_OUTPUT ) loop->addStatement( rk_xPrev.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
	loop->addStatement( rk_eta.getCols( 0,NX ) += Bh*rk_kkk.getCols( 0,NX ) );
	Vector outTemp = evaluateDerivedPolynomial( 1.0 );
	ExportVariable tempCoefs( "tempCoefs", Matrix( outTemp, BT_TRUE ) );
	if( NXA > 0) loop->addStatement( rk_eta.getCols( NX,NX+NXA ) == tempCoefs*rk_kkk.getCols( NX,NX+NXA ) );

	// generate continuous output (only the last grid point):
	if( CONTINUOUS_OUTPUT ) {
		loop->addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
		if( NDX > 0 ) loop->addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == tempCoefs*rk_kkk.getCols( 0,NX ) );

	}
	for( run5 = 0; run5 < rk_outputs.size(); run5++ ) {
		run2 = outputGrids[run5].getNumIntervals()-1;
		loop->addFunctionCall( getNameOUTPUT( run5 ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run5 );
		uint outputDim = outputGrids[run5].getNumIntervals( )*numOutputs*(NX+NU+1);
		for( run3 = 0; run3 < numOutputs; run3++ ) { // TODO: EXPORTINDEX STUFF FROM MILAN
			loop->addStatement( String( rk_outputs[run5].getName() ) << "[run*" << String( outputDim ) << "+" << String( run2*numOutputs*(NX+NU+1)+run3 ) << "] = " << rk_rhsOutputTemp.get( 0,run3 ) << ";\n" );
		}
	}

	// derivatives wrt the states (IFT):
	ExportForLoop loop4( run1,0,NX );
	for( run2 = 0; run2 < numStages; run2++ ) { // TODO: EXPORTINDEX STUFF FROM MILAN
		for( run3 = 0; run3 < NX+NXA; run3++ ) {
			loop4.addStatement( rk_b.getCol( run2*(NX+NXA)+run3 ) == zeroM.getCols( 0,1 ) - rk_diffsTemp.getSubMatrix( run2,run2+1,run1+run3*NX,run1+run3*NX+1 ) );
		}
	}
	loop4.addStatement( String( "if( 0 == " ) << run1.getName() << " ) {\n" );	// factorization of the new matrix rk_A not yet calculated!
	loop4.addFunctionCall( solver->getNameSolveFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loop4.addStatement( String( "}\n else {\n" ) );
	loop4.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loop4.addStatement( String( "}\n" ) );

	// generate sensitivities wrt states for continuous output (except for the last grid point):
	for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
		ExportForLoop loop41( i,0,outputGrids[run2].getNumIntervals()-1 );
		evaluatePolynomial( loop41, rk_outH, gridVariables[run2], i, h );
		if( NXA > 0 || NDX > 0 ) evaluateDerivedPolynomial( loop41, rk_out2, gridVariables[run2], i );
		Vector dependencyX, dependencyZ, dependencyDX;
		if( EXPORT_RHS ) dependencyX = sumRow( outputExpressions[run2].getDependencyPattern( x ) );
		if( EXPORT_RHS && NXA > 0 ) dependencyZ = sumRow( outputExpressions[run2].getDependencyPattern( z ) );
		if( EXPORT_RHS && NDX > 0 ) dependencyDX = sumRow( outputExpressions[run2].getDependencyPattern( dx ) );
		for( run4 = 0; run4 < NX; run4++ ) {
			if( !EXPORT_RHS || (int)dependencyX(run4) != 0 ) {
				loop41.addStatement( String(rk_rhsTemp.get( 0,run4 )) << " = (" << run4 << " == " << run1.getName() << ");\n" );
				for( run5 = 0; run5 < numStages; run5++ ) {
					loop41.addStatement( rk_rhsTemp.getCol( run4 ) += rk_outH.getCol( run5 )*rk_sol.getCol( run5*NX+run4 ) );
				}
				loop41.addStatement( rk_xxx.getCol( run4 ) == rk_xPrev.getCol( run4 ) + rk_outH*rk_kkk.getCol( run4 ) );
			}
		}
		for( run4 = 0; run4 < NXA; run4++ ) {
			if( !EXPORT_RHS || (int)dependencyZ(run4) != 0 ) {
				loop41.addStatement( rk_rhsTemp.getCol( NX+run4 ) == rk_out2.getCol( 0 )*rk_sol.getCol( numStages*NX+run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop41.addStatement( rk_rhsTemp.getCol( NX+run4 ) += rk_out2.getCol( run5 )*rk_sol.getCol( numStages*NX+run5*NXA+run4 ) );
				}
				loop41.addStatement( rk_xxx.getCol( NX+run4 ) == rk_out2*rk_kkk.getCol( NX+run4 ) );
			}
		}
		for( run4 = 0; run4 < NDX; run4++ ) {
			if( !EXPORT_RHS || (int)dependencyDX(run4) != 0 ) {
				loop41.addStatement( rk_rhsTemp.getCol( NX+NXA+run4 ) == rk_out2.getCol( 0 )*rk_sol.getCol( run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop41.addStatement( rk_rhsTemp.getCol( NX+NXA+run4 ) += rk_out2.getCol( run5 )*rk_sol.getCol( run5*NX+run4 ) );
				}
				loop41.addStatement( rk_xxx.getCol( inputDim-diffsDim+run4 ) == rk_out2*rk_kkk.getCol( run4 ) );
			}
		}

		loop41.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run2 );
		uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
		for( run4 = 0; run4 < numOutputs; run4++ ) {
			if( EXPORT_RHS ) {
				dependencyX = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
				if( NXA > 0 ) dependencyZ = outputExpressions[run2].getRow(run4).getDependencyPattern( z );
				if( NDX > 0 ) dependencyDX = outputExpressions[run2].getRow(run4).getDependencyPattern( dx );
			}
			// TODO: EXPORTINDEX STUFF FROM MILAN
			loop41.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run1.getName() << "+" << numOutputs+run4*NX << "] = 0.0;\n" );
			for( run5 = 0; run5 < NX; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyX(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop41.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run1.getName() << "+" << numOutputs+run4*NX << "] += " << rk_rhsTemp.get( 0,run5 ) << "*" << tempString << ";\n" );
				}
			}
			for( run5 = 0; run5 < NXA; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyZ(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop41.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run1.getName() << "+" << numOutputs+run4*NX << "] += " << rk_rhsTemp.get( 0,NX+run5 ) << "*" << tempString << ";\n" );
				}
			}
			for( run5 = 0; run5 < NDX; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyDX(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,numOutputs*(NX+NXA+NU)+run4*NDX+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop41.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run1.getName() << "+" << numOutputs+run4*NX << "] += " << rk_rhsTemp.get( 0,NX+NXA+run5 ) << "*" << tempString << ";\n" );
				}
			}
		}
		loop4.addStatement( loop41 );
	}

	if( NDX > 0 ) {
		ExportForLoop loop42( i,0,NDX );
		loop42.addStatement( rk_rhsTemp.getCol( i ) == tempCoefs.getCol( 0 )*rk_sol.getCol( i ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop42.addStatement( rk_rhsTemp.getCol( i ) += tempCoefs.getCol( run3 )*rk_sol.getCol( i+run3*NX ) );
		}
		loop4.addStatement( loop42 );
	}

	// TODO:  THE CODE THAT IS COMMENTED OUT SHOULD ACTUALLY BE USED AFTER MILANS UPDATE !!
//	// update rk_diffsNew with the new sensitivities:
//	ExportForLoop loop43( i,0,NX );
//	loop43.addStatement( String(rk_diffsNew.get( i,run1 )) << " = (" << i.getName() << " == " << run1.getName() << ");\n" );
//	for (run3 = 0; run3 < numStages; run3++ ) {
//		loop43.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1,run1+1 ) += Bh.getCol( run3 )*rk_sol.getCol( i+run3*NX ) );
//	}
//	loop4.addStatement( loop43 );
//	if( NXA > 0 ) {
//		ExportForLoop loop44( i,0,NXA );
//		loop44.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1,run1+1 ) == tempCoefs.getCol( 0 )*rk_sol.getCol( i+numStages*NX ) );
//		for (run3 = 1; run3 < numStages; run3++ ) {
//			loop44.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1,run1+1 ) += tempCoefs.getCol( run3 )*rk_sol.getCol( i+numStages*NX+run3*NXA ) );
//		}
//		loop4.addStatement( loop44 );
//	}
	// update rk_diffsNew with the new sensitivities:
	for( run2 = 0; run2 < NX; run2++ ) {
		loop4.addStatement( String(rk_diffsNew.get( run2,run1 )) << " = (" << run2 << " == " << run1.getName() << ");\n" );
		for (run3 = 0; run3 < numStages; run3++ ) {
			loop4.addStatement( rk_diffsNew.getSubMatrix( run2,run2+1,run1,run1+1 ) += Bh.getCol( run3 )*rk_sol.getCol( run3*NX+run2 ) );
		}
	}
	for( run2 = 0; run2 < NXA; run2++ ) {
		loop4.addStatement( rk_diffsNew.getSubMatrix( NX+run2,NX+run2+1,run1,run1+1 ) == tempCoefs.getCol( 0 )*rk_sol.getCol( numStages*NX+run2 ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop4.addStatement( rk_diffsNew.getSubMatrix( NX+run2,NX+run2+1,run1,run1+1 ) += tempCoefs.getCol( run3 )*rk_sol.getCol( numStages*NX+run3*NXA+run2 ) );
		}
	}

	// generate sensitivities wrt states for continuous output (only the last grid point):
	if( CONTINUOUS_OUTPUT ) {
		loop4.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
		if( NDX > 0 ) loop4.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == tempCoefs*rk_kkk.getCols( 0,NX ) );

	}
	for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
		run3 = outputGrids[run2].getNumIntervals()-1;
		loop4.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run2 );
		uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
		for( run4 = 0; run4 < numOutputs; run4++ ) {
			Vector dependencyX, dependencyZ, dependencyDX;
			if( EXPORT_RHS ) dependencyX = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
			if( EXPORT_RHS && NXA > 0 ) dependencyZ = outputExpressions[run2].getRow(run4).getDependencyPattern( z );
			if( EXPORT_RHS && NDX > 0 ) dependencyDX = outputExpressions[run2].getRow(run4).getDependencyPattern( dx );
			// TODO: EXPORTINDEX STUFF FROM MILAN
			loop4.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << run1.getName() << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX ) << "] = 0.0;\n" );
			for( run5 = 0; run5 < NX; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyX(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop4.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << run1.getName() << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX ) << "] += " << rk_diffsNew.get( run5,run1 ) << "*" << tempString << ";\n" );
				}
			}
			for( run5 = 0; run5 < NXA; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyZ(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop4.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << run1.getName() << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX ) << "] += " << rk_diffsNew.get( NX+run5,run1 ) << "*" << tempString << ";\n" );
				}
			}
			for( run5 = 0; run5 < NDX; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyDX(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,numOutputs*(NX+NXA+NU)+run4*NDX+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop4.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << run1.getName() << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX ) << "] += " << rk_rhsTemp.get( 0,run5 ) << "*" << tempString << ";\n" );
				}
			}
		}
	}
	loop->addStatement( loop4 );


	// derivatives wrt the control inputs (IFT):
	if( NU > 0 ) {
	ExportForLoop loop5( run1,0,NU );
	for( run2 = 0; run2 < numStages; run2++ ) { // TODO: EXPORTINDEX STUFF FROM MILAN
		for( run3 = 0; run3 < NX+NXA; run3++ ) {
			loop5.addStatement( rk_b.getCol( run2*(NX+NXA)+run3 ) == zeroM.getCols( 0,1 ) - rk_diffsTemp.getSubMatrix( run2,run2+1,run1+(NX+NXA)*(NX+NXA)+run3*NU,run1+(NX+NXA)*(NX+NXA)+run3*NU+1 ) );
		}
	}
	loop5.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );

	// generate sensitivities wrt controls for continuous output (except for the last grid point):
	for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
		ExportForLoop loop51( i,0,outputGrids[run2].getNumIntervals()-1 );
		evaluatePolynomial( loop51, rk_outH, gridVariables[run2], i, h );
		if( NXA > 0 || NDX > 0 ) evaluateDerivedPolynomial( loop51, rk_out2, gridVariables[run2], i );
		Vector dependencyX, dependencyZ, dependencyDX;
		if( EXPORT_RHS ) dependencyX = sumRow( outputExpressions[run2].getDependencyPattern( x ) );
		if( EXPORT_RHS && NXA > 0 ) dependencyZ = sumRow( outputExpressions[run2].getDependencyPattern( z ) );
		if( EXPORT_RHS && NDX > 0 ) dependencyDX = sumRow( outputExpressions[run2].getDependencyPattern( dx ) );
		for( run4 = 0; run4 < NX; run4++ ) {
			if( !EXPORT_RHS || (int)dependencyX(run4) != 0 ) {
				loop51.addStatement( rk_rhsTemp.getCol( run4 ) == rk_outH.getCol( 0 )*rk_sol.getCol( run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop51.addStatement( rk_rhsTemp.getCol( run4 ) += rk_outH.getCol( run5 )*rk_sol.getCol( run5*NX+run4 ) );
				}
				loop51.addStatement( rk_xxx.getCol( run4 ) == rk_xPrev.getCol( run4 ) + rk_outH*rk_kkk.getCol( run4 ) );
			}
		}
		for( run4 = 0; run4 < NXA; run4++ ) {
			if( !EXPORT_RHS || (int)dependencyZ(run4) != 0 ) {
				loop51.addStatement( rk_rhsTemp.getCol( NX+run4 ) == rk_out2.getCol( 0 )*rk_sol.getCol( numStages*NX+run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop51.addStatement( rk_rhsTemp.getCol( NX+run4 ) += rk_out2.getCol( run5 )*rk_sol.getCol( numStages*NX+run5*NXA+run4 ) );
				}
				loop51.addStatement( rk_xxx.getCol( NX+run4 ) == rk_out2*rk_kkk.getCol( NX+run4 ) );
			}
		}
		for( run4 = 0; run4 < NDX; run4++ ) {
			if( !EXPORT_RHS || (int)dependencyDX(run4) != 0 ) {
				loop51.addStatement( rk_rhsTemp.getCol( NX+NXA+run4 ) == rk_out2.getCol( 0 )*rk_sol.getCol( run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop51.addStatement( rk_rhsTemp.getCol( NX+NXA+run4 ) += rk_out2.getCol( run5 )*rk_sol.getCol( run5*NX+run4 ) );
				}
				loop51.addStatement( rk_xxx.getCol( inputDim-diffsDim+run4 ) == rk_out2*rk_kkk.getCol( run4 ) );
			}
		}

		loop51.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run2 );
		uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
		for( run4 = 0; run4 < numOutputs; run4++ ) {
			if( EXPORT_RHS ) {
				dependencyX = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
				if( NXA > 0 ) dependencyZ = outputExpressions[run2].getRow(run4).getDependencyPattern( z );
				if( NDX > 0 ) dependencyDX = outputExpressions[run2].getRow(run4).getDependencyPattern( dx );
			}
			// TODO: EXPORTINDEX STUFF FROM MILAN
			loop51.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run1.getName() << "+" << numOutputs*(1+NX)+run4*NU << "] = " <<  rk_diffsOutputTemp.get( 0,run1+numOutputs*(NX+NXA)+run4*NU ) << ";\n" );
			for( run5 = 0; run5 < NX; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyX(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop51.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run1.getName() << "+" << numOutputs*(1+NX)+run4*NU << "] += " << rk_rhsTemp.get( 0,run5 ) << "*" << tempString << ";\n" );
				}
			}
			for( run5 = 0; run5 < NXA; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyZ(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop51.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run1.getName() << "+" << numOutputs*(1+NX)+run4*NU << "] += " << rk_rhsTemp.get( 0,NX+run5 ) << "*" << tempString << ";\n" );
				}
			}
			for( run5 = 0; run5 < NDX; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyDX(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,numOutputs*(NX+NXA+NU)+run4*NDX+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop51.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+i*" << String( numOutputs*(NX+NU+1) ) << "+" << run1.getName() << "+" << numOutputs*(1+NX)+run4*NU << "] += " << rk_rhsTemp.get( 0,NX+NXA+run5 ) << "*" << tempString << ";\n" );
				}
			}
		}
		loop5.addStatement( loop51 );
	}

	if( NDX > 0 ) {
		ExportForLoop loop52( i,0,NDX );
		loop52.addStatement( rk_rhsTemp.getCol( i ) == tempCoefs.getCol( 0 )*rk_sol.getCol( i ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop52.addStatement( rk_rhsTemp.getCol( i ) += tempCoefs.getCol( run3 )*rk_sol.getCol( i+run3*NX ) );
		}
		loop5.addStatement( loop52 );
	}

//	// TODO:  THE CODE THAT IS COMMENTED OUT SHOULD ACTUALLY BE USED AFTER MILANS UPDATE !!
//	// update rk_diffsNew with the new sensitivities:
//	ExportForLoop loop53( i,0,NX );
//	loop53.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1+NX,run1+NX+1 ) == Bh.getCol( 0 )*rk_sol.getCol( i ) );
//	for (run3 = 1; run3 < numStages; run3++ ) {
//		loop53.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1+NX,run1+NX+1 ) += Bh.getCol( run3 )*rk_sol.getCol( i+run3*NX ) );
//	}
//	loop5.addStatement( loop53 );
//	if( NXA > 0 ) {
//		ExportForLoop loop54( i,0,NXA );
//		loop54.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1+NX,run1+NX+1 ) == tempCoefs.getCol( 0 )*rk_sol.getCol( i+numStages*NX ) );
//		for (run3 = 1; run3 < numStages; run3++ ) {
//			loop54.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1+NX,run1+NX+1 ) += tempCoefs.getCol( run3 )*rk_sol.getCol( i+numStages*NX+run3*NXA ) );
//		}
//		loop5.addStatement( loop54 );
//	}
	// update rk_diffsNew with the new sensitivities:
	for( run2 = 0; run2 < NX; run2++ ) {
		loop5.addStatement( rk_diffsNew.getSubMatrix( run2,run2+1,run1+NX,run1+NX+1 ) == Bh.getCol( 0 )*rk_sol.getCol( run2 ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop5.addStatement( rk_diffsNew.getSubMatrix( run2,run2+1,run1+NX,run1+NX+1 ) += Bh.getCol( run3 )*rk_sol.getCol( run3*NX+run2 ) );
		}
	}
	for( run2 = 0; run2 < NXA; run2++ ) {
		loop5.addStatement( rk_diffsNew.getSubMatrix( NX+run2,NX+run2+1,run1+NX,run1+NX+1 ) == tempCoefs.getCol( 0 )*rk_sol.getCol( numStages*NX+run2 ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop5.addStatement( rk_diffsNew.getSubMatrix( NX+run2,NX+run2+1,run1+NX,run1+NX+1 ) += tempCoefs.getCol( run3 )*rk_sol.getCol( numStages*NX+run3*NXA+run2 ) );
		}
	}

	// generate sensitivities wrt controls for continuous output (only the last grid point):
	if( CONTINUOUS_OUTPUT ) {
		loop5.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
		if( NDX > 0 ) loop5.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == tempCoefs*rk_kkk.getCols( 0,NX ) );

	}
	for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
		run3 = outputGrids[run2].getNumIntervals()-1;
		loop5.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run2 );
		uint outputDim = outputGrids[run2].getNumIntervals( )*numOutputs*(NX+NU+1);
		for( run4 = 0; run4 < numOutputs; run4++ ) {
			Vector dependencyX, dependencyZ, dependencyDX;
			if( EXPORT_RHS ) dependencyX = outputExpressions[run2].getRow(run4).getDependencyPattern( x );
			if( EXPORT_RHS && NXA > 0 ) dependencyZ = outputExpressions[run2].getRow(run4).getDependencyPattern( z );
			if( EXPORT_RHS && NDX > 0 ) dependencyDX = outputExpressions[run2].getRow(run4).getDependencyPattern( dx );
			// TODO: EXPORTINDEX STUFF FROM MILAN
			loop5.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << run1.getName() << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU ) << "] = " <<  rk_diffsOutputTemp.get( 0,run1+numOutputs*(NX+NXA)+run4*NU ) << ";\n" );
			for( run5 = 0; run5 < NX; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyX(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,run4*NX+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop5.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << run1.getName() << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU ) << "] += " << rk_diffsNew.get( run5,run1+NX ) << "*" << tempString << ";\n" );
				}
			}
			for( run5 = 0; run5 < NXA; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyZ(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,numOutputs*NX+run4*NXA+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop5.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << run1.getName() << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU ) << "] += " << rk_diffsNew.get( NX+run5,run1+NX ) << "*" << tempString << ";\n" );
				}
			}
			for( run5 = 0; run5 < NDX; run5++ ) {
				if( !EXPORT_RHS || (int)dependencyDX(run5) != 0 ) {
					tempString = rk_diffsOutputTemp.get( 0,numOutputs*(NX+NXA+NU)+run4*NDX+run5 ); // TODO: EXPORTINDEX STUFF FROM MILAN
					loop5.addStatement( String( rk_outputs[run2].getName() ) << "[run*" << String( outputDim ) << "+" << run1.getName() << "+" << String( run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU ) << "] += " << rk_rhsTemp.get( 0,run5 ) << "*" << tempString << ";\n" );
				}
			}
		}
	}
	loop->addStatement( loop5 );
	}

	// Computation of the sensitivities using the chain rule:
	ExportForLoop loop01( i,0,NX+NXA );
	ExportForLoop loop02( j,0,NX ); // TODO: EXPORTINDEX STUFF FROM MILAN
	loop02.addStatement( String( "rk_eta[" ) << String( NX+NXA ) << "+i*" << String( NX ) << "+j] = acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "]*acadoWorkspace.rk_diffsPrev[j];\n" );
	ExportForLoop loop021( k,1,NX );
	loop021.addStatement( String( "rk_eta[" ) << String( NX+NXA ) << "+i*" << String( NX ) << "+j] += acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "+" << k.getName() << "]*acadoWorkspace.rk_diffsPrev[" << k.getName() << "*" << String( NX+NU ) << "+j];\n" );
	loop02.addStatement( loop021 );
	loop01.addStatement( loop02 );
	loop->addStatement( loop01 );

	if( NU > 0 ) {
		ExportForLoop loop03( i,0,NX+NXA );
		ExportForLoop loop04( j,0,NU ); // TODO: EXPORTINDEX STUFF FROM MILAN
		loop04.addStatement( String( "rk_eta[" ) << String( (NX+NXA)*(1+NX) ) << "+i*" << String( NU ) << "+j] = acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "+" << String( NX ) << "+j] + acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "]*acadoWorkspace.rk_diffsPrev[" << String( NX ) << "+j];\n" );
		ExportForLoop loop041( k,1,NX );
		loop041.addStatement( String( "rk_eta[" ) << String( (NX+NXA)*(1+NX) ) << "+i*" << String( NU ) << "+j] += acadoWorkspace.rk_diffsNew[i*" << String( NX+NU ) << "+" << k.getName() << "]*acadoWorkspace.rk_diffsPrev[" << k.getName() << "*" << String( NX+NU ) << "+" << String( NX ) << "+j];\n" );
		loop04.addStatement( loop041 );
		loop03.addStatement( loop04 );
		loop->addStatement( loop03 );
	}

	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !hasEquidistantGrid()) ) {
		loop->addStatement( String( "if( run > 0 ) {\n" ) );

		// chain rule for the sensitivities of the continuous output:
		for( run5 = 0; run5 < rk_outputs.size(); run5++ ) {
			ExportForLoop loop05( run1,0,outputGrids[run5].getNumIntervals() );
			uint numOutputs = getDimOUTPUT( run5 );
			uint outputDim = outputGrids[run5].getNumIntervals( )*numOutputs*(NX+NU+1);
			ExportForLoop loop051( k,0,numOutputs*(NX+NU) ); // TODO: EXPORTINDEX STUFF FROM MILAN
			loop051.addStatement( String( rk_diffsOutputTemp.get( 0,k ) ) << " = " << rk_outputs[run5].getName() << "[run*" << String( outputDim ) << "+(" << run1.getName() << "*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs ) << "+" << k.getName() << "];\n" );
			loop05.addStatement( loop051 );

			ExportForLoop loop06( i,0,numOutputs );
			ExportForLoop loop07( j,0,NX ); // TODO: EXPORTINDEX STUFF FROM MILAN
			loop07.addStatement( String( rk_outputs[run5].getName() ) << "[run*" << String( outputDim ) << "+(" << run1.getName() << "*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs ) << "+i*" << String( NX ) << "+j] = acadoWorkspace.rk_diffsOutputTemp[i*" << String( NX ) << "]*acadoWorkspace.rk_diffsPrev[j];\n" );
			ExportForLoop loop071( k,1,NX );
			loop071.addStatement( String( rk_outputs[run5].getName() ) << "[run*" << String( outputDim ) << "+(" << run1.getName() << "*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs ) << "+i*" << String( NX ) << "+j] += acadoWorkspace.rk_diffsOutputTemp[i*" << String( NX ) << "+" << k.getName() << "]*acadoWorkspace.rk_diffsPrev[" << k.getName() << "*" << String( NX+NU ) << "+j];\n" );
			loop07.addStatement( loop071 );
			loop06.addStatement( loop07 );
			loop05.addStatement( loop06 );

			if( NU > 0 ) {
				ExportForLoop loop08( i,0,numOutputs );
				ExportForLoop loop09( j,0,NU ); // TODO: EXPORTINDEX STUFF FROM MILAN
				loop09.addStatement( String( rk_outputs[run5].getName() ) << "[run*" << String( outputDim ) << "+(" << run1.getName() << "*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs*(1+NX) ) << "+i*" << String( NU ) << "+j] = acadoWorkspace.rk_diffsOutputTemp[" << String( numOutputs*NX ) << "+i*" << String( NU ) << "+j] + acadoWorkspace.rk_diffsOutputTemp[i*" << String( NX ) << "]*acadoWorkspace.rk_diffsPrev[" << String( NX ) << "+j];\n" );
				ExportForLoop loop091( k,1,NX );
				loop091.addStatement( String( rk_outputs[run5].getName() ) << "[run*" << String( outputDim ) << "+(" << run1.getName() << "*" << String( numOutputs*(NX+NU+1) ) << ")+" << String( numOutputs*(1+NX) ) << "+i*" << String( NU ) << "+j] += acadoWorkspace.rk_diffsOutputTemp[i*" << String( NX ) << "+" << k.getName() << "]*acadoWorkspace.rk_diffsPrev[" << k.getName() << "*" << String( NX+NU ) << "+" << String( NX ) << "+j];\n" );
				loop09.addStatement( loop091 );
				loop08.addStatement( loop09 );
				loop05.addStatement( loop08 );
			}
			loop->addStatement( loop05 );
		}
		loop->addStatement( String( "}\n" ) );
	}

	loop->addStatement( rk_ttt += Matrix(h) );
	loop->addStatement( String( rk_num.get(0,0) ) << " += 1;\n" );

    // end of the integrator loop.
    if( !hasEquidistantGrid() ) {
		loop->addStatement( "}\n" );
	}
    else {
    	integrate.addStatement( *loop );
    }
	code.addFunction( integrate );
    code.addLinebreak( 2 );

	solver->getCode( code );

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::initializeDDMatrix( )
{
	uint i, j, k;
	DD = Matrix( numStages, numStages );
	
	for( i = 0; i < numStages; i++ ) {
		for( j = 0; j < numStages; j++ ) {
			DD( i, j ) = 1;
			for( k = 0; k < numStages; k++ ) {
				if( k != j ) {
					DD( i, j ) *= ((1+cc(i))-cc(k))/(cc(j)-cc(k));
				}
			}
		}
	}
    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::initializeCoefficients( )
{
	uint i, j, k, index;
	double sum;
	Vector cVec( numStages-1 );
	Vector products;
	coeffs = Matrix( numStages, numStages );
	
	for( i = 0; i < numStages; i++ ) {
		for( j = 0; j < numStages; j++ ) {
			coeffs( i, j ) = 1/((double) numStages-i);
			index = 0;
			for( k = 0; k < numStages; k++ ) {
				if( k != j ) {
					coeffs( i, j ) *= 1/((double) cc(j)-cc(k));
					cVec(index) = cc(k);
					index++;
				}
			}
			
			if( i > 0 ) {
				products = computeCombinations( cVec, 0, i );
				sum = 0.0;
				for( k = 0; k < products.getDim(); k++ ) {
					sum += products(k);
				}
				if( i%2 == 0 ) {
					coeffs( i, j ) *= sum;
				}
				else {
					coeffs( i, j ) *= (-1.0*sum);
				}
			}
		}
	}
	
    return SUCCESSFUL_RETURN;
}


Vector ImplicitRungeKuttaExport::computeCombinations( const Vector& cVec, uint index, uint numEls ) {
	uint k, l;
	Vector products;
	
	if( numEls == 0 ) {
		products = Vector(1);
		products(0) = 1;
		return products;
	}
	products = Vector();
	for( k = index; k < cVec.getDim()-numEls+1; k++ ) {
		Vector temp = computeCombinations( cVec, k+1, numEls-1 );
		for( l = 0; l < temp.getDim(); l++ ) {
			temp(l) *= cVec(k);
		}
		products.append(temp);
	}
	
	return products;
}


Vector ImplicitRungeKuttaExport::evaluatePolynomial( double time )
{
	uint i, j;
	Vector coeffsPolyn( numStages );
	
	for( j = 0; j < numStages; j++ ) {
		coeffsPolyn( j ) = 0.0;
		for( i = 0; i < numStages; i++ ) {
			coeffsPolyn( j ) += pow( time, static_cast<int>(numStages-i) )*coeffs( i,j );
		}
	} 
	
    return coeffsPolyn;
}


returnValue ImplicitRungeKuttaExport::evaluatePolynomial( ExportStatementBlock& block, const ExportVariable& variable, const ExportVariable& gridVariable, const ExportIndex& indexTime, double h )
{
	uint i, j;

	block.addStatement( (String)polynEvalVar.getName() << " = " << gridVariable.getName() << "[" << indexTime.getName() << "];\n" );
	for( i = 0; i < numStages; i++ ) {
		for( j = 0; j < numStages; j++ ) {
			if( i == 0 ) {
				block.addStatement( (String)variable.getFullName() << "[" << j << "] = " << polynEvalVar.getName() << "*" << coeffs( numStages-1-i,j ) << ";\n" );
			}
			else {
				block.addStatement( (String)variable.getFullName() << "[" << j << "] += " << polynEvalVar.getName() << "*" << coeffs( numStages-1-i,j ) << ";\n" );
			}
		}
		if( i < (numStages-1) ) block.addStatement( (String)polynEvalVar.getName() << " *= " << gridVariable.getName() << "[" << indexTime.getName() << "];\n" );
	}
	for( j = 0; j < numStages; j++ ) {
		block.addStatement( (String)variable.getFullName() << "[" << j << "] *= " << h << ";\n" );
	}

    return SUCCESSFUL_RETURN;
}


Vector ImplicitRungeKuttaExport::evaluateDerivedPolynomial( double time )
{
	uint i, j;
	Vector coeffsPolyn( numStages );

	// construct the Lagrange interpolating polynomials:
	for( i = 0; i < numStages; i++ ) {
		coeffsPolyn( i ) = 1.0;
		for( j = 0; j < numStages; j++ ) {
			if( i != j ) {
				coeffsPolyn( i ) *= (time-cc(j))/(cc(i)-cc(j));
			}
		}
	}

    return coeffsPolyn;
}


returnValue ImplicitRungeKuttaExport::evaluateDerivedPolynomial( ExportStatementBlock& block, const ExportVariable& variable, const ExportVariable& gridVariable, const ExportIndex& indexTime )
{
	uint i, j;
	
	// construct the Lagrange interpolating polynomials:
	for( i = 0; i < numStages; i++ ) {
		for( j = 0; j < numStages; j++ ) {
			if( (i == 0 && j == 1) || (i != j && j == 0) ) {
				block.addStatement( (String)variable.getFullName() << "[" << i << "] = (" << gridVariable.getName() << "[" << indexTime.getName() << "] - " << cc(j) << ")*" << 1/(cc(i)-cc(j)) << ";\n" );
			}
			else if( i != j ) {
				block.addStatement( (String)variable.getFullName() << "[" << i << "] *= (" << gridVariable.getName() << "[" << indexTime.getName() << "] - " << cc(j) << ")*" << 1/(cc(i)-cc(j)) << ";\n" );
			}
		}
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setup( )
{
	int intMode;
	userInteraction->get( IMPLICIT_INTEGRATOR_MODE,intMode ); 
	switch( (ImplicitIntegratorMode) intMode ) {
		case IFTR:
			REUSE = BT_TRUE;
			break;
		case IFT:
			REUSE = BT_FALSE;
			break;
		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}
	
	int newNumIts;
	userInteraction->get( IMPLICIT_INTEGRATOR_NUM_ITS,newNumIts ); 
	if (newNumIts >= 0) {
		numIts = newNumIts;
	}
	
	int newNumItsInit;
	userInteraction->get( IMPLICIT_INTEGRATOR_NUM_ITS_INIT,newNumItsInit );
	if (newNumItsInit >= 0) {
		numItsInit = newNumItsInit;
	}
	
	diffsDim = (NX+NXA)*(NX+NU);
	inputDim = (NX+NXA)*(NX+NU+1) + NU + NP;
	
	rk_num = ExportVariable( "rk_num", 1, 1, INT, ACADO_WORKSPACE, BT_TRUE );
	rk_ttt = ExportVariable( "rk_ttt", 1, 1, REAL, ACADO_WORKSPACE, BT_TRUE );
	rk_xxx = ExportVariable( "rk_xxx", 1, inputDim-diffsDim+NDX, REAL, ACADO_WORKSPACE );
	if( CONTINUOUS_OUTPUT ) rk_xPrev = ExportVariable( "rk_xPrev", 1, inputDim-diffsDim, REAL, ACADO_WORKSPACE );
	rk_kkk = ExportVariable( "rk_kkk", numStages, NX+NXA, REAL, ACADO_WORKSPACE );
	rk_sol = ExportVariable( "rk_sol", 1, numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_A = ExportVariable( "rk_A", numStages*(NX+NXA), numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_b = ExportVariable( "rk_b", 1, numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_rhsTemp = ExportVariable( "rk_rhsTemp", 1, NX+NXA+NDX, REAL, ACADO_WORKSPACE );
	rk_diffsTemp = ExportVariable( "rk_diffsTemp", numStages, (NX+NXA)*(NX+NXA+NU+NDX), REAL, ACADO_WORKSPACE );
	rk_diffsPrev = ExportVariable( "rk_diffsPrev", NX+NXA, NX+NU, REAL, ACADO_WORKSPACE );
	rk_diffsNew = ExportVariable( "rk_diffsNew", NX+NXA, NX+NU, REAL, ACADO_WORKSPACE );
	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
	if( hasEquidistantGrid() ) {
		integrate = ExportFunction( "integrate", rk_eta );
	}
	else {
		integrate = ExportFunction( "integrate", rk_index, rk_eta );
	}
	uint i;
	for( i = 0; i < rk_outputs.size(); i++ ) {
		integrate.addArgument( rk_outputs[i] );
	}
	integrate.addLinebreak( );	// TO MAKE SURE IT GETS EXPORTED
	
	// setup linear solver:
	int solverType;
	userInteraction->get( LINEAR_ALGEBRA_SOLVER,solverType );

	if ( solver )
		delete solver;
	solver = 0;

	switch( (LinearAlgebraSolver) solverType ) {
		case GAUSS_LU:
			solver = new ExportGaussElim( userInteraction,commonHeaderName );
			break;
		case HOUSEHOLDER_QR:
			solver = new ExportHouseholderQR( userInteraction,commonHeaderName );
			break;
		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}
	solver->setReuse( BT_TRUE ); 	// IFTR method
	solver->init( (NX+NXA)*numStages );
	solver->setup();

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setupOutput( const std::vector<Grid> outputGrids_, const std::vector<Expression> outputExpressions_ ) {
	returnValue val = SUCCESSFUL_RETURN;
	CONTINUOUS_OUTPUT = BT_TRUE;
	if( outputGrids_.size() != outputExpressions_.size() ) return ACADOERROR( RET_INVALID_ARGUMENTS ); 
	outputGrids = outputGrids_;
	outputExpressions = outputExpressions_;

	uint i;
	uint maxOutputs = 0;
	rk_outputs.clear();
	OUTPUTS.clear();
	diffs_OUTPUTS.clear();
	for( i = 0; i < outputGrids.size(); i++ ) {
		uint numOutputs = outputExpressions_[i].getDim();
		uint outputDim = outputGrids[i].getNumIntervals( )*grid.getNumIntervals()*numOutputs*(NX+NU+1);
		
		if( numOutputs > maxOutputs ) maxOutputs = numOutputs;
		
		DifferentialEquation f_Output;
		f_Output << outputExpressions_[i];
	
		DifferentialEquation g_Output;
		g_Output << forwardDerivative( outputExpressions_[i], x );
		g_Output << forwardDerivative( outputExpressions_[i], z );
		g_Output << forwardDerivative( outputExpressions_[i], u );
		g_Output << forwardDerivative( outputExpressions_[i], dx );
	
		ExportODEfunction OUTPUT, diffs_OUTPUT;
		val = val & OUTPUT.init( f_Output,String("acado_output")<<String(i)<<"_rhs",NX,NXA,NU ) & diffs_OUTPUT.init( g_Output,String("acado_output")<<String(i)<<"_diffs",NX,NXA,NU );
		
		ExportVariable rk_output( String("rk_output")<<String(i), 1, outputDim, REAL );
		rk_outputs.push_back( rk_output );
		
		OUTPUTS.push_back( OUTPUT );
		diffs_OUTPUTS.push_back( diffs_OUTPUT );
	}
	
	setup();
	rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, ACADO_WORKSPACE );
	rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(NX+NXA+NU+NDX), REAL, ACADO_WORKSPACE );
	rk_outH = ExportVariable( "rk_outH", 1, numStages, REAL, ACADO_WORKSPACE );
	rk_out2 = ExportVariable( "rk_out2", 1, numStages, REAL, ACADO_WORKSPACE );
	polynEvalVar = ExportVariable( "tmp_polyn", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );
	
	return ( val );
}


returnValue ImplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<String> _outputNames,
									  	  const std::vector<String> _diffs_outputNames,
										  const std::vector<uint> _dims_output ) {
	if( (ODE.getFunctionDim()) == 0 && (rk_outputs.size() + OUTPUTS.size() + diffs_OUTPUTS.size()) == 0) {
		CONTINUOUS_OUTPUT = BT_TRUE;
		if( outputGrids_.size() != _outputNames.size() || outputGrids_.size() != _diffs_outputNames.size() || outputGrids_.size() != _dims_output.size() ) {
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		}
		outputGrids = outputGrids_;
		name_OUTPUTS = _outputNames;
		name_diffs_OUTPUTS = _diffs_outputNames;
		num_OUTPUTS = _dims_output;

		uint i;
		uint maxOutputs = 0;
		rk_outputs.clear();
		OUTPUTS.clear();
		diffs_OUTPUTS.clear();
		for( i = 0; i < outputGrids.size(); i++ ) {
			uint numOutputs = num_OUTPUTS[i];
			uint outputDim = outputGrids[i].getNumIntervals( )*grid.getNumIntervals()*numOutputs*(NX+NU+1);

			if( numOutputs > maxOutputs ) maxOutputs = numOutputs;

			ExportVariable rk_output( String("rk_output")<<String(i), 1, outputDim, REAL );
			rk_outputs.push_back( rk_output );
		}

		setup();
		rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, ACADO_WORKSPACE );
		rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(NX+NXA+NU+NDX), REAL, ACADO_WORKSPACE );
		rk_outH = ExportVariable( "rk_outH", 1, numStages, REAL, ACADO_WORKSPACE );
		rk_out2 = ExportVariable( "rk_out2", 1, numStages, REAL, ACADO_WORKSPACE );
		polynEvalVar = ExportVariable( "tmp_polyn", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );

		EXPORT_RHS = BT_FALSE;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}


	return SUCCESSFUL_RETURN;
}



// PROTECTED:


returnValue ImplicitRungeKuttaExport::copy(	const ImplicitRungeKuttaExport& arg
									)
{
	numStages = arg.numStages;
	numIts = arg.numIts;
	numItsInit = arg.numItsInit;
	diffsDim = arg.diffsDim;
	inputDim = arg.inputDim;
	
	ODE = arg.ODE;
	OUTPUTS = arg.OUTPUTS;
	diffs_ODE = arg.diffs_ODE;
	diffs_OUTPUTS = arg.diffs_OUTPUTS;
	name_ODE = arg.name_ODE;
	name_OUTPUTS = arg.name_OUTPUTS;
	num_OUTPUTS = arg.num_OUTPUTS;
	name_diffs_ODE = arg.name_diffs_ODE;
	name_diffs_OUTPUTS = arg.name_diffs_OUTPUTS;
	grid = arg.grid;
	outputGrids = arg.outputGrids;
	solver = arg.solver;

	// ExportVariables
	rk_ttt = arg.rk_ttt;
	rk_xxx = arg.rk_xxx;
	rk_xPrev = arg.rk_xPrev;
	rk_kkk = arg.rk_kkk;
	rk_sol = arg.rk_sol;
	rk_A = arg.rk_A;
	rk_b = arg.rk_b;
	rk_rhsTemp = arg.rk_rhsTemp;
	rk_diffsTemp = arg.rk_diffsTemp;
	rk_diffsPrev = arg.rk_diffsPrev;
	rk_diffsNew = arg.rk_diffsNew;
	rk_eta = arg.rk_eta;
	rk_xPrev = arg.rk_xPrev;
	rk_rhsOutputTemp = arg.rk_rhsOutputTemp;
	rk_diffsOutputTemp = arg.rk_diffsOutputTemp;
	rk_outH = arg.rk_outH;
	rk_out2 = arg.rk_out2;
	polynEvalVar = arg.polynEvalVar;
	rk_outputs = arg.rk_outputs;
	
	// ExportFunctions
	integrate = arg.integrate;
	
	REUSE = arg.REUSE;
	CONTINUOUS_OUTPUT = arg.CONTINUOUS_OUTPUT;
	
	DD = arg.DD;
	coeffs = arg.coeffs;
	
	return SUCCESSFUL_RETURN;
}


uint ImplicitRungeKuttaExport::getNumIts() const
{
	return numIts;
}


uint ImplicitRungeKuttaExport::getNumItsInit() const
{
	return numItsInit;
}


CLOSE_NAMESPACE_ACADO

// end of file.
