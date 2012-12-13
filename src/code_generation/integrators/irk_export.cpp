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
	for( uint i = 0; i < rhs.getDim(); i++ ) {
		g << forwardDerivative( rhs(i), x );
		g << forwardDerivative( rhs(i), z );
		g << forwardDerivative( rhs(i), u );
		g << forwardDerivative( rhs(i), dx );
	}

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

		int measGrid;
		get( MEASUREMENT_GRID, measGrid );
		if( (MeasurementGrid)measGrid == ONLINE_GRID ) {
			for( uint i = 0; i < gridVariables.size(); i++ ) {
				declarations.addDeclaration( gridVariables[i],dataStruct );
			}
		}
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
	
	Matrix zeroM = zeros( 1,NX+NXA );
	initializeButcherTableau();
	initializeDDMatrix();
	initializeCoefficients();
	   
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();    

	ExportVariable Ah( "Ah_mat",  AA*=h );
	Matrix Bh( bb*=h, BT_TRUE );
//	ExportVariable DM( "DD", DD );
	
	ExportIndex i( "i" );
	ExportIndex j( "j" );
	ExportIndex k( "k" );
	ExportIndex run( "run" );
	ExportIndex run1( "run1" );
	ExportIndex tmp_index1("tmp_index1");
	ExportIndex tmp_index2("tmp_index2");
	ExportIndex tmp_index3("tmp_index3");
	ExportIndex tmp_index4("tmp_index4");
	ExportVariable tmp_meas("tmp_meas", 1, outputGrids.size(), INT, ACADO_LOCAL);
	
	code.addDeclaration( Ah );
	code.addLinebreak( 2 );

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !hasEquidistantGrid() ) {
		ExportVariable numStepsV( "numSteps", numSteps, STATIC_CONSTANT );
		code.addDeclaration( numStepsV );
		code.addLinebreak( 2 );
		integrate.addStatement( String( "int " ) << numInt.getName() << " = " << numStepsV.getName() << "[" << rk_index.getName() << "];\n" );
	}
	
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );
	bool SUB_GRID = ((MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID);

	std::vector<ExportVariable> polynVariables;
	std::vector<ExportVariable> polynDerVariables;
	std::vector<ExportVariable> numMeasVariables;
	for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
		if( (MeasurementGrid)measGrid != ONLINE_GRID ) {
			Matrix polynV = evaluatePolynomial( run5 );
			Matrix polynDerV = evaluateDerivedPolynomial( run5 );
			Vector measurements = divideMeasurements( run5 );

			ExportVariable polynVariable( (String)"polynOutput" << run5, polynV*=h );
			code.addDeclaration( polynVariable );
			polynVariable = ExportVariable( (String)"polynOutput" << run5, totalMeas[run5], numStages );
			polynVariables.push_back( polynVariable );

			if( NXA > 0 || NDX > 0 ) {
				ExportVariable polynDerVariable( (String)"polynDerOutput" << run5, polynDerV );
				code.addDeclaration( polynDerVariable );
				polynDerVariable = ExportVariable( (String)"polynDerOutput" << run5, totalMeas[run5], numStages );
				polynDerVariables.push_back( polynDerVariable );
			}

			ExportVariable numMeasVariable( (String)"numMeas" << run5, measurements, STATIC_CONSTANT );
			if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
				code.addDeclaration( numMeasVariable );
			}
			numMeasVariables.push_back( numMeasVariable );
		}
	}

	integrate.addIndex( i );
	integrate.addIndex( j );
	integrate.addIndex( k );
	integrate.addIndex( run );
	integrate.addIndex( run1 );
	integrate.addIndex( tmp_index1 );
	integrate.addIndex( tmp_index2 );
	integrate.addIndex( tmp_index3 );
	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !hasEquidistantGrid()) ) {
		integrate.addIndex( tmp_index4 );
	}
	std::vector<ExportIndex> numMeas;
	ExportVariable time_tmp( "time_tmp", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );
	if( CONTINUOUS_OUTPUT ) {
		for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
			ExportIndex numMeasTmp( (String)"numMeasTmp" << run5 );
			numMeas.push_back( numMeasTmp );
			integrate.addIndex( numMeas[run5] );
		}

		if( (MeasurementGrid)measGrid == ONLINE_GRID ) {
			integrate.addDeclaration( tmp_meas );
			integrate.addDeclaration( polynEvalVar );
			integrate.addDeclaration( time_tmp );
		}

		for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
			integrate.addStatement( numMeas[run5] == 0 );
		}
	}
	integrate.addStatement( rk_ttt == Matrix(grid.getFirstTime() + 1.0/grid.getNumIntervals()) );

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

	if( CONTINUOUS_OUTPUT && (MeasurementGrid)measGrid == ONLINE_GRID ) {
		for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
			loop->addStatement( tmp_index1 == numMeas[run5] );
			loop->addStatement( String("while( ") << tmp_index1.getName() << " < " << String(totalMeas[run5]) << " && " << gridVariables[run5].get(0,tmp_index1) << " <= " << rk_ttt.getFullName() << " ) {\n" );
			loop->addStatement( tmp_index1 == tmp_index1+1 );
			loop->addStatement( String("}\n") );
			loop->addStatement( String(tmp_meas.get( 0,run5 )) << " = " << tmp_index1.getName() << " - " << numMeas[run5].getName() << ";\n" );
		}
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
	ExportForLoop loop111( j,0,NX+NXA );
	loop111.addStatement( tmp_index1 == run1*(NX+NXA)+j );
	for( run3 = 0; run3 < numStages; run3++ ) { // differential states
		if( NDX == 0 ) {
			loop111.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,run3*NX,run3*NX+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,j*(NVARS),j*(NVARS)+NX ) );
			loop111.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) " );
			loop111.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,j+run3*NX,j+run3*NX+1 ) -= 1 );
		}
		else {
			loop111.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,run3*NX,run3*NX+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( 0,1,j*(NVARS),j*(NVARS)+NX ) );
			loop111.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) {\n" );
			loop111.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,run3*NX,run3*NX+NX ) += rk_diffsTemp.getSubMatrix( 0,1,j*(NVARS)+NX+NXA+NU,j*(NVARS)+NVARS ) );
			loop111.addStatement( String( "}\n" ) );
		}
	}
	if( NXA > 0 ) {
		for( run3 = 0; run3 < numStages; run3++ ) { // algebraic states
			loop111.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) {\n" );
			loop111.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,numStages*NX+run3*NXA,numStages*NX+run3*NXA+NXA ) == rk_diffsTemp.getSubMatrix( 0,1,j*(NVARS)+NX,j*(NVARS)+NX+NXA ) );
			loop111.addStatement( String( "}\n else {\n" ) );
			loop111.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,numStages*NX+run3*NXA,numStages*NX+run3*NXA+NXA ) == zeroM.getCols( 0,NXA-1 ) );
			loop111.addStatement( String( "}\n" ) );
		}
	}
	loop11.addStatement( loop111 );
	// matrix rk_b:
	if( NDX == 0 ) {
		loop11.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) - rk_rhsTemp.getCols( 0,NX ) );
	}
	else {
		loop11.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == zeroM.getCols( 0,NX-1 ) - rk_rhsTemp.getCols( 0,NX ) );
	}
	if( NXA > 0 ) loop11.addStatement( rk_b.getCols( run1*(NX+NXA)+NX,(run1+1)*(NX+NXA) ) == zeroM.getCols( 0,NXA-1 ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
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
		loop21.addStatement( rk_b.getCols( run1*(NX+NXA),run1*(NX+NXA)+NX ) == zeroM.getCols( 0,NX-1 ) - rk_rhsTemp.getCols( 0,NX ) );
	}
	if( NXA > 0 ) {
		loop21.addStatement( rk_b.getCols( run1*(NX+NXA)+NX,(run1+1)*(NX+NXA) ) == zeroM.getCols( 0,NXA-1 ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
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
		ExportStatementBlock *loop22;
		ExportForLoop tmpLoop2( i,0,totalMeas[run5]-1 );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop22 = &tmpLoop2;
			loop22->addStatement( rk_outH == polynVariables[run5].getRow( i ) );
			if( NXA > 0 || NDX > 0 ) loop22->addStatement( rk_out2 == polynDerVariables[run5].getRow( i ) );
		}
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop22 = loop;
			loop22->addStatement( String("for(i = 0; i < (int)") << numMeasVariables[run5].get(0,run) << "; i++) {\n" );
			loop22->addStatement( tmp_index2 == numMeas[run5]+i );
			loop22->addStatement( rk_outH == polynVariables[run5].getRow( tmp_index2 ) );
			if( NXA > 0 || NDX > 0 ) loop22->addStatement( rk_out2 == polynDerVariables[run5].getRow( tmp_index2 ) );
		}
		else { // ONLINE_GRID
			loop22 = loop;
			loop22->addStatement( String(tmp_index3.getName()) << " = " << tmp_meas.get( 0,run5 ) << ";\n" );
			loop22->addStatement( String("for(i = 0; i < (int)") << tmp_index3.getName() << "; i++) {\n" );
			loop22->addStatement( tmp_index2 == numMeas[run5]+i );

			uint scale = grid.getNumIntervals();
			double scale2 = 1.0/grid.getNumIntervals();
			loop22->addStatement( time_tmp.getName() << " = " << String(scale) << "*(" << gridVariables[run5].get(0,tmp_index2) << "-" << String(scale2) << "*" << run.getName() << ");\n" );

			evaluatePolynomial( *loop22, rk_outH, time_tmp, h );
			if( NXA > 0 || NDX > 0 ) evaluateDerivedPolynomial( *loop22, rk_outH, time_tmp );
		}

		Vector dependencyX, dependencyZ, dependencyDX;
		if( EXPORT_RHS || CRS_FORMAT ) {
			dependencyX = sumRow( outputDependencies[run5].getCols( 0,NX-1 ) );
			if( NXA > 0 ) dependencyZ = sumRow( outputDependencies[run5].getCols( NX,NX+NXA-1 ) );
			if( NDX > 0 ) dependencyDX = sumRow( outputDependencies[run5].getCols( NX+NXA+NU,NVARS-1 ) );
		}
		for( run3 = 0; run3 < NX; run3++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyX(run3) != 0 ) {
				loop22->addStatement( rk_xxx.getCols( run3,run3+1 ) == rk_eta.getCols( run3,run3+1 ) + rk_outH*rk_kkk.getCol( run3 ) );
			}
		}
		for( run3 = 0; run3 < NXA; run3++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyZ(run3) != 0 ) {
				loop22->addStatement( rk_xxx.getCols( NX+run3,NX+run3+1 ) == rk_out2*rk_kkk.getCol( NX+run3 ) );
			}
		}
		for( run3 = 0; run3 < NDX; run3++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyDX(run3) != 0 ) {
				loop22->addStatement( rk_xxx.getCols( inputDim-diffsDim+run3,inputDim-diffsDim+run3+1 ) == rk_out2*rk_kkk.getCol( run3 ) );
			}
		}
		loop22->addFunctionCall( getNameOUTPUT( run5 ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run5 );
		uint outputDim = numOutputs*(NX+NU+1);
		loop22->addStatement( tmp_index2 == numMeas[run5]*outputDim+i*(numOutputs*(NX+NU+1)) );
		for( run3 = 0; run3 < numOutputs; run3++ ) {
			loop22->addStatement( rk_outputs[run5].getCol( tmp_index2+run3 ) == rk_rhsOutputTemp.getCol( run3 ) );
		}
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop->addStatement( *loop22 );
		}
		else {
			loop22->addStatement( "}\n" );
		}
	}

	// solution calculated --> evaluate and save the necessary derivatives in rk_diffsTemp and update the matrix rk_A:
	ExportForLoop loop3( run1,0,numStages );
	loop3.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
	if( NXA > 0 ) loop3.addStatement( rk_xxx.getCols( NX,NX+NXA ) == rk_kkk.getSubMatrix( run1,run1+1,NX,NX+NXA ) );
	if( NDX > 0 ) loop3.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );

	loop3.addFunctionCall( getNameDiffsODE(), rk_xxx, rk_diffsTemp.getAddress(run1,0) );

	ExportForLoop loop31( j,0,NX+NXA );
	loop31.addStatement( tmp_index1 == run1*(NX+NXA)+j );
	for( run3 = 0; run3 < numStages; run3++ ) { // differential states
		if( NDX == 0 ) {
			loop31.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,run3*NX,run3*NX+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( run1,run1+1,j*(NVARS),j*(NVARS)+NX ) );
			loop31.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) " );
			loop31.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,j+run3*NX,j+run3*NX+1 ) -= 1 );
		}
		else {
			loop31.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,run3*NX,run3*NX+NX ) == Ah.getSubMatrix( run1,run1+1,run3,run3+1 )*rk_diffsTemp.getSubMatrix( run1,run1+1,j*(NVARS),j*(NVARS)+NX ) );
			loop31.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) {" );
			loop31.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,run3*NX,run3*NX+NX ) += rk_diffsTemp.getSubMatrix( run1,run1+1,j*(NVARS)+NX+NXA+NU,j*(NVARS)+NVARS ) );
			loop31.addStatement( String( "}\n" ) );
		}
	}
	if( NXA > 0 ) {
		for( run3 = 0; run3 < numStages; run3++ ) { // algebraic states
			loop31.addStatement( String( "if( " ) << run3 << " == " << run1.getName() << " ) {\n" );
			loop31.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,numStages*NX+run3*NXA,numStages*NX+run3*NXA+NXA ) == rk_diffsTemp.getSubMatrix( run1,run1+1,j*(NVARS)+NX,j*(NVARS)+NX+NXA ) );
			loop31.addStatement( String( "}\n else {\n" ) );
			loop31.addStatement( rk_A.getSubMatrix( tmp_index1,tmp_index1+1,numStages*NX+run3*NXA,numStages*NX+run3*NXA+NXA ) == zeroM.getCols( 0,NXA-1 ) );
			loop31.addStatement( String( "}\n" ) );
		}
	}
	loop3.addStatement( loop31 );
	loop->addStatement( loop3 );

	// update rk_eta:
	if( CONTINUOUS_OUTPUT ) loop->addStatement( rk_xPrev.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
	loop->addStatement( rk_eta.getCols( 0,NX ) += Bh*rk_kkk.getCols( 0,NX ) );
	Matrix tempCoefs( evaluateDerivedPolynomial( 1.0 ), BT_TRUE );
	if( NXA > 0) loop->addStatement( rk_eta.getCols( NX,NX+NXA ) == tempCoefs*rk_kkk.getCols( NX,NX+NXA ) );

	// generate continuous output (only the last grid point):
	if( CONTINUOUS_OUTPUT && SUB_GRID ) {
		loop->addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
		if( NDX > 0 ) loop->addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == tempCoefs*rk_kkk.getCols( 0,NX ) );

		for( run5 = 0; run5 < rk_outputs.size(); run5++ ) {
			run2 = totalMeas[run5]-1;
			loop->addFunctionCall( getNameOUTPUT( run5 ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
			uint numOutputs = getDimOUTPUT( run5 );
			uint outputDim = numOutputs*(NX+NU+1);
			for( run3 = 0; run3 < numOutputs; run3++ ) {
				loop->addStatement( rk_outputs[run5].getCol( numMeas[run5]*outputDim+run2*numOutputs*(NX+NU+1)+run3 ) == rk_rhsOutputTemp.getCol( run3 ) );
			}
		}
	}

	// derivatives wrt the states (IFT):
	ExportForLoop loop4( run1,0,NX );
	ExportForLoop loop40( i,0,numStages );
	for( run3 = 0; run3 < NX+NXA; run3++ ) {
		loop40.addStatement( rk_b.getCol( i*(NX+NXA)+run3 ) == zeroM.getCol( 0 ) - rk_diffsTemp.getSubMatrix( i,i+1,run1+run3*(NVARS),run1+run3*(NVARS)+1 ) );
	}
	loop4.addStatement( loop40 );
	loop4.addStatement( String( "if( 0 == " ) << run1.getName() << " ) {\n" );	// factorization of the new matrix rk_A not yet calculated!
	loop4.addFunctionCall( solver->getNameSolveFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loop4.addStatement( String( "}\n else {\n" ) );
	loop4.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );
	loop4.addStatement( String( "}\n" ) );

	// generate sensitivities wrt states for continuous output (except for the last grid point):
	for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
		ExportStatementBlock *loop41;
		ExportForLoop tmpLoop3( i,0,totalMeas[run2]-1 );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop41 = &tmpLoop3;
			loop41->addStatement( rk_outH == polynVariables[run2].getRow( i ) );
			if( NXA > 0 || NDX > 0 ) loop41->addStatement( rk_out2 == polynDerVariables[run2].getRow( i ) );
		}
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop41 = &loop4;
			loop41->addStatement( String("for(i = 0; i < (int)") << numMeasVariables[run2].get(0,run) << "; i++) {\n" );
			loop41->addStatement( tmp_index2 == numMeas[run2]+i );
			loop41->addStatement( rk_outH == polynVariables[run2].getRow( tmp_index2 ) );
			if( NXA > 0 || NDX > 0 ) loop41->addStatement( rk_out2 == polynDerVariables[run2].getRow( tmp_index2 ) );
		}
		else { // ONLINE_GRID
			loop41 = &loop4;
			loop41->addStatement( String(tmp_index3.getName()) << " = " << tmp_meas.get( 0,run2 ) << ";\n" );
			loop41->addStatement( String("for(i = 0; i < (int)") << tmp_index3.getName() << "; i++) {\n" );
			loop41->addStatement( tmp_index2 == numMeas[run2]+i );

			uint scale = grid.getNumIntervals();
			double scale2 = 1.0/grid.getNumIntervals();
			loop41->addStatement( time_tmp.getName() << " = " << String(scale) << "*(" << gridVariables[run2].get(0,tmp_index2) << "-" << String(scale2) << "*" << run.getName() << ");\n" );

			evaluatePolynomial( *loop41, rk_outH, time_tmp, h );
			if( NXA > 0 || NDX > 0 ) evaluateDerivedPolynomial( *loop41, rk_outH, time_tmp );
		}

		Vector dependencyX, dependencyZ, dependencyDX;
		if( EXPORT_RHS || CRS_FORMAT ) {
			dependencyX = sumRow( outputDependencies[run2].getCols( 0,NX-1 ) );
			if( NXA > 0 ) dependencyZ = sumRow( outputDependencies[run2].getCols( NX,NX+NXA-1 ) );
			if( NDX > 0 ) dependencyDX = sumRow( outputDependencies[run2].getCols( NX+NXA+NU,NVARS-1 ) );
		}
		for( run4 = 0; run4 < NX; run4++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyX(run4) != 0 ) {
				loop41->addStatement( String(rk_rhsTemp.get( 0,run4 )) << " = (" << run4 << " == " << run1.getName() << ");\n" );
				for( run5 = 0; run5 < numStages; run5++ ) {
					loop41->addStatement( rk_rhsTemp.getCol( run4 ) += rk_outH.getCol( run5 )*rk_sol.getCol( run5*NX+run4 ) );
				}
				loop41->addStatement( rk_xxx.getCol( run4 ) == rk_xPrev.getCol( run4 ) + rk_outH*rk_kkk.getCol( run4 ) );
			}
		}
		for( run4 = 0; run4 < NXA; run4++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyZ(run4) != 0 ) {
				loop41->addStatement( rk_rhsTemp.getCol( NX+run4 ) == rk_out2.getCol( 0 )*rk_sol.getCol( numStages*NX+run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop41->addStatement( rk_rhsTemp.getCol( NX+run4 ) += rk_out2.getCol( run5 )*rk_sol.getCol( numStages*NX+run5*NXA+run4 ) );
				}
				loop41->addStatement( rk_xxx.getCol( NX+run4 ) == rk_out2*rk_kkk.getCol( NX+run4 ) );
			}
		}
		for( run4 = 0; run4 < NDX; run4++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyDX(run4) != 0 ) {
				loop41->addStatement( rk_rhsTemp.getCol( NX+NXA+run4 ) == rk_out2.getCol( 0 )*rk_sol.getCol( run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop41->addStatement( rk_rhsTemp.getCol( NX+NXA+run4 ) += rk_out2.getCol( run5 )*rk_sol.getCol( run5*NX+run4 ) );
				}
				loop41->addStatement( rk_xxx.getCol( inputDim-diffsDim+run4 ) == rk_out2*rk_kkk.getCol( run4 ) );
			}
		}

		loop41->addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run2 );
		uint outputDim = numOutputs*(NX+NU+1);
		loop41->addStatement( tmp_index1 == numMeas[run2]*outputDim+i*(numOutputs*(NX+NU+1)) );
		loop41->addStatement( tmp_index2 == tmp_index1+run1 );
		for( run4 = 0; run4 < numOutputs; run4++ ) {
			if( EXPORT_RHS || CRS_FORMAT ) {
				dependencyX = (outputDependencies[run2].getCols( 0,NX-1 )).getRow( run4 );
				if( NXA > 0 ) dependencyZ = (outputDependencies[run2].getCols( NX,NX+NXA-1 )).getRow( run4 );
				if( NDX > 0 ) dependencyDX = (outputDependencies[run2].getCols( NX+NXA+NU,NVARS-1 )).getRow( run4 );
			}

			loop41->addStatement( rk_outputs[run2].getCol( tmp_index2+numOutputs+run4*NX ) == 0 );
			for( run5 = 0; run5 < NX; run5++ ) {
				if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyX(run5) != 0 ) {
					loop41->addStatement( rk_outputs[run2].getCol( tmp_index2+numOutputs+run4*NX ) += rk_diffsOutputTemp.getCol( run4*NVARS+run5 )*rk_rhsTemp.getCol( run5 ) );
				}
			}
			for( run5 = 0; run5 < NXA; run5++ ) {
				if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyZ(run5) != 0 ) {
					loop41->addStatement( rk_outputs[run2].getCol( tmp_index2+numOutputs+run4*NX ) += rk_diffsOutputTemp.getCol( run4*NVARS+NX+run5 )*rk_rhsTemp.getCol( NX+run5 ) );
				}
			}
			for( run5 = 0; run5 < NDX; run5++ ) {
				if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyDX(run5) != 0 ) {
					loop41->addStatement( rk_outputs[run2].getCol( tmp_index2+numOutputs+run4*NX ) += rk_diffsOutputTemp.getCol( run4*NVARS+NX+NXA+NU+run5 )*rk_rhsTemp.getCol( NX+NXA+run5 ) );
				}
			}
		}
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop4.addStatement( *loop41 );
		}
		else {
			loop41->addStatement( "}\n" );
		}
	}

	if( NDX > 0 ) {
		ExportForLoop loop42( i,0,NDX );
		loop42.addStatement( rk_rhsTemp.getCol( i ) == tempCoefs.getCol( 0 )*rk_sol.getCol( i ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop42.addStatement( rk_rhsTemp.getCol( i ) += tempCoefs.getCol( run3 )*rk_sol.getCol( i+run3*NX ) );
		}
		loop4.addStatement( loop42 );
	}

	// update rk_diffsNew with the new sensitivities:
	ExportForLoop loop43( i,0,NX );
	loop43.addStatement( String(rk_diffsNew.get( i,run1 )) << " = (" << i.getName() << " == " << run1.getName() << ");\n" );
	for (run3 = 0; run3 < numStages; run3++ ) {
		loop43.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1,run1+1 ) += Bh.getCol( run3 )*rk_sol.getCol( i+run3*NX ) );
	}
	loop4.addStatement( loop43 );
	if( NXA > 0 ) {
		ExportForLoop loop44( i,0,NXA );
		loop44.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1,run1+1 ) == tempCoefs.getCol( 0 )*rk_sol.getCol( i+numStages*NX ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop44.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1,run1+1 ) += tempCoefs.getCol( run3 )*rk_sol.getCol( i+numStages*NX+run3*NXA ) );
		}
		loop4.addStatement( loop44 );
	}

	// generate sensitivities wrt states for continuous output (only the last grid point):
	if( CONTINUOUS_OUTPUT && SUB_GRID ) {
		loop4.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
		if( NDX > 0 ) loop4.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == tempCoefs*rk_kkk.getCols( 0,NX ) );

		for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
			run3 = totalMeas[run2]-1;
			loop4.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
			uint numOutputs = getDimOUTPUT( run2 );
			uint outputDim = numOutputs*(NX+NU+1);
			loop4.addStatement( tmp_index2 == numMeas[run2]*outputDim+run1 );
			for( run4 = 0; run4 < numOutputs; run4++ ) {
				Vector dependencyX, dependencyZ, dependencyDX;
				if( EXPORT_RHS || CRS_FORMAT ) {
					dependencyX = (outputDependencies[run2].getCols( 0,NX-1 )).getRow( run4 );
					if( NXA > 0 ) dependencyZ = (outputDependencies[run2].getCols( NX,NX+NXA-1 )).getRow( run4 );
					if( NDX > 0 ) dependencyDX = (outputDependencies[run2].getCols( NX+NXA+NU,NVARS-1 )).getRow( run4 );
				}
				loop4.addStatement( rk_outputs[run2].getCol( tmp_index2+run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX ) == 0 );
				for( run5 = 0; run5 < NX; run5++ ) {
					if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyX(run5) != 0 ) {
						loop4.addStatement( rk_outputs[run2].getCol( tmp_index2+run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX ) += rk_diffsOutputTemp.getCol( run4*NVARS+run5 )*rk_diffsNew.getSubMatrix( run5,run5+1,run1,run1+1 ) );
					}
				}
				for( run5 = 0; run5 < NXA; run5++ ) {
					if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyZ(run5) != 0 ) {
						loop4.addStatement( rk_outputs[run2].getCol( tmp_index2+run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX ) += rk_diffsOutputTemp.getCol( run4*NVARS+NX+run5 )*rk_diffsNew.getSubMatrix( NX+run5,NX+run5+1,run1,run1+1 ) );
					}
				}
				for( run5 = 0; run5 < NDX; run5++ ) {
					if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyDX(run5) != 0 ) {
						loop4.addStatement( rk_outputs[run2].getCol( tmp_index2+run3*numOutputs*(NX+NU+1)+numOutputs+run4*NX ) += rk_diffsOutputTemp.getCol( run4*NVARS+NX+NXA+NU+run5 )*rk_rhsTemp.getCol( run5 ) );
					}
				}
			}
		}
	}
	loop->addStatement( loop4 );


	// derivatives wrt the control inputs (IFT):
	if( NU > 0 ) {
	ExportForLoop loop5( run1,0,NU );
	ExportForLoop loop50( i,0,numStages );
	for( run3 = 0; run3 < NX+NXA; run3++ ) {
		loop50.addStatement( rk_b.getCol( i*(NX+NXA)+run3 ) == zeroM.getCol( 0 ) - rk_diffsTemp.getSubMatrix( i,i+1,run1+run3*(NVARS)+NX+NXA,run1+run3*(NVARS)+NX+NXA+1 ) );
	}
	loop5.addStatement( loop50 );
	loop5.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_sol.getAddress(0,0) );

	// generate sensitivities wrt controls for continuous output (except for the last grid point):
	for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
		ExportStatementBlock *loop51;
		ExportForLoop tmpLoop4( i,0,totalMeas[run2]-1 );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop51 = &tmpLoop4;
			loop51->addStatement( rk_outH == polynVariables[run2].getRow( i ) );
			if( NXA > 0 || NDX > 0 ) loop51->addStatement( rk_out2 == polynDerVariables[run2].getRow( i ) );
		}
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop51 = &loop5;
			loop51->addStatement( String("for(i = 0; i < (int)") << numMeasVariables[run2].get(0,run) << "; i++) {\n" );
			loop51->addStatement( tmp_index2 == numMeas[run2]+i );
			loop51->addStatement( rk_outH == polynVariables[run2].getRow( tmp_index2 ) );
			if( NXA > 0 || NDX > 0 ) loop51->addStatement( rk_out2 == polynDerVariables[run2].getRow( tmp_index2 ) );
		}
		else { // ONLINE_GRID
			loop51 = &loop5;
			loop51->addStatement( String(tmp_index3.getName()) << " = " << tmp_meas.get( 0,run2 ) << ";\n" );
			loop51->addStatement( String("for(i = 0; i < (int)") << tmp_index3.getName() << "; i++) {\n" );
			loop51->addStatement( tmp_index2 == numMeas[run2]+i );

			uint scale = grid.getNumIntervals();
			double scale2 = 1.0/grid.getNumIntervals();
			loop51->addStatement( time_tmp.getName() << " = " << String(scale) << "*(" << gridVariables[run2].get(0,tmp_index2) << "-" << String(scale2) << "*" << run.getName() << ");\n" );

			evaluatePolynomial( *loop51, rk_outH, time_tmp, h );
			if( NXA > 0 || NDX > 0 ) evaluateDerivedPolynomial( *loop51, rk_outH, time_tmp );
		}

		Vector dependencyX, dependencyZ, dependencyDX;
		if( EXPORT_RHS || CRS_FORMAT ) {
			dependencyX = sumRow( outputDependencies[run2].getCols( 0,NX-1 ) );
			if( NXA > 0 ) dependencyZ = sumRow( outputDependencies[run2].getCols( NX,NX+NXA-1 ) );
			if( NDX > 0 ) dependencyDX = sumRow( outputDependencies[run2].getCols( NX+NXA+NU,NVARS-1 ) );
		}
		for( run4 = 0; run4 < NX; run4++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyX(run4) != 0 ) {
				loop51->addStatement( rk_rhsTemp.getCol( run4 ) == rk_outH.getCol( 0 )*rk_sol.getCol( run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop51->addStatement( rk_rhsTemp.getCol( run4 ) += rk_outH.getCol( run5 )*rk_sol.getCol( run5*NX+run4 ) );
				}
				loop51->addStatement( rk_xxx.getCol( run4 ) == rk_xPrev.getCol( run4 ) + rk_outH*rk_kkk.getCol( run4 ) );
			}
		}
		for( run4 = 0; run4 < NXA; run4++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyZ(run4) != 0 ) {
				loop51->addStatement( rk_rhsTemp.getCol( NX+run4 ) == rk_out2.getCol( 0 )*rk_sol.getCol( numStages*NX+run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop51->addStatement( rk_rhsTemp.getCol( NX+run4 ) += rk_out2.getCol( run5 )*rk_sol.getCol( numStages*NX+run5*NXA+run4 ) );
				}
				loop51->addStatement( rk_xxx.getCol( NX+run4 ) == rk_out2*rk_kkk.getCol( NX+run4 ) );
			}
		}
		for( run4 = 0; run4 < NDX; run4++ ) {
			if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyDX(run4) != 0 ) {
				loop51->addStatement( rk_rhsTemp.getCol( NX+NXA+run4 ) == rk_out2.getCol( 0 )*rk_sol.getCol( run4 ) );
				for( run5 = 1; run5 < numStages; run5++ ) {
					loop51->addStatement( rk_rhsTemp.getCol( NX+NXA+run4 ) += rk_out2.getCol( run5 )*rk_sol.getCol( run5*NX+run4 ) );
				}
				loop51->addStatement( rk_xxx.getCol( inputDim-diffsDim+run4 ) == rk_out2*rk_kkk.getCol( run4 ) );
			}
		}

		loop51->addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( run2 );
		uint outputDim = numOutputs*(NX+NU+1);
		loop51->addStatement( tmp_index1 == numMeas[run2]*outputDim+i*(numOutputs*(NX+NU+1)) );
		loop51->addStatement( tmp_index2 == tmp_index1+run1 );
		for( run4 = 0; run4 < numOutputs; run4++ ) {
			if( EXPORT_RHS || CRS_FORMAT ) {
				dependencyX = (outputDependencies[run2].getCols( 0,NX-1 )).getRow( run4 );
				if( NXA > 0 ) dependencyZ = (outputDependencies[run2].getCols( NX,NX+NXA-1 )).getRow( run4 );
				if( NDX > 0 ) dependencyDX = (outputDependencies[run2].getCols( NX+NXA+NU,NVARS-1 )).getRow( run4 );
			}
			loop51->addStatement( rk_outputs[run2].getCol( tmp_index2+numOutputs*(1+NX)+run4*NU ) == rk_diffsOutputTemp.getCol( run1+run4*NVARS+NX+NXA ) );
			for( run5 = 0; run5 < NX; run5++ ) {
				if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyX(run5) != 0 ) {
					loop51->addStatement( rk_outputs[run2].getCol( tmp_index2+numOutputs*(1+NX)+run4*NU ) += rk_diffsOutputTemp.getCol( run4*NVARS+run5 )*rk_rhsTemp.getCol( run5 ) );
				}
			}
			for( run5 = 0; run5 < NXA; run5++ ) {
				if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyZ(run5) != 0 ) {
					loop51->addStatement( rk_outputs[run2].getCol( tmp_index2+numOutputs*(1+NX)+run4*NU ) += rk_diffsOutputTemp.getCol( run4*NVARS+NX+run5 )*rk_rhsTemp.getCol( NX+run5 ) );
				}
			}
			for( run5 = 0; run5 < NDX; run5++ ) {
				if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyDX(run5) != 0 ) {
					loop51->addStatement( rk_outputs[run2].getCol( tmp_index2+numOutputs*(1+NX)+run4*NU ) += rk_diffsOutputTemp.getCol( run4*NVARS+NX+NXA+NU+run5 )*rk_rhsTemp.getCol( NX+NXA+run5 ) );
				}
			}
		}
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop5.addStatement( *loop51 );
		}
		else {
			loop51->addStatement( "}\n" );
		}
	}

	if( NDX > 0 ) {
		ExportForLoop loop52( i,0,NDX );
		loop52.addStatement( rk_rhsTemp.getCol( i ) == tempCoefs.getCol( 0 )*rk_sol.getCol( i ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop52.addStatement( rk_rhsTemp.getCol( i ) += tempCoefs.getCol( run3 )*rk_sol.getCol( i+run3*NX ) );
		}
		loop5.addStatement( loop52 );
	}

	// update rk_diffsNew with the new sensitivities:
	ExportForLoop loop53( i,0,NX );
	loop53.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1+NX,run1+NX+1 ) == Bh.getCol( 0 )*rk_sol.getCol( i ) );
	for (run3 = 1; run3 < numStages; run3++ ) {
		loop53.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1+NX,run1+NX+1 ) += Bh.getCol( run3 )*rk_sol.getCol( i+run3*NX ) );
	}
	loop5.addStatement( loop53 );
	if( NXA > 0 ) {
		ExportForLoop loop54( i,0,NXA );
		loop54.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1+NX,run1+NX+1 ) == tempCoefs.getCol( 0 )*rk_sol.getCol( i+numStages*NX ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop54.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1+NX,run1+NX+1 ) += tempCoefs.getCol( run3 )*rk_sol.getCol( i+numStages*NX+run3*NXA ) );
		}
		loop5.addStatement( loop54 );
	}

	// generate sensitivities wrt controls for continuous output (only the last grid point):
	if( CONTINUOUS_OUTPUT && SUB_GRID ) {
		loop5.addStatement( rk_xxx.getCols( 0,NX+NXA ) == rk_eta.getCols( 0,NX+NXA ) );
		if( NDX > 0 ) loop5.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == tempCoefs*rk_kkk.getCols( 0,NX ) );

		for( run2 = 0; run2 < rk_outputs.size(); run2++ ) {
			run3 = totalMeas[run2]-1;
			loop5.addFunctionCall( getNameDiffsOUTPUT( run2 ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
			uint numOutputs = getDimOUTPUT( run2 );
			uint outputDim = numOutputs*(NX+NU+1);
			loop5.addStatement( tmp_index2 == numMeas[run2]*outputDim+run1 );
			for( run4 = 0; run4 < numOutputs; run4++ ) {
				Vector dependencyX, dependencyZ, dependencyDX;
				if( EXPORT_RHS || CRS_FORMAT ) {
					dependencyX = (outputDependencies[run2].getCols( 0,NX-1 )).getRow( run4 );
					if( NXA > 0 ) dependencyZ = (outputDependencies[run2].getCols( NX,NX+NXA-1 )).getRow( run4 );
					if( NDX > 0 ) dependencyDX = (outputDependencies[run2].getCols( NX+NXA+NU,NVARS-1 )).getRow( run4 );
				}

				loop5.addStatement( rk_outputs[run2].getCol( tmp_index2+run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU ) == rk_diffsOutputTemp.getCol( run1+run4*NVARS+NX+NXA ) );
				for( run5 = 0; run5 < NX; run5++ ) {
					if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyX(run5) != 0 ) {
						loop5.addStatement( rk_outputs[run2].getCol( tmp_index2+run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU ) += rk_diffsOutputTemp.getCol( run4*NVARS+run5 )*rk_diffsNew.getSubMatrix( run5,run5+1,run1+NX,run1+NX+1 ) );
					}
				}
				for( run5 = 0; run5 < NXA; run5++ ) {
					if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyZ(run5) != 0 ) {
						loop5.addStatement( rk_outputs[run2].getCol( tmp_index2+run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU ) += rk_diffsOutputTemp.getCol( run4*NVARS+NX+run5 )*rk_diffsNew.getSubMatrix( NX+run5,NX+run5+1,run1+NX,run1+NX+1 ) );
					}
				}
				for( run5 = 0; run5 < NDX; run5++ ) {
					if( (!EXPORT_RHS && !CRS_FORMAT) || (int)dependencyDX(run5) != 0 ) {
						loop5.addStatement( rk_outputs[run2].getCol( tmp_index2+run3*numOutputs*(NX+NU+1)+numOutputs*(1+NX)+run4*NU ) += rk_diffsOutputTemp.getCol( run4*NVARS+NX+NXA+NU+run5 )*rk_rhsTemp.getCol( run5 ) );
					}
				}
			}
		}
	}
	loop->addStatement( loop5 );
	}

	// Computation of the sensitivities using the chain rule:
	ExportForLoop loop01( i,0,NX+NXA );
	ExportForLoop loop02( j,0,NX );
	loop02.addStatement( tmp_index2 == j+i*NX );
	loop02.addStatement( rk_eta.getCol( tmp_index2+NX+NXA ) == rk_diffsNew.getSubMatrix( i,i+1,0,1 )*rk_diffsPrev.getSubMatrix( 0,1,j,j+1 ) );
	ExportForLoop loop021( k,1,NX );
	loop021.addStatement( rk_eta.getCol( tmp_index2+NX+NXA ) += rk_diffsNew.getSubMatrix( i,i+1,k,k+1 )*rk_diffsPrev.getSubMatrix( k,k+1,j,j+1 ) );
	loop02.addStatement( loop021 );
	loop01.addStatement( loop02 );
	loop->addStatement( loop01 );

	if( NU > 0 ) {
		ExportForLoop loop03( i,0,NX+NXA );
		ExportForLoop loop04( j,0,NU );
		loop04.addStatement( tmp_index2 == j+i*NU );
		loop04.addStatement( rk_eta.getCol( tmp_index2+(NX+NXA)*(1+NX) ) == rk_diffsNew.getSubMatrix( i,i+1,NX+j,NX+j+1 ) + rk_diffsNew.getSubMatrix( i,i+1,0,1 )*rk_diffsPrev.getSubMatrix( 0,1,NX+j,NX+j+1 ) );
		ExportForLoop loop041( k,1,NX );
		loop041.addStatement( rk_eta.getCol( tmp_index2+(NX+NXA)*(1+NX) ) += rk_diffsNew.getSubMatrix( i,i+1,k,k+1 )*rk_diffsPrev.getSubMatrix( k,k+1,NX+j,NX+j+1 ) );
		loop04.addStatement( loop041 );
		loop03.addStatement( loop04 );
		loop->addStatement( loop03 );
	}

	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !hasEquidistantGrid()) ) {
		loop->addStatement( String( "if( run > 0 ) {\n" ) );

		// chain rule for the sensitivities of the continuous output:
		for( run5 = 0; run5 < rk_outputs.size(); run5++ ) {
			ExportStatementBlock *loop05;
			ExportForLoop tmpLoop5( run1,0,totalMeas[run5] );
			if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) loop05 = &tmpLoop5;
			else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
				loop05 = loop;
				loop05->addStatement( String("for(run1 = 0; run1 < (int)") << numMeasVariables[run5].get(0,run) << "; run1++) {\n" );
			}
			else { // ONLINE_GRID
				loop05 = loop;
				loop05->addStatement( String("for(run1 = 0; run1 < (int)") << tmp_meas.get(0,run5) << "; run1++) {\n" );
			}

			uint numOutputs = getDimOUTPUT( run5 );
			uint outputDim = numOutputs*(NX+NU+1);
			loop05->addStatement( tmp_index1 == numMeas[run5]*outputDim+run1*(numOutputs*(NX+NU+1)) );
			ExportForLoop loop051( k,0,numOutputs*(NX+NU) );
			loop051.addStatement( tmp_index2 == tmp_index1+k );
			loop051.addStatement( rk_diffsOutputTemp.getCol( k ) == rk_outputs[run5].getCol( tmp_index2+numOutputs ) );
			loop05->addStatement( loop051 );

			loop05->addStatement( tmp_index1 == numMeas[run5]*outputDim+run1*(numOutputs*(NX+NU+1)) );
			ExportForLoop loop06( i,0,numOutputs );
			loop06.addStatement( tmp_index2 == tmp_index1+i*NX );
			ExportForLoop loop07( j,0,NX );
			loop07.addStatement( tmp_index3 == tmp_index2+j );
			loop07.addStatement( rk_outputs[run5].getCol( tmp_index3+numOutputs ) == rk_diffsOutputTemp.getCol( i*NX )*rk_diffsPrev.getSubMatrix( 0,1,j,j+1 ) );
			ExportForLoop loop071( k,1,NX );
			loop071.addStatement( tmp_index4 == i*NX+k );
			loop071.addStatement( rk_outputs[run5].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev.getSubMatrix( k,k+1,j,j+1 ) );
			loop07.addStatement( loop071 );
			loop06.addStatement( loop07 );
			loop05->addStatement( loop06 );

			if( NU > 0 ) {
				ExportForLoop loop08( i,0,numOutputs );
				loop08.addStatement( tmp_index2 == tmp_index1+i*NU );
				ExportForLoop loop09( j,0,NU );
				loop09.addStatement( tmp_index3 == tmp_index2+j );
				loop09.addStatement( tmp_index4 == i*NU+j );
				loop09.addStatement( rk_outputs[run5].getCol( tmp_index3+numOutputs*(1+NX) ) == rk_diffsOutputTemp.getCol( tmp_index4+numOutputs*NX ) + rk_diffsOutputTemp.getCol( i*NX )*rk_diffsPrev.getSubMatrix( 0,1,NX+j,NX+j+1 ) );
				ExportForLoop loop091( k,1,NX );
				loop091.addStatement( tmp_index4 == i*NX+k );
				loop091.addStatement( rk_outputs[run5].getCol( tmp_index3+numOutputs*(1+NX) ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev.getSubMatrix( k,k+1,NX+j,NX+j+1 ) );
				loop09.addStatement( loop091 );
				loop08.addStatement( loop09 );
				loop05->addStatement( loop08 );
			}
			if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
				loop->addStatement( *loop05 );
			}
			else {
				loop05->addStatement( "}\n" );
			}
		}
		loop->addStatement( String( "}\n" ) );
	}

	loop->addStatement( rk_ttt += 1.0/grid.getNumIntervals() );
	loop->addStatement( String( rk_num.get(0,0) ) << " += 1;\n" );

	for( run5 = 0; run5 < rk_outputs.size(); run5++ ) {
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) loop->addStatement( numMeas[run5] == numMeas[run5]+totalMeas[run5] );
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop->addStatement( numMeas[run5].getName() << " += " << numMeasVariables[run5].get(0,run) << ";\n" );
		}
		else { // ONLINE_GRID
			loop->addStatement( numMeas[run5].getName() << " += " << tmp_meas.get(0,run5) << ";\n" );
		}
	}

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


Matrix ImplicitRungeKuttaExport::evaluatePolynomial( uint index )
{
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	Matrix polynV(totalMeas[index],numStages);
	double scale = 1.0/outputGrids[index].getTime( totalMeas[index] );
	double scale2 = 1.0/grid.getNumIntervals();
	for( uint i = 1; i <= totalMeas[index]; i++ ) {
		double time = scale*outputGrids[index].getTime(i);
		if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			uint interv = getIntegrationInterval( time );
			time = (time-interv*scale2)/scale2;
		}
		polynV.setRow( i-1, evaluatePolynomial( time ) );
	}

	return polynV;
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


returnValue ImplicitRungeKuttaExport::evaluatePolynomial( ExportStatementBlock& block, const ExportVariable& variable, const ExportVariable& gridVariable, double h )
{
	uint i, j;

	block.addStatement( polynEvalVar == gridVariable );
	for( i = 0; i < numStages; i++ ) {
		for( j = 0; j < numStages; j++ ) {
			if( i == 0 ) {
				block.addStatement( variable.getCol( j ) == polynEvalVar*coeffs( numStages-1-i,j ) );
			}
			else {
				block.addStatement( variable.getCol( j ) += polynEvalVar*coeffs( numStages-1-i,j ) );
			}
		}
		if( i < (numStages-1) ) block.addStatement( polynEvalVar == polynEvalVar*gridVariable );
	}
	for( j = 0; j < numStages; j++ ) {
		block.addStatement( (String)variable.getFullName() << "[" << j << "] *= " << h << ";\n" );
	}

    return SUCCESSFUL_RETURN;
}


Matrix ImplicitRungeKuttaExport::evaluateDerivedPolynomial( uint index )
{
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	Matrix polynDerV(totalMeas[index],numStages);
	double scale = 1.0/outputGrids[index].getTime( totalMeas[index] );
	double scale2 = 1.0/grid.getNumIntervals();
	for( uint i = 1; i <= totalMeas[index]; i++ ) {
		double time = scale*outputGrids[index].getTime(i);
		if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			uint interv = getIntegrationInterval( time );
			time = (time-interv*scale2)/scale2;
		}
		polynDerV.setRow( i-1, evaluateDerivedPolynomial( time ) );
	}

	return polynDerV;
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


returnValue ImplicitRungeKuttaExport::evaluateDerivedPolynomial( ExportStatementBlock& block, const ExportVariable& variable, const ExportVariable& gridVariable )
{
	uint i, j;
	
	// construct the Lagrange interpolating polynomials:
	for( i = 0; i < numStages; i++ ) {
		for( j = 0; j < numStages; j++ ) {
			if( (i == 0 && j == 1) || (i != j && j == 0) ) {
				block.addStatement( (String)variable.getFullName() << "[" << i << "] = (" << gridVariable.getName() << " - " << cc(j) << ")*" << 1/(cc(i)-cc(j)) << ";\n" );
			}
			else if( i != j ) {
				block.addStatement( (String)variable.getFullName() << "[" << i << "] *= (" << gridVariable.getName() << " - " << cc(j) << ")*" << 1/(cc(i)-cc(j)) << ";\n" );
			}
		}
	}

    return SUCCESSFUL_RETURN;
}


Vector ImplicitRungeKuttaExport::divideMeasurements( uint index )
{
	Vector meas = zeros(1,grid.getNumIntervals());

	for( uint i = 1; i <= outputGrids[index].getNumIntervals(); i++ ) {
		uint interv = getIntegrationInterval( outputGrids[index].getTime(i) );
		meas(interv) = meas(interv)+1;
	}

	return meas;
}


returnValue ImplicitRungeKuttaExport::setup( )
{
	if( CONTINUOUS_OUTPUT && !hasEquidistantGrid() ) return ACADOERROR( RET_INVALID_OPTION );

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
	
	NVARS = NX+NXA+NU+NDX;
	diffsDim = (NX+NXA)*(NX+NU);
	inputDim = (NX+NXA)*(NX+NU+1) + NU + NP;
	
	rk_num = ExportVariable( "rk_num", 1, 1, INT, ACADO_VARIABLES, BT_TRUE );
	rk_ttt = ExportVariable( "rk_ttt", 1, 1, REAL, ACADO_WORKSPACE, BT_TRUE );
	rk_xxx = ExportVariable( "rk_xxx", 1, inputDim-diffsDim+NDX, REAL, ACADO_WORKSPACE );
	if( CONTINUOUS_OUTPUT ) rk_xPrev = ExportVariable( "rk_xPrev", 1, inputDim-diffsDim, REAL, ACADO_WORKSPACE );
	rk_kkk = ExportVariable( "rk_kkk", numStages, NX+NXA, REAL, ACADO_WORKSPACE );
	rk_sol = ExportVariable( "rk_sol", 1, numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_A = ExportVariable( "rk_A", numStages*(NX+NXA), numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_b = ExportVariable( "rk_b", 1, numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_rhsTemp = ExportVariable( "rk_rhsTemp", 1, NX+NXA+NDX, REAL, ACADO_WORKSPACE );
	rk_diffsTemp = ExportVariable( "rk_diffsTemp", numStages, (NX+NXA)*(NVARS), REAL, ACADO_WORKSPACE );
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

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	uint i;
	uint maxOutputs = 0;
	rk_outputs.clear();
	OUTPUTS.clear();
	diffs_OUTPUTS.clear();
	for( i = 0; i < outputGrids.size(); i++ ) {
		uint numOutputs = outputExpressions_[i].getDim();
		uint outputDim = outputGrids[i].getNumIntervals( )*numOutputs*(NX+NU+1);
		
		if( numOutputs > maxOutputs ) maxOutputs = numOutputs;
		
		DifferentialEquation f_Output;
		f_Output << outputExpressions_[i];
	
		DifferentialEquation g_Output;
		for( uint j = 0; j < outputExpressions_[i].getDim(); j++ ) {
			g_Output << forwardDerivative( outputExpressions_[i](j), x );
			g_Output << forwardDerivative( outputExpressions_[i](j), z );
			g_Output << forwardDerivative( outputExpressions_[i](j), u );
			g_Output << forwardDerivative( outputExpressions_[i](j), dx );
		}
	
		ExportODEfunction OUTPUT, diffs_OUTPUT;
		val = val & OUTPUT.init( f_Output,String("acado_output")<<String(i)<<"_rhs",NX,NXA,NU ) & diffs_OUTPUT.init( g_Output,String("acado_output")<<String(i)<<"_diffs",NX,NXA,NU );
		
		ExportVariable rk_output( String("rk_output")<<String(i), 1, outputDim, REAL );
		rk_outputs.push_back( rk_output );
		
		OUTPUTS.push_back( OUTPUT );
		diffs_OUTPUTS.push_back( diffs_OUTPUT );

		Matrix dependencyMat = outputExpressions[i].getDependencyPattern( x );
		dependencyMat.appendCols( outputExpressions[i].getDependencyPattern( z ) );
		dependencyMat.appendCols( outputExpressions[i].getDependencyPattern( u ) );
		dependencyMat.appendCols( outputExpressions[i].getDependencyPattern( dx ) );

		outputDependencies.push_back( dependencyMat );

		if((MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID) totalMeas.push_back( (int)ceil((double)outputGrids[i].getNumIntervals()/((double) grid.getNumIntervals()) - 10.0*EPS) );
		else totalMeas.push_back( outputGrids[i].getNumIntervals() );

		// Export output grids
		ExportVariable gridVariable( (String)"gridOutput" << i, 1, totalMeas[i], REAL, ACADO_VARIABLES );
		gridVariables.push_back( gridVariable );
	}
	
	setup();
	rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, ACADO_WORKSPACE );
	rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(NVARS), REAL, ACADO_WORKSPACE );
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

		int measGrid;
		get( MEASUREMENT_GRID, measGrid );

		uint i;
		uint maxOutputs = 0;
		rk_outputs.clear();
		OUTPUTS.clear();
		diffs_OUTPUTS.clear();
		for( i = 0; i < outputGrids.size(); i++ ) {
			uint numOutputs = num_OUTPUTS[i];
			uint outputDim = outputGrids[i].getNumIntervals( )*numOutputs*(NX+NU+1);

			if( numOutputs > maxOutputs ) maxOutputs = numOutputs;

			ExportVariable rk_output( String("rk_output")<<String(i), 1, outputDim, REAL );
			rk_outputs.push_back( rk_output );

			if((MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID) totalMeas.push_back( (int)ceil((double)outputGrids[i].getNumIntervals()/((double) grid.getNumIntervals()) - 10.0*EPS) );
			else totalMeas.push_back( outputGrids[i].getNumIntervals() );

			// Export output grids
			ExportVariable gridVariable( (String)"gridOutput" << i, 1, totalMeas[i], REAL, ACADO_VARIABLES );
			gridVariables.push_back( gridVariable );
		}

		setup();
		rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, ACADO_WORKSPACE );
		rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(NVARS), REAL, ACADO_WORKSPACE );
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


returnValue ImplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<String> _outputNames,
									  	  const std::vector<String> _diffs_outputNames,
										  const std::vector<uint> _dims_output,
										  const std::vector<Matrix> _outputDependencies ) {

	outputDependencies = _outputDependencies;
	CRS_FORMAT = BT_TRUE;

	return setupOutput( outputGrids_, _outputNames, _diffs_outputNames, _dims_output );
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
	
	gridVariables = arg.gridVariables;
	totalMeas = arg.totalMeas;

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
