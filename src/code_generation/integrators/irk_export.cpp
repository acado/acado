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
	reuse = BT_TRUE;
	continuousOutput = BT_FALSE;

	solver = 0;
}

ImplicitRungeKuttaExport::ImplicitRungeKuttaExport( const ImplicitRungeKuttaExport& arg ) : RungeKuttaExport( arg )
{
	diffsDim = 0;
	inputDim = 0;
	numIts = arg.numIts ;
	numItsInit = arg.numItsInit ;
    diffs_outputs = arg.diffs_outputs;
    outputs = arg.outputs;
	outputGrids = arg.outputGrids;
    solver = arg.solver;
	reuse = arg.reuse;;
	continuousOutput = arg.continuousOutput;
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


returnValue ImplicitRungeKuttaExport::setDifferentialEquation(	const Expression& rhs_ )
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
	if( rhs_.getNumRows() != (NX+NXA) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	
	DifferentialEquation f;
	f << rhs_;
	
	DifferentialEquation g;
	for( uint i = 0; i < rhs_.getDim(); i++ ) {
		g << forwardDerivative( rhs_(i), x );
		g << forwardDerivative( rhs_(i), z );
		g << forwardDerivative( rhs_(i), u );
		g << forwardDerivative( rhs_(i), dx );
	}

	setup();
	return (rhs.init( f,"acado_rhs",NX,NXA,NU ) & diffs_rhs.init( g,"acado_diffs",NX,NXA,NU ) );
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
	
	if( exportRhs ) {
		ExportVariable max = rhs.getGlobalExportVariable();
		if( diffs_rhs.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_rhs.getGlobalExportVariable();
		}
		uint i;
		for( i = 0; i < outputs.size(); i++ ) {
			if( outputs[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
				max = outputs[i].getGlobalExportVariable();
			}
			if( diffs_outputs[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
				max = diffs_outputs[i].getGlobalExportVariable();
			}
		}
		declarations.addDeclaration( max,dataStruct );
	}

	declarations.addDeclaration( rk_ttt,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( rk_kkk,dataStruct );
	declarations.addDeclaration( rk_A,dataStruct );
	declarations.addDeclaration( rk_b,dataStruct );
	declarations.addDeclaration( rk_rhsTemp,dataStruct );
	declarations.addDeclaration( rk_diffsTemp,dataStruct );
	declarations.addDeclaration( reset_int,dataStruct );
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) declarations.addDeclaration( rk_diffsPrev,dataStruct );
	declarations.addDeclaration( rk_diffsNew,dataStruct );
	
	if( continuousOutput ) {
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

	if( exportRhs ) {
		declarations.addDeclaration( rhs );
		declarations.addDeclaration( diffs_rhs );
	}
	else {
		Function tmpFun;
		ExportODEfunction tmpExport(tmpFun, getNameRHS());
		declarations.addDeclaration( tmpExport );
		tmpExport = ExportODEfunction(tmpFun, getNameDiffsRHS());
		declarations.addDeclaration( tmpExport );
	}
	uint i;
	if( exportRhs && continuousOutput ) {
		for( i = 0; i < outputs.size(); i++ ) {
			declarations.addDeclaration( outputs[i] );
			declarations.addDeclaration( diffs_outputs[i] );
		}
	}
	else {
		for( i = 0; i < name_outputs.size(); i++ ) {
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
	if( exportRhs ) {
		code.addFunction( rhs );
		code.addStatement( "\n\n" );
		code.addFunction( diffs_rhs );
		code.addStatement( "\n\n" );

		if( continuousOutput ) {
			uint i;
			for( i = 0; i < outputs.size(); i++ ) {
				code.addFunction( outputs[i] );
				code.addStatement( "\n\n" );
				code.addFunction( diffs_outputs[i] );
				code.addStatement( "\n\n" );
			}
		}
	}

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	// export RK scheme
	uint run2, run3, run4, run5;
	String tempString;
	
	Matrix zeroM = zeros( 1,NX+NXA );
	initializeButcherTableau();
	initializeDDMatrix();
	initializeCoefficients();

	ExportVariable Ah;
	ExportVariable Bh;
	ExportVariable rk_tPrev;
	if( equidistant ) {
		double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
		Ah = ExportVariable( "Ah_mat", AA*=h );
		code.addDeclaration( Ah );
		code.addLinebreak( 2 );
		// TODO: Ask Milan why this does NOT work properly !!
		Vector BB( bb );
		Bh = ExportVariable( "Bh_mat", Matrix( BB*=h, BT_TRUE ) );
	}
	else {
		Ah = ExportVariable( "Ah_mat", numStages, numStages, REAL, ACADO_LOCAL );
		Bh = ExportVariable( "Bh_mat", 1, numStages, REAL, ACADO_LOCAL );
		integrate.addDeclaration( Ah );
		integrate.addDeclaration( Bh );

		Vector steps(grid.getNumIntervals());
		for( run5 = 0; run5 < grid.getNumIntervals(); run5++ ) {
			steps(run5) = grid.getTime(run5+1) - grid.getTime(run5);
		}
		stepsH = ExportVariable( "steps_H", steps );
		integrate.addDeclaration( stepsH );
		stepsH = ExportVariable( "steps_H", 1, grid.getNumIntervals(), REAL, ACADO_LOCAL );

		if( (MeasurementGrid)measGrid == ONLINE_GRID ) {
			rk_tPrev = ExportVariable( "rk_ttt_prev", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );
			integrate.addDeclaration( rk_tPrev );
		}
	}
	
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

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !equidistantControlGrid() ) {
		ExportVariable numStepsV( "numSteps", numSteps, STATIC_CONST_INT );
		code.addDeclaration( numStepsV );
		code.addLinebreak( 2 );
		integrate.addStatement( String( "int " ) << numInt.getName() << " = " << numStepsV.getName() << "[" << rk_index.getName() << "];\n" );
	}

	for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
		if( (MeasurementGrid)measGrid != ONLINE_GRID ) {
			Matrix polynV = evaluatePolynomial( run5 );
			Matrix polynDerV = evaluateDerivedPolynomial( run5 );
			Vector measurements = divideMeasurements( run5 );

			ExportVariable polynVariable;
			if( equidistant ) {
				double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
				polynVariable = ExportVariable( (String)"polynOutput" << run5, polynV*=h );
			}
			else {
				polynVariable = ExportVariable( (String)"polynOutput" << run5, polynV );
			}
			code.addDeclaration( polynVariable );
			polynVariable = ExportVariable( (String)"polynOutput" << run5, totalMeas[run5], numStages );
			polynVariables.push_back( polynVariable );

			if( NXA > 0 || NDX > 0 ) {
				ExportVariable polynDerVariable( (String)"polynDerOutput" << run5, polynDerV );
				code.addDeclaration( polynDerVariable );
				polynDerVariable = ExportVariable( (String)"polynDerOutput" << run5, totalMeas[run5], numStages );
				polynDerVariables.push_back( polynDerVariable );
			}

			ExportVariable numMeasVariable( (String)"numMeas" << run5, measurements, STATIC_CONST_INT );
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
	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !equidistantControlGrid()) ) {
		integrate.addIndex( tmp_index4 );
	}
	ExportVariable time_tmp( "time_tmp", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );
	if( continuousOutput ) {
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
	integrate.addStatement( rk_ttt == Matrix(grid.getFirstTime()) );
	integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
	integrate.addLinebreak( );

    // integrator loop:
	ExportForLoop tmpLoop( run, 0, grid.getNumIntervals() );
	ExportStatementBlock *loop;
	if( equidistantControlGrid() ) {
		loop = &tmpLoop;
	}
	else {
	    loop = &integrate;
		loop->addStatement( String("for(") << run.getName() << " = 0; " << run.getName() << " < " << numInt.getName() << "; " << run.getName() << "++ ) {\n" );
	}

	if( equidistant ) {
		loop->addStatement( rk_ttt += Matrix(1.0/grid.getNumIntervals()) );
	}
	else {
		if( (MeasurementGrid)measGrid == ONLINE_GRID ) loop->addStatement( rk_tPrev == rk_ttt );
		loop->addStatement( rk_ttt += Matrix(1.0/(grid.getLastTime()-grid.getFirstTime()))*stepsH.getCol( run ) );
	}

	if( !equidistant ) {
		// MULTIPLICATIONS WITH THE CORRECT INTEGRATION STEP SIZE
		for( run5 = 0; run5 < numStages; run5++ ) {
			loop->addStatement( Ah.getRow( run5 ) == stepsH.getCol( run )*Matrix(AA.getRow( run5 ),BT_TRUE) );
		}
		loop->addStatement( Bh == stepsH.getCol( run )*Matrix( bb, BT_TRUE ) );
	}

	if( continuousOutput && (MeasurementGrid)measGrid == ONLINE_GRID ) {
		for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
			loop->addStatement( tmp_index1 == numMeas[run5] );
			loop->addStatement( String("while( ") << tmp_index1.getName() << " < " << String(totalMeas[run5]) << " && " << gridVariables[run5].get(0,tmp_index1) << " <= " << rk_ttt.getFullName() << " ) {\n" );
			loop->addStatement( tmp_index1 == tmp_index1+1 );
			loop->addStatement( String("}\n") );
			loop->addStatement( String(tmp_meas.get( 0,run5 )) << " = " << tmp_index1.getName() << " - " << numMeas[run5].getName() << ";\n" );
		}
	}

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		// Set rk_diffsPrev:
		loop->addStatement( String("if( run > 0 ) {\n") );
		ExportForLoop loopTemp( i,0,NX );
		loopTemp.addStatement( rk_diffsPrev.getSubMatrix( i,i+1,0,NX ) == rk_eta.getCols( i*NX+NX+NXA,i*NX+NX+NXA+NX ) );
		if( NU > 0 ) loopTemp.addStatement( rk_diffsPrev.getSubMatrix( i,i+1,NX,NX+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1),i*NU+(NX+NXA)*(NX+1)+NU ) );
		loop->addStatement( loopTemp );
		loop->addStatement( String("}\n") );
	}

	if( reuse ) loop->addStatement( String( "if( " ) << reset_int.getFullName() << " ) {\n" );
	// Initialization iterations:
	ExportForLoop loop1( i,0,numItsInit+1 ); // NOTE: +1 because 0 will lead to NaNs, so the minimum number of iterations is 1 at the initialization
	evaluateMatrix( &loop1, run1, j, tmp_index1, Ah, BT_TRUE );
	loop1.addFunctionCall( solver->getNameSolveFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0) );
	ExportForLoop loopTemp( j,0,numStages );
	loopTemp.addStatement( rk_kkk.getSubMatrix( j,j+1,0,NX ) += rk_b.getCols( j*NX,j*NX+NX ) );													// differential states
	if(NXA > 0) loopTemp.addStatement( rk_kkk.getSubMatrix( j,j+1,NX,NX+NXA ) += rk_b.getCols( j*NXA+numStages*NX,j*NXA+numStages*NX+NXA ) );		// algebraic states
	loop1.addStatement( loopTemp );
	loop->addStatement( loop1 );
	if( reuse ) loop->addStatement( String( "}\n" ) );


	// the rest (numIts) of the Newton iterations with reuse of the Jacobian (no evaluation or factorization needed)
	ExportForLoop loop2( i,0,numIts );
	ExportForLoop loop21( run1,0,numStages );
	loop21.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
	if( NXA > 0 ) loop21.addStatement( rk_xxx.getCols( NX,NX+NXA ) == rk_kkk.getSubMatrix( run1,run1+1,NX,NX+NXA ) );
	if( NDX > 0 ) loop21.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( run1,run1+1,0,NX ) );

	loop21.addFunctionCall( getNameRHS(), rk_xxx, rk_rhsTemp.getAddress(0,0) );

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
	loop2.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0) );
	loopTemp = ExportForLoop( j,0,numStages );
	loopTemp.addStatement( rk_kkk.getSubMatrix( j,j+1,0,NX ) += rk_b.getCols( j*NX,j*NX+NX ) );														// differential states
	if(NXA > 0) loopTemp.addStatement( rk_kkk.getSubMatrix( j,j+1,NX,NX+NXA ) += rk_b.getCols( j*NXA+numStages*NX,j*NXA+numStages*NX+NXA ) );		// algebraic states
	loop2.addStatement( loopTemp );
	loop->addStatement( loop2 );

	// solution calculated --> evaluate and save the necessary derivatives in rk_diffsTemp and update the matrix rk_A:
	evaluateMatrix( loop, run1, j, tmp_index1, Ah, BT_FALSE );

	// generate continuous output:
	generateOutput( loop, run, i, tmp_index2, tmp_index3, tmp_meas, rk_tPrev, time_tmp );

	Matrix tempCoefs( evaluateDerivedPolynomial( 0.0 ), BT_TRUE );

	// derivatives wrt the states (IFT):
	ExportForLoop loop4( run1,0,NX );
	ExportForLoop loop40( i,0,numStages );
	for( run3 = 0; run3 < NX+NXA; run3++ ) {
		loop40.addStatement( rk_b.getCol( i*(NX+NXA)+run3 ) == zeroM.getCol( 0 ) - rk_diffsTemp.getSubMatrix( i,i+1,run1+run3*(nVars),run1+run3*(nVars)+1 ) );
	}
	loop4.addStatement( loop40 );
	loop4.addStatement( String( "if( 0 == " ) << run1.getName() << " ) {\n" );	// factorization of the new matrix rk_A not yet calculated!
	loop4.addFunctionCall( solver->getNameSolveFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0) );
	loop4.addStatement( String( "}\n else {\n" ) );
	loop4.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0) );
	loop4.addStatement( String( "}\n" ) );

	// generate sensitivities wrt states for continuous output:
	generateSensitivitiesOutput( &loop4, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, rk_tPrev, time_tmp, BT_TRUE );

	// update rk_diffsNew with the new sensitivities:
	ExportForLoop loop43( i,0,NX );
	loop43.addStatement( String(rk_diffsNew.get( i,run1 )) << " = (" << i.getName() << " == " << run1.getName() << ");\n" );
	for (run3 = 0; run3 < numStages; run3++ ) {
		// TODO: simplify when ExportVariable can contain constant data again !
		if( equidistant ) {
			double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
			loop43.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1,run1+1 ) += (h*bb( run3 ))*rk_b.getCol( i+run3*NX ) );
		}
		else {
			loop43.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1,run1+1 ) += Bh.getCol( run3 )*rk_b.getCol( i+run3*NX ) );
		}
	}
	loop4.addStatement( loop43 );
	if( NXA > 0 ) {
		loop4.addStatement( String("if( run == 0 ) {\n") );
		ExportForLoop loop44( i,0,NXA );
		loop44.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1,run1+1 ) == tempCoefs.getCol( 0 )*rk_b.getCol( i+numStages*NX ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop44.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1,run1+1 ) += tempCoefs.getCol( run3 )*rk_b.getCol( i+numStages*NX+run3*NXA ) );
		}
		loop4.addStatement( loop44 );
		loop4.addStatement( String("}\n") );
	}
	loop->addStatement( loop4 );


	// derivatives wrt the control inputs (IFT):
	if( NU > 0 ) {
	ExportForLoop loop5( run1,0,NU );
	ExportForLoop loop50( i,0,numStages );
	for( run3 = 0; run3 < NX+NXA; run3++ ) {
		loop50.addStatement( rk_b.getCol( i*(NX+NXA)+run3 ) == zeroM.getCol( 0 ) - rk_diffsTemp.getSubMatrix( i,i+1,run1+run3*(nVars)+NX+NXA,run1+run3*(nVars)+NX+NXA+1 ) );
	}
	loop5.addStatement( loop50 );
	loop5.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0) );

	// generate sensitivities wrt controls for continuous output:
	generateSensitivitiesOutput( &loop5, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, rk_tPrev, time_tmp, BT_FALSE );

	// update rk_diffsNew with the new sensitivities:
	ExportForLoop loop53( i,0,NX );
	loop53.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1+NX,run1+NX+1 ) == Bh.getCol( 0 )*rk_b.getCol( i ) );
	for (run3 = 1; run3 < numStages; run3++ ) {
		// TODO: simplify when ExportVariable can contain constant data again !
		if( equidistant ) {
			double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
			loop53.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1+NX,run1+NX+1 ) += (h*bb( run3 ))*rk_b.getCol( i+run3*NX ) );
		}
		else {
			loop53.addStatement( rk_diffsNew.getSubMatrix( i,i+1,run1+NX,run1+NX+1 ) += Bh.getCol( run3 )*rk_b.getCol( i+run3*NX ) );
		}
	}
	loop5.addStatement( loop53 );
	if( NXA > 0 ) {
		loop5.addStatement( String("if( run == 0 ) {\n") );
		ExportForLoop loop54( i,0,NXA );
		loop54.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1+NX,run1+NX+1 ) == tempCoefs.getCol( 0 )*rk_b.getCol( i+numStages*NX ) );
		for (run3 = 1; run3 < numStages; run3++ ) {
			loop54.addStatement( rk_diffsNew.getSubMatrix( i+NX,i+NX+1,run1+NX,run1+NX+1 ) += tempCoefs.getCol( run3 )*rk_b.getCol( i+numStages*NX+run3*NXA ) );
		}
		loop5.addStatement( loop54 );
		loop5.addStatement( String("}\n") );
	}
	loop->addStatement( loop5 );
	}


	// update rk_eta:
	loop->addStatement( rk_eta.getCols( 0,NX ) += Bh*rk_kkk.getCols( 0,NX ) );
	if( NXA > 0) {
		loop->addStatement( String("if( run == 0 ) {\n") );
		loop->addStatement( rk_eta.getCols( NX,NX+NXA ) == tempCoefs*rk_kkk.getCols( NX,NX+NXA ) );
		loop->addStatement( String("}\n") );
	}


	// Computation of the sensitivities using the chain rule:
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( String( "if( run == 0 ) {\n" ) );
	}
	ExportForLoop loop01( i,0,NX+NXA );
	ExportForLoop loop02( j,0,NX );
	loop02.addStatement( tmp_index2 == j+i*NX );
	loop02.addStatement( rk_eta.getCol( tmp_index2+NX+NXA ) == rk_diffsNew.getSubMatrix( i,i+1,j,j+1 ) );
	loop01.addStatement( loop02 );
	loop->addStatement( loop01 );

	if( NU > 0 ) {
		ExportForLoop loop03( i,0,NX+NXA );
		ExportForLoop loop04( j,0,NU );
		loop04.addStatement( tmp_index2 == j+i*NU );
		loop04.addStatement( rk_eta.getCol( tmp_index2+(NX+NXA)*(1+NX) ) == rk_diffsNew.getSubMatrix( i,i+1,NX+j,NX+j+1 ) );
		loop03.addStatement( loop04 );
		loop->addStatement( loop03 );
	}

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( String( "}\n" ) );
		loop->addStatement( String( "else {\n" ) );
		ExportForLoop loop01( i,0,NX );
		ExportForLoop loop02( j,0,NX );
		loop02.addStatement( tmp_index2 == j+i*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index2+NX+NXA ) == rk_diffsNew.getSubMatrix( i,i+1,0,1 )*rk_diffsPrev.getSubMatrix( 0,1,j,j+1 ) );
		ExportForLoop loop021( k,1,NX );
		loop021.addStatement( rk_eta.getCol( tmp_index2+NX+NXA ) += rk_diffsNew.getSubMatrix( i,i+1,k,k+1 )*rk_diffsPrev.getSubMatrix( k,k+1,j,j+1 ) );
		loop02.addStatement( loop021 );
		loop01.addStatement( loop02 );
		loop->addStatement( loop01 );

		if( NU > 0 ) {
			ExportForLoop loop03( i,0,NX );
			ExportForLoop loop04( j,0,NU );
			loop04.addStatement( tmp_index2 == j+i*NU );
			loop04.addStatement( rk_eta.getCol( tmp_index2+(NX+NXA)*(1+NX) ) == rk_diffsNew.getSubMatrix( i,i+1,NX+j,NX+j+1 ) + rk_diffsNew.getSubMatrix( i,i+1,0,1 )*rk_diffsPrev.getSubMatrix( 0,1,NX+j,NX+j+1 ) );
			ExportForLoop loop041( k,1,NX );
			loop041.addStatement( rk_eta.getCol( tmp_index2+(NX+NXA)*(1+NX) ) += rk_diffsNew.getSubMatrix( i,i+1,k,k+1 )*rk_diffsPrev.getSubMatrix( k,k+1,NX+j,NX+j+1 ) );
			loop04.addStatement( loop041 );
			loop03.addStatement( loop04 );
			loop->addStatement( loop03 );
		}
	}

	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !equidistantControlGrid()) ) {

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
	}

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( String( "}\n" ) );
	}

	loop->addStatement( String( reset_int.get(0,0) ) << " = 0;\n" );

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
    if( !equidistantControlGrid() ) {
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


returnValue ImplicitRungeKuttaExport::evaluateMatrix( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index, const ExportVariable& Ah, BooleanType evaluateB )
{
	Matrix zeroM = zeros( 1,NX+NXA );
	uint i;

	ExportForLoop loop1( index1,0,numStages );
	loop1.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(index1)*rk_kkk.getCols( 0,NX ) );
	if( NXA > 0 ) loop1.addStatement( rk_xxx.getCols( NX,NX+NXA ) == rk_kkk.getSubMatrix( index1,index1+1,NX,NX+NXA ) );
	if( NDX > 0 ) loop1.addStatement( rk_xxx.getCols( inputDim-diffsDim,inputDim-diffsDim+NDX ) == rk_kkk.getSubMatrix( index1,index1+1,0,NX ) );

	if( evaluateB ) loop1.addFunctionCall( getNameRHS(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
	loop1.addFunctionCall( getNameDiffsRHS(), rk_xxx, rk_diffsTemp.getAddress(index1,0) );
	ExportForLoop loop2( index2,0,NX+NXA );
	loop2.addStatement( tmp_index == index1*(NX+NXA)+index2 );
	for( i = 0; i < numStages; i++ ) { // differential states
		if( NDX == 0 ) {
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX,i*NX+NX ) == Ah.getSubMatrix( index1,index1+1,i,i+1 )*rk_diffsTemp.getSubMatrix( index1,index1+1,index2*(nVars),index2*(nVars)+NX ) );
			loop2.addStatement( String( "if( " ) << i << " == " << index1.getName() << " ) " );
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,index2+i*NX,index2+i*NX+1 ) -= 1 );
		}
		else {
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX,i*NX+NX ) == Ah.getSubMatrix( index1,index1+1,i,i+1 )*rk_diffsTemp.getSubMatrix( index1,index1+1,index2*(nVars),index2*(nVars)+NX ) );
			loop2.addStatement( String( "if( " ) << i << " == " << index1.getName() << " ) {\n" );
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX,i*NX+NX ) += rk_diffsTemp.getSubMatrix( index1,index1+1,index2*(nVars)+NX+NXA+NU,index2*(nVars)+nVars ) );
			loop2.addStatement( String( "}\n" ) );
		}
	}
	if( NXA > 0 ) {
		for( i = 0; i < numStages; i++ ) { // algebraic states
			loop2.addStatement( String( "if( " ) << i << " == " << index1.getName() << " ) {\n" );
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,numStages*NX+i*NXA,numStages*NX+i*NXA+NXA ) == rk_diffsTemp.getSubMatrix( index1,index1+1,index2*(nVars)+NX,index2*(nVars)+NX+NXA ) );
			loop2.addStatement( String( "}\n else {\n" ) );
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,numStages*NX+i*NXA,numStages*NX+i*NXA+NXA ) == zeroM.getCols( 0,NXA-1 ) );
			loop2.addStatement( String( "}\n" ) );
		}
	}
	loop1.addStatement( loop2 );
	if( evaluateB ) {
		// vector rk_b:
		if( NDX == 0 ) {
			loop1.addStatement( rk_b.getCols( index1*(NX+NXA),index1*(NX+NXA)+NX ) == rk_kkk.getSubMatrix( index1,index1+1,0,NX ) - rk_rhsTemp.getCols( 0,NX ) );
		}
		else {
			loop1.addStatement( rk_b.getCols( index1*(NX+NXA),index1*(NX+NXA)+NX ) == zeroM.getCols( 0,NX-1 ) - rk_rhsTemp.getCols( 0,NX ) );
		}
		if( NXA > 0 ) loop1.addStatement( rk_b.getCols( index1*(NX+NXA)+NX,(index1+1)*(NX+NXA) ) == zeroM.getCols( 0,NXA-1 ) - rk_rhsTemp.getCols( NX,NX+NXA ) );
	}
	block->addStatement( loop1 );

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::generateOutput( ExportStatementBlock* block, const ExportIndex& index0,
		const ExportIndex& index1, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
		const ExportVariable& tmp_meas, const ExportVariable& rk_tPrev, const ExportVariable& time_tmp )
{
	uint i, j;
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	for( i = 0; i < rk_outputs.size(); i++ ) {
		ExportStatementBlock *loop22;
		ExportForLoop tmpLoop2( index1,0,totalMeas[i] );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop22 = &tmpLoop2;
			if( equidistant ) {
				loop22->addStatement( rk_outH == polynVariables[i].getRow( index1 ) );
			}
			else {
				loop22->addStatement( rk_outH == stepsH.getCol( index0 )*polynVariables[i].getRow( index1 ) );
			}
			if( NXA > 0 || NDX > 0 ) loop22->addStatement( rk_out2 == polynDerVariables[i].getRow( index1 ) );
		}
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop22 = block;
			loop22->addStatement( String("for(") << index1.getName() << " = 0; " << index1.getName() << " < (int)" << numMeasVariables[i].get(0,index0) << "; " << index1.getName() << "++) {\n" );
			loop22->addStatement( tmp_index1 == numMeas[i]+index1 );
			if( equidistant ) {
				loop22->addStatement( rk_outH == polynVariables[i].getRow( tmp_index1 ) );
			}
			else {
				loop22->addStatement( rk_outH == stepsH.getCol( index0 )*polynVariables[i].getRow( tmp_index1 ) );
			}
			if( NXA > 0 || NDX > 0 ) loop22->addStatement( rk_out2 == polynDerVariables[i].getRow( tmp_index1 ) );
		}
		else { // ONLINE_GRID
			loop22 = block;
			loop22->addStatement( String(tmp_index2.getName()) << " = " << tmp_meas.get( 0,i ) << ";\n" );
			loop22->addStatement( String("for(") << index1.getName() << " = 0; " << index1.getName() << " < (int)" << tmp_index2.getName() << "; " << index1.getName() << "++) {\n" );
			loop22->addStatement( tmp_index1 == numMeas[i]+index1 );

			if( equidistant ) {
				uint scale = grid.getNumIntervals();
				double scale2 = 1.0/grid.getNumIntervals();
				loop22->addStatement( time_tmp.getName() << " = " << String(scale) << "*(" << gridVariables[i].get(0,tmp_index1) << "-" << String(scale2) << "*" << index0.getName() << ");\n" );
			}
			else {
				loop22->addStatement( time_tmp.getName() << " = " << (grid.getLastTime()-grid.getFirstTime()) << "*(" << gridVariables[i].get(0,tmp_index1) << "-" << rk_tPrev.getName() << ")/" << stepsH.get( 0,index0 ) << ";\n" );
			}

			String h;
			if( equidistant ) 	h = String((grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals());
			else				h = stepsH.get( 0,index0 );
			evaluatePolynomial( *loop22, rk_outH, time_tmp, h );
			if( NXA > 0 || NDX > 0 ) evaluateDerivedPolynomial( *loop22, rk_out2, time_tmp );
		}

		Vector dependencyX, dependencyZ, dependencyDX;
		if( exportRhs || crsFormat ) {
			dependencyX = sumRow( outputDependencies[i].getCols( 0,NX-1 ) );
			if( NXA > 0 ) dependencyZ = sumRow( outputDependencies[i].getCols( NX,NX+NXA-1 ) );
			if( NDX > 0 ) dependencyDX = sumRow( outputDependencies[i].getCols( NX+NXA+NU,nVars-1 ) );
		}
		for( j = 0; j < NX; j++ ) {
			if( (!exportRhs && !crsFormat) || (int)dependencyX(j) != 0 ) {
				loop22->addStatement( rk_xxx.getCols( j,j+1 ) == rk_eta.getCols( j,j+1 ) + rk_outH*rk_kkk.getCol( j ) );
			}
		}
		for( j = 0; j < NXA; j++ ) {
			if( (!exportRhs && !crsFormat) || (int)dependencyZ(j) != 0 ) {
				loop22->addStatement( rk_xxx.getCols( NX+j,NX+j+1 ) == rk_out2*rk_kkk.getCol( NX+j ) );
			}
		}
		for( j = 0; j < NDX; j++ ) {
			if( (!exportRhs && !crsFormat) || (int)dependencyDX(j) != 0 ) {
				loop22->addStatement( rk_xxx.getCols( inputDim-diffsDim+j,inputDim-diffsDim+j+1 ) == rk_out2*rk_kkk.getCol( j ) );
			}
		}
		loop22->addFunctionCall( getNameOUTPUT( i ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( i );
		uint outputDim = numOutputs*(NX+NU+1);
		loop22->addStatement( tmp_index1 == numMeas[i]*outputDim+index1*(numOutputs*(NX+NU+1)) );
		for( j = 0; j < numOutputs; j++ ) {
			loop22->addStatement( rk_outputs[i].getCol( tmp_index1+j ) == rk_rhsOutputTemp.getCol( j ) );
		}
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			block->addStatement( *loop22 );
		}
		else {
			loop22->addStatement( "}\n" );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::generateSensitivitiesOutput( ExportStatementBlock* block, const ExportIndex& index0,
		const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
		const ExportIndex& tmp_index3, const ExportVariable& tmp_meas, const ExportVariable& rk_tPrev, const ExportVariable& time_tmp, BooleanType STATES )
{
	uint i, j, k;
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	for( i = 0; i < rk_outputs.size(); i++ ) {
		ExportStatementBlock *loop;
		ExportForLoop tmpLoop( index2,0,totalMeas[i] );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop = &tmpLoop;
			if( equidistant ) {
				loop->addStatement( rk_outH == polynVariables[i].getRow( index2 ) );
			}
			else {
				loop->addStatement( rk_outH == stepsH.getCol( index0 )*polynVariables[i].getRow( index2 ) );
			}
			if( NXA > 0 || NDX > 0 ) loop->addStatement( rk_out2 == polynDerVariables[i].getRow( index2 ) );
		}
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop = block;
			loop->addStatement( String("for(") << index2.getName() << " = 0; " << index2.getName() << " < (int)" << numMeasVariables[i].get(0,index0) << "; " << index2.getName() << "++) {\n" );
			loop->addStatement( tmp_index2 == numMeas[i]+index2 );
			if( equidistant ) {
				loop->addStatement( rk_outH == polynVariables[i].getRow( tmp_index2 ) );
			}
			else {
				loop->addStatement( rk_outH == stepsH.getCol( index0 )*polynVariables[i].getRow( tmp_index2 ) );
			}
			if( NXA > 0 || NDX > 0 ) loop->addStatement( rk_out2 == polynDerVariables[i].getRow( tmp_index2 ) );
		}
		else { // ONLINE_GRID
			loop = block;
			loop->addStatement( String(tmp_index3.getName()) << " = " << tmp_meas.get( 0,i ) << ";\n" );
			loop->addStatement( String("for(") << index2.getName() << " = 0; " << index2.getName() << " < (int)" << tmp_index3.getName() << "; " << index2.getName() << "++) {\n" );
			loop->addStatement( tmp_index2 == numMeas[i]+index2 );

			if( equidistant ) {
				uint scale = grid.getNumIntervals();
				double scale2 = 1.0/grid.getNumIntervals();
				loop->addStatement( time_tmp.getName() << " = " << String(scale) << "*(" << gridVariables[i].get(0,tmp_index2) << "-" << String(scale2) << "*" << index0.getName() << ");\n" );
			}
			else {
				loop->addStatement( time_tmp.getName() << " = " << (grid.getLastTime()-grid.getFirstTime()) << "*(" << gridVariables[i].get(0,tmp_index2) << "-" << rk_tPrev.getName() << ")/" << stepsH.get( 0,index0 ) << ";\n" );
			}

			String h;
			if( equidistant ) 	h = String((grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals());
			else				h = stepsH.get( 0,index0 );
			evaluatePolynomial( *loop, rk_outH, time_tmp, h );
			if( NXA > 0 || NDX > 0 ) evaluateDerivedPolynomial( *loop, rk_out2, time_tmp );
		}

		Vector dependencyX, dependencyZ, dependencyDX;
		if( exportRhs || crsFormat ) {
			dependencyX = sumRow( outputDependencies[i].getCols( 0,NX-1 ) );
			if( NXA > 0 ) dependencyZ = sumRow( outputDependencies[i].getCols( NX,NX+NXA-1 ) );
			if( NDX > 0 ) dependencyDX = sumRow( outputDependencies[i].getCols( NX+NXA+NU,nVars-1 ) );
		}
		for( j = 0; j < NX; j++ ) {
			if( (!exportRhs && !crsFormat) || (int)dependencyX(j) != 0 ) {
				if( STATES ) {
					loop->addStatement( String(rk_rhsTemp.get( 0,j )) << " = (" << j << " == " << index1.getName() << ");\n" );
				}
				else {
					loop->addStatement( rk_rhsTemp.getCol( j ) == 0.0 );
				}
				for( k = 0; k < numStages; k++ ) {
					loop->addStatement( rk_rhsTemp.getCol( j ) += rk_outH.getCol( k )*rk_b.getCol( k*NX+j ) );
				}
				loop->addStatement( rk_xxx.getCol( j ) == rk_eta.getCol( j ) + rk_outH*rk_kkk.getCol( j ) );
			}
		}
		for( j = 0; j < NXA; j++ ) {
			if( (!exportRhs && !crsFormat) || (int)dependencyZ(j) != 0 ) {
				loop->addStatement( rk_rhsTemp.getCol( NX+j ) == rk_out2.getCol( 0 )*rk_b.getCol( numStages*NX+j ) );
				for( k = 1; k < numStages; k++ ) {
					loop->addStatement( rk_rhsTemp.getCol( NX+j ) += rk_out2.getCol( k )*rk_b.getCol( numStages*NX+k*NXA+j ) );
				}
				loop->addStatement( rk_xxx.getCol( NX+j ) == rk_out2*rk_kkk.getCol( NX+j ) );
			}
		}
		for( j = 0; j < NDX; j++ ) {
			if( (!exportRhs && !crsFormat) || (int)dependencyDX(j) != 0 ) {
				loop->addStatement( rk_rhsTemp.getCol( NX+NXA+j ) == rk_out2.getCol( 0 )*rk_b.getCol( j ) );
				for( k = 1; k < numStages; k++ ) {
					loop->addStatement( rk_rhsTemp.getCol( NX+NXA+j ) += rk_out2.getCol( k )*rk_b.getCol( k*NX+j ) );
				}
				loop->addStatement( rk_xxx.getCol( inputDim-diffsDim+j ) == rk_out2*rk_kkk.getCol( j ) );
			}
		}

		loop->addFunctionCall( getNameDiffsOUTPUT( i ), rk_xxx, rk_diffsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( i );
		uint outputDim = numOutputs*(NX+NU+1);
		loop->addStatement( tmp_index1 == numMeas[i]*outputDim+index2*(numOutputs*(NX+NU+1)) );
		loop->addStatement( tmp_index2 == tmp_index1+index1 );
		for( j = 0; j < numOutputs; j++ ) {
			if( exportRhs || crsFormat ) {
				dependencyX = (outputDependencies[i].getCols( 0,NX-1 )).getRow( j );
				if( NXA > 0 ) dependencyZ = (outputDependencies[i].getCols( NX,NX+NXA-1 )).getRow( j );
				if( NDX > 0 ) dependencyDX = (outputDependencies[i].getCols( NX+NXA+NU,nVars-1 )).getRow( j );
			}
			uint offset;
			if( STATES ) {
				offset = numOutputs+j*NX;
				loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) == 0.0 );
			}
			else {
				offset = numOutputs*(1+NX)+j*NU;
				loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) == rk_diffsOutputTemp.getCol( index1+j*nVars+NX+NXA ) );
			}

			for( k = 0; k < NX; k++ ) {
				if( (!exportRhs && !crsFormat) || (int)dependencyX(k) != 0 ) {
					loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) += rk_diffsOutputTemp.getCol( j*nVars+k )*rk_rhsTemp.getCol( k ) );
				}
			}
			for( k = 0; k < NXA; k++ ) {
				if( (!exportRhs && !crsFormat) || (int)dependencyZ(k) != 0 ) {
					loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) += rk_diffsOutputTemp.getCol( j*nVars+NX+k )*rk_rhsTemp.getCol( NX+k ) );
				}
			}
			for( k = 0; k < NDX; k++ ) {
				if( (!exportRhs && !crsFormat) || (int)dependencyDX(k) != 0 ) {
					loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) += rk_diffsOutputTemp.getCol( j*nVars+NX+NXA+NU+k )*rk_rhsTemp.getCol( NX+NXA+k ) );
				}
			}
		}
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			block->addStatement( *loop );
		}
		else {
			loop->addStatement( "}\n" );
		}
	}

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
	Matrix polynV(totalMeas[index],numStages);

	double scale = 1.0/outputGrids[index].getTime( totalMeas[index] );
	double scale2 = 1.0/(grid.getLastTime() - grid.getFirstTime());
	for( uint i = 0; i < totalMeas[index]; i++ ) {
		double time = scale*outputGrids[index].getTime(i);
		if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			uint interv = getIntegrationInterval( time );
			time = (time-scale2*grid.getTime(interv))/(scale2*(grid.getTime(interv+1)-grid.getTime(interv)));
		}
		polynV.setRow( i, evaluatePolynomial( time ) );
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


returnValue ImplicitRungeKuttaExport::evaluatePolynomial( ExportStatementBlock& block, const ExportVariable& variable, const ExportVariable& gridVariable, const String& h )
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
	Matrix polynDerV(totalMeas[index],numStages);

	double scale = 1.0/outputGrids[index].getTime( totalMeas[index] );
	double scale2 = 1.0/(grid.getLastTime() - grid.getFirstTime());
	for( uint i = 0; i < totalMeas[index]; i++ ) {
		double time = scale*outputGrids[index].getTime(i);
		if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			uint interv = getIntegrationInterval( time );
			time = (time-scale2*grid.getTime(interv))/(scale2*(grid.getTime(interv+1)-grid.getTime(interv)));
		}
		polynDerV.setRow( i, evaluateDerivedPolynomial( time ) );
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

	for( uint i = 0; i < outputGrids[index].getNumIntervals(); i++ ) {
		uint interv = getIntegrationInterval( outputGrids[index].getTime(i) );
		meas(interv) = meas(interv)+1;
	}

	return meas;
}


returnValue ImplicitRungeKuttaExport::setup( )
{
	if( continuousOutput && !equidistantControlGrid() ) return ACADOERROR( RET_INVALID_OPTION );
	if( !equidistant && !equidistantControlGrid() ) return ACADOERROR( RET_INVALID_OPTION );

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );
	if( !equidistant && (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) return ACADOERROR( RET_INVALID_OPTION );

	int intMode;
	userInteraction->get( IMPLICIT_INTEGRATOR_MODE,intMode ); 
	switch( (ImplicitIntegratorMode) intMode ) {
		case IFTR:
			reuse = BT_TRUE;
			break;
		case IFT:
			reuse = BT_FALSE;
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
	
	nVars = NX+NXA+NU+NDX;
	diffsDim = (NX+NXA)*(NX+NU);
	inputDim = (NX+NXA)*(NX+NU+1) + NU + NP;
	
	rk_ttt = ExportVariable( "rk_ttt", 1, 1, REAL, ACADO_WORKSPACE, BT_TRUE );
	rk_xxx = ExportVariable( "rk_xxx", 1, inputDim-diffsDim+NDX, REAL, ACADO_WORKSPACE );
	rk_kkk = ExportVariable( "rk_kkk", numStages, NX+NXA, REAL, ACADO_WORKSPACE );
	rk_A = ExportVariable( "rk_A", numStages*(NX+NXA), numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_b = ExportVariable( "rk_b", 1, numStages*(NX+NXA), REAL, ACADO_WORKSPACE );
	rk_rhsTemp = ExportVariable( "rk_rhsTemp", 1, NX+NXA+NDX, REAL, ACADO_WORKSPACE );
	rk_diffsTemp = ExportVariable( "rk_diffsTemp", numStages, (NX+NXA)*(nVars), REAL, ACADO_WORKSPACE );
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) rk_diffsPrev = ExportVariable( "rk_diffsPrev", NX, NX+NU, REAL, ACADO_WORKSPACE );
	rk_diffsNew = ExportVariable( "rk_diffsNew", NX+NXA, NX+NU, REAL, ACADO_WORKSPACE );
	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
	if( equidistantControlGrid() ) {
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
	continuousOutput = BT_TRUE;
	if( outputGrids_.size() != outputExpressions_.size() ) return ACADOERROR( RET_INVALID_ARGUMENTS ); 
	outputGrids = outputGrids_;
	outputExpressions = outputExpressions_;

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	uint i;
	uint maxOutputs = 0;
	rk_outputs.clear();
	outputs.clear();
	diffs_outputs.clear();
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
		
		outputs.push_back( OUTPUT );
		diffs_outputs.push_back( diffs_OUTPUT );

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
	rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(nVars), REAL, ACADO_WORKSPACE );
	rk_outH = ExportVariable( "rk_outH", 1, numStages, REAL, ACADO_WORKSPACE );
	rk_out2 = ExportVariable( "rk_out2", 1, numStages, REAL, ACADO_WORKSPACE );
	polynEvalVar = ExportVariable( "tmp_polyn", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );
	
	return ( val );
}


returnValue ImplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<String> _outputNames,
									  	  const std::vector<String> _diffs_outputNames,
										  const std::vector<uint> _dims_output ) {
	if( (rhs.getFunctionDim()) == 0 && (rk_outputs.size() + outputs.size() + diffs_outputs.size()) == 0) {
		continuousOutput = BT_TRUE;
		if( outputGrids_.size() != _outputNames.size() || outputGrids_.size() != _diffs_outputNames.size() || outputGrids_.size() != _dims_output.size() ) {
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		}
		outputGrids = outputGrids_;
		name_outputs = _outputNames;
		name_diffs_outputs = _diffs_outputNames;
		num_outputs = _dims_output;

		int measGrid;
		get( MEASUREMENT_GRID, measGrid );

		uint i;
		uint maxOutputs = 0;
		rk_outputs.clear();
		outputs.clear();
		diffs_outputs.clear();
		for( i = 0; i < outputGrids.size(); i++ ) {
			uint numOutputs = num_outputs[i];
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
		rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(nVars), REAL, ACADO_WORKSPACE );
		rk_outH = ExportVariable( "rk_outH", 1, numStages, REAL, ACADO_WORKSPACE );
		rk_out2 = ExportVariable( "rk_out2", 1, numStages, REAL, ACADO_WORKSPACE );
		polynEvalVar = ExportVariable( "tmp_polyn", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );

		exportRhs = BT_FALSE;
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
	crsFormat = BT_TRUE;

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
	
	rhs = arg.rhs;
	outputs = arg.outputs;
	diffs_rhs = arg.diffs_rhs;
	diffs_outputs = arg.diffs_outputs;
	name_rhs = arg.name_rhs;
	name_outputs = arg.name_outputs;
	num_outputs = arg.num_outputs;
	name_diffs_rhs = arg.name_diffs_rhs;
	name_diffs_outputs = arg.name_diffs_outputs;
	grid = arg.grid;
	outputGrids = arg.outputGrids;
	solver = arg.solver;

	// ExportVariables
	rk_ttt = arg.rk_ttt;
	rk_xxx = arg.rk_xxx;
	rk_kkk = arg.rk_kkk;
	rk_A = arg.rk_A;
	rk_b = arg.rk_b;
	rk_rhsTemp = arg.rk_rhsTemp;
	rk_diffsTemp = arg.rk_diffsTemp;
	rk_diffsPrev = arg.rk_diffsPrev;
	rk_diffsNew = arg.rk_diffsNew;
	rk_eta = arg.rk_eta;
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
	
	reuse = arg.reuse;
	continuousOutput = arg.continuousOutput;
	
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
