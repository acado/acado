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
 *    \file src/code_generation/integrators/irk_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/irk_export.hpp>

#include <sstream>
using namespace std;



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

ImplicitRungeKuttaExport::ImplicitRungeKuttaExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : RungeKuttaExport( _userInteraction,_commonHeaderName )
{
	NX1 = 0;
	NX2 = 0;
	NDX2 = 0;
	NVARS2 = 0;
	NX3 = 0;
	NDX3 = 0;
	NXA3 = 0;
	NVARS3 = 0;

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
	NX1 = arg.NX1;
	NX2 = arg.NX2;
	NDX2 = arg.NDX2;
	NVARS2 = arg.NVARS2;
	NX3 = arg.NX3;
	NDX3 = arg.NDX3;
	NXA3 = arg.NXA3;
	NVARS3 = arg.NVARS3;

	diffsDim = 0;
	inputDim = 0;
	numIts = arg.numIts ;
	numItsInit = arg.numItsInit ;
    diffs_outputs = arg.diffs_outputs;
    outputs = arg.outputs;
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


returnValue ImplicitRungeKuttaExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const Expression& _rhs )
{
	if( !A3.isEmpty() ) {
		if( A3.getNumRows() != M3.getNumRows() || M3.getNumRows() != M3.getNumCols() || A3.getNumRows() != A3.getNumCols() || A3.getNumRows() != _rhs.getDim() ) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		NX3 = A3.getNumRows();
		if( !equidistant ) {
			// TODO: WHAT IF NONEQUIDISTANT INTEGRATION GRID??
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		M33 = M3;
		A33 = A3;

		OutputFcn f;
		f << _rhs;
		Parameter         dummy0;
		Control           dummy1;
		DifferentialState dummy2;
		AlgebraicState 	  dummy3;
		DifferentialStateDerivative dummy4;
		dummy0.clearStaticCounters();
		dummy1.clearStaticCounters();
		dummy2.clearStaticCounters();
		x = DifferentialState(NX1+NX2);
		u = Control(NU);
		p = Parameter(NP);

		if( (uint)f.getNDX() > (NX1+NX2) ) {
			return ACADOERROR( RET_INVALID_OPTION );
		}
		if( f.getNDX() > 0 ) NDX3 = NX1+NX2;
		else NDX3 = 0;
		dummy4.clearStaticCounters();
		dx = DifferentialStateDerivative(NDX3);

		if( f.getNXA() > 0 && NXA == 0 ) {
			return ACADOERROR( RET_INVALID_OPTION );
		}
		if( f.getNXA() > 0 ) NXA3 = NXA;
		else NXA3 = 0;
		dummy3.clearStaticCounters();
		z = AlgebraicState(NXA3);

		uint i;
		OutputFcn g;
		for( i = 0; i < _rhs.getDim(); i++ ) {
			g << forwardDerivative( _rhs(i), x );
			g << forwardDerivative( _rhs(i), z );
			g << forwardDerivative( _rhs(i), u );
			g << forwardDerivative( _rhs(i), dx );
		}

		dummy2.clearStaticCounters();
		x = DifferentialState(NX);

		Matrix dependencyMat = _rhs.getDependencyPattern( x );
		Vector dependency = sumRow( dependencyMat );
		for( i = NX1+NX2; i < NX; i++ ) {
			if( acadoRoundAway(dependency(i)) != 0 ) { // This expression should not depend on these differential states
				return RET_UNABLE_TO_EXPORT_CODE;
			}
		}

		OutputFcn f_large;
		Matrix A3_large = expandOutputMatrix(A3);
		f_large << _rhs + A3_large*x;

		return (rhs3.init( f_large,"acado_rhs3",NX,NXA,NU ) & diffs_rhs3.init( g,"acado_diffs3",NX,NXA,NU ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setDifferentialEquation(	const Expression& rhs_ )
{
	if( rhs_.getDim() > 0 ) {
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

		NX2 = rhs_.getDim() - NXA;
		x = DifferentialState(NX1+NX2);
		z = AlgebraicState(NXA);
		u = Control(NU);
		p = Parameter(NP);

		DifferentialEquation f;
		f << rhs_;

		NDX2 = f.getNDX();
		if( NDX2 > 0 && (NDX2 < NX2 || NDX2 > (NX1+NX2)) ) {
			return ACADOERROR( RET_INVALID_OPTION );
		}
		else if( NDX2 > 0 ) {
			NDX2 = NX1+NX2;
		}
		dx = DifferentialStateDerivative(NDX2);
		NDX = NX;

		DifferentialEquation g;
		for( uint i = 0; i < rhs_.getDim(); i++ ) {
			g << forwardDerivative( rhs_(i), x );
			g << forwardDerivative( rhs_(i), z );
			g << forwardDerivative( rhs_(i), u );
			g << forwardDerivative( rhs_(i), dx );
		}

		return (rhs.init( f,"acado_rhs",NX,NXA,NU ) & diffs_rhs.init( g,"acado_diffs",NX,NXA,NU ) );
	}
	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setModel(	const String& _rhs, const String& _diffs_rhs ) {

	IntegratorExport::setModel( _rhs, _diffs_rhs );

	NDX2 = NDX;

	setup();

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setLinearOutput( const Matrix& M3, const Matrix& A3, const String& _rhs3, const String& _diffs_rhs3 )
{
	if( !A3.isEmpty() ) {
		if( A3.getNumRows() != M3.getNumRows() || M3.getNumRows() != M3.getNumCols() || A3.getNumRows() != A3.getNumCols() ) {
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		NX3 = A3.getNumRows();
		if( !equidistant ) {
			// TODO: WHAT IF NONEQUIDISTANT INTEGRATION GRID??
			return RET_UNABLE_TO_EXPORT_CODE;
		}
		M33 = M3;
		A33 = A3;

		name_rhs3 = String(_rhs3);
		name_diffs_rhs3 = String(_diffs_rhs3);
		exportRhs = BT_FALSE;

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
	}

	return SUCCESSFUL_RETURN;
}


ExportVariable ImplicitRungeKuttaExport::getAuxVariable() const
{
	ExportVariable max;
	if( NX1 > 0 ) {
		max = lin_input.getGlobalExportVariable();
	}
	if( NX2 > 0 || NXA > 0 ) {
		if( rhs.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = rhs.getGlobalExportVariable();
		}
		if( diffs_rhs.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_rhs.getGlobalExportVariable();
		}
	}
	if( NX3 > 0 ) {
		if( rhs3.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = rhs3.getGlobalExportVariable();
		}
		if( diffs_rhs3.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_rhs3.getGlobalExportVariable();
		}
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
	return max;
}


returnValue ImplicitRungeKuttaExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	if( NX2 > 0 || NXA > 0 ) solver->getDataDeclarations( declarations,dataStruct );
	
	if( NX1 > 0 || exportRhs ) {
		ExportVariable max = getAuxVariable();
		int useOMP;
		get(CG_USE_OPENMP, useOMP);
		declarations.addDeclaration( max,dataStruct );
	}

	int debugMode;
	get( INTEGRATOR_DEBUG_MODE, debugMode );
	if ( (BooleanType)debugMode == BT_TRUE ) {
		declarations.addDeclaration( debug_mat,dataStruct );
	}
	declarations.addDeclaration( rk_ttt,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( rk_kkk,dataStruct );
	declarations.addDeclaration( rk_diffK,dataStruct );

	declarations.addDeclaration( rk_A,dataStruct );
	declarations.addDeclaration( rk_b,dataStruct );
	declarations.addDeclaration( rk_auxSolver,dataStruct );
	declarations.addDeclaration( rk_rhsTemp,dataStruct );
	declarations.addDeclaration( rk_diffsTemp2,dataStruct );
	declarations.addDeclaration( rk_diffsTemp3,dataStruct );

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		declarations.addDeclaration( rk_diffsPrev1,dataStruct );
		declarations.addDeclaration( rk_diffsPrev2,dataStruct );
		declarations.addDeclaration( rk_diffsPrev3,dataStruct );
	}

	declarations.addDeclaration( rk_diffsNew1,dataStruct );
	declarations.addDeclaration( rk_diffsNew2,dataStruct );
	declarations.addDeclaration( rk_diffsNew3,dataStruct );
	
	if( CONTINUOUS_OUTPUT ) {
		declarations.addDeclaration( rk_rhsOutputTemp,dataStruct );
		declarations.addDeclaration( rk_diffsOutputTemp,dataStruct );
		declarations.addDeclaration( rk_outH,dataStruct );
		declarations.addDeclaration( rk_out,dataStruct );

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
	if( NX2 > 0) solver->getFunctionDeclarations( declarations );
	if( NX2 != NX ) declarations.addDeclaration( fullRhs );

	if( NX1 > 0 ) {
		declarations.addDeclaration( lin_input );
	}
	if( exportRhs ) {
		if( NX2 > 0 || NXA > 0 ) {
			declarations.addDeclaration( rhs );
			declarations.addDeclaration( diffs_rhs );
		}
		if( NX3 > 0 ) {
			declarations.addDeclaration( rhs3 );
			declarations.addDeclaration( diffs_rhs3 );
		}
	}
	else {
		Function tmpFun;
		tmpFun << zeros(1,1);
		ExportODEfunction tmpExport(tmpFun, getNameRHS());
		declarations.addDeclaration( tmpExport );
		tmpExport = ExportODEfunction(tmpFun, getNameDiffsRHS());
		declarations.addDeclaration( tmpExport );

		if( NX3 > 0 ) {
			tmpExport = ExportODEfunction(tmpFun, getNameOutputRHS());
			declarations.addDeclaration( tmpExport );
			tmpExport = ExportODEfunction(tmpFun, getNameOutputDiffs());
			declarations.addDeclaration( tmpExport );
		}
	}
	uint i;
	if( exportRhs && CONTINUOUS_OUTPUT ) {
		for( i = 0; i < outputs.size(); i++ ) {
			declarations.addDeclaration( outputs[i] );
			declarations.addDeclaration( diffs_outputs[i] );
		}
	}
	else {
		for( i = 0; i < name_outputs.size(); i++ ) {
			Function tmpFun;
			tmpFun << zeros(1,1);
			ExportODEfunction tmpExport(tmpFun, getNameOUTPUT(i));
			declarations.addDeclaration( tmpExport );
			tmpExport = ExportODEfunction(tmpFun, getNameDiffsOUTPUT(i));
			declarations.addDeclaration( tmpExport );
		}
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::getCode(	ExportStatementBlock& code )
{
	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP ) {
		ExportVariable max = getAuxVariable();
		max.setName( "auxVar" );
		max.setDataStruct( ACADO_LOCAL );
		if( NX2 > 0 || NXA > 0 ) {
			rhs.setGlobalExportVariable( max );
			diffs_rhs.setGlobalExportVariable( max );
		}
		if( NX3 > 0 ) {
			rhs3.setGlobalExportVariable( max );
			diffs_rhs3.setGlobalExportVariable( max );
		}
		for( uint i = 0; i < outputs.size(); i++ ) {
			outputs[i].setGlobalExportVariable( max );
			diffs_outputs[i].setGlobalExportVariable( max );
		}

		getDataDeclarations( code, ACADO_LOCAL );

		stringstream s;
		s << "#pragma omp threadprivate( "
				<< max.getFullName().getName() << ", "
				<< rk_ttt.getFullName().getName() << ", "
				<< rk_xxx.getFullName().getName() << ", "
				<< rk_kkk.getFullName().getName() << ", "
				<< rk_diffK.getFullName().getName() << ", "
				<< rk_rhsTemp.getFullName().getName() << ", "
				<< rk_auxSolver.getFullName().getName();
		if( NX1 > 0 ) {
			if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) s << ", " << rk_diffsPrev1.getFullName().getName();
			s << ", " << rk_diffsNew1.getFullName().getName();
		}
		if( NX2 > 0 || NXA > 0 ) {
			s << ", " << rk_A.getFullName().getName();
			s << ", " << rk_b.getFullName().getName();
			if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) s << ", " << rk_diffsPrev2.getFullName().getName();
			s << ", " << rk_diffsNew2.getFullName().getName();
			s << ", " << rk_diffsTemp2.getFullName().getName();
			solver->appendVariableNames( s );
		}
		if( NX3 > 0 ) {
			if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) s << ", " << rk_diffsPrev3.getFullName().getName();
			s << ", " << rk_diffsNew3.getFullName().getName();
			s << ", " << rk_diffsTemp3.getFullName().getName();
		}
		s << " )" << endl << endl;
		code.addStatement( s.str().c_str() );
	}

	if( NX1 > 0 ) {
		code.addFunction( lin_input );
		code.addStatement( "\n\n" );
	}
	if( exportRhs ) {
		if( NX2 > 0 || NXA > 0 ) {
			code.addFunction( rhs );
			code.addStatement( "\n\n" );
			code.addFunction( diffs_rhs );
			code.addStatement( "\n\n" );
		}

		if( NX3 > 0 ) {
			code.addFunction( rhs3 );
			code.addStatement( "\n\n" );
			code.addFunction( diffs_rhs3 );
			code.addStatement( "\n\n" );
		}

		if( CONTINUOUS_OUTPUT ) {
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
	uint run5;
	String tempString;
	
	initializeButcherTableau();
	initializeDDMatrix();
	initializeCoefficients();

	ExportVariable Ah;
	ExportVariable Bh;
	ExportVariable rk_tPrev;
	if( equidistant ) {
		double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
		Matrix tmp = AA;
		Ah = ExportVariable( "Ah_mat", tmp*=h, STATIC_CONST_REAL );
		code.addDeclaration( Ah );
		code.addLinebreak( 2 );
		// TODO: Ask Milan why this does NOT work properly !!
		Ah = ExportVariable( "Ah_mat", numStages, numStages, STATIC_CONST_REAL, ACADO_LOCAL );

		Vector BB( bb );
		Bh = ExportVariable( "Bh_mat", Matrix( BB*=h, BT_FALSE ) );
	}
	else {
		if( NX1 > 0 || NX3 > 0 ) {
			// TODO: WHAT IF NONEQUIDISTANT INTEGRATION GRID??
			return RET_UNABLE_TO_EXPORT_CODE;
		}

		Ah = ExportVariable( "Ah_mat", numStages, numStages, REAL, ACADO_LOCAL );
		Bh = ExportVariable( "Bh_mat", numStages, 1, REAL, ACADO_LOCAL );
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
	ExportVariable determinant( "det", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );
	integrate.addDeclaration( determinant );

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

	prepareOutputEvaluation( code );

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
		for( run5 = 0; run5 < numStages; run5++ ) {
			loop->addStatement( Bh.getRow(run5) == stepsH.getCol( run )*Matrix(bb).getRow(run5) );
		}
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

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		// Set rk_diffsPrev:
		loop->addStatement( String("if( run > 0 ) {\n") );
		if( NX1 > 0 ) {
			ExportForLoop loopTemp1( i,0,NX1 );
			loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,0,NX1 ) == rk_eta.getCols( i*NX+NX+NXA,i*NX+NX+NXA+NX1 ) );
			if( NU > 0 ) loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,NX1,NX1+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1),i*NU+(NX+NXA)*(NX+1)+NU ) );
			loop->addStatement( loopTemp1 );
		}
		if( NX2 > 0 ) {
			ExportForLoop loopTemp2( i,0,NX2 );
			loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,0,NX1+NX2 ) == rk_eta.getCols( i*NX+NX+NXA+NX1*NX,i*NX+NX+NXA+NX1*NX+NX1+NX2 ) );
			if( NU > 0 ) loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,NX1+NX2,NX1+NX2+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+NX1*NU,i*NU+(NX+NXA)*(NX+1)+NX1*NU+NU ) );
			loop->addStatement( loopTemp2 );
		}
		if( NX3 > 0 ) {
			ExportForLoop loopTemp3( i,0,NX3 );
			loopTemp3.addStatement( rk_diffsPrev3.getSubMatrix( i,i+1,0,NX ) == rk_eta.getCols( i*NX+NX+NXA+(NX1+NX2)*NX,i*NX+NX+NXA+(NX1+NX2)*NX+NX ) );
			if( NU > 0 ) loopTemp3.addStatement( rk_diffsPrev3.getSubMatrix( i,i+1,NX,NX+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+(NX1+NX2)*NU,i*NU+(NX+NXA)*(NX+1)+(NX1+NX2)*NU+NU ) );
			loop->addStatement( loopTemp3 );
		}
		loop->addStatement( String("}\n") );
	}

	// PART 1: The linear input system
	prepareInputSystem( code );
	solveInputSystem( loop, i, run1, j, tmp_index1, Ah );

	// PART 2: The fully implicit system
	solveImplicitSystem( loop, i, run1, j, tmp_index1, Ah, determinant );

	// PART 3: The linear output system
	prepareOutputSystem( code );
	solveOutputSystem( loop, i, run1, j, tmp_index1, Ah );

	// generate continuous OUTPUT:
	generateOutput( loop, run, i, tmp_index2, tmp_index3, tmp_meas, rk_tPrev, time_tmp );

	// DERIVATIVES wrt the states (IFT):
	if( NX1 > 0 ) {
		ExportForLoop loop4( run1,0,NX1 );
		// PART 1: The linear input system
		sensitivitiesInputSystem( &loop4, run1, i, Bh, BT_TRUE );
		// PART 2: The fully implicit system
		sensitivitiesImplicitSystem( &loop4, run1, i, j, tmp_index1, tmp_index2, Ah, Bh, determinant, BT_TRUE, 1 );
		// PART 3: The linear output system
		sensitivitiesOutputSystem( &loop4, run1, i, j, k, tmp_index1, tmp_index2, Ah, Bh, BT_TRUE, 1 );
		// generate sensitivities wrt states for continuous output:
		sensitivitiesOutputs( &loop4, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, rk_tPrev, time_tmp, BT_TRUE, 0 );
		loop->addStatement( loop4 );
	}
	if( NX2 > 0 ) {
		ExportForLoop loop4( run1,NX1,NX1+NX2 );
		// PART 2: The fully implicit system
		sensitivitiesImplicitSystem( &loop4, run1, i, j, tmp_index1, tmp_index2, Ah, Bh, determinant, BT_TRUE, 2 );
		// PART 3: The linear output system
		sensitivitiesOutputSystem( &loop4, run1, i, j, k, tmp_index1, tmp_index2, Ah, Bh, BT_TRUE, 2 );
		// generate sensitivities wrt states for continuous output:
		sensitivitiesOutputs( &loop4, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, rk_tPrev, time_tmp, BT_TRUE, NX1 );
		loop->addStatement( loop4 );
	}
	if( NX3 > 0 ) {
		ExportForLoop loop4( run1,NX1+NX2,NX );
		// PART 3: The linear output system
		sensitivitiesOutputSystem( &loop4, run1, i, j, k, tmp_index1, tmp_index2, Ah, Bh, BT_TRUE, 3 );
		// generate sensitivities wrt states for continuous output:
		sensitivitiesOutputs( &loop4, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, rk_tPrev, time_tmp, BT_TRUE, NX1+NX2 );
		loop->addStatement( loop4 );
	}


	// DERIVATIVES wrt the control inputs (IFT):
	if( NU > 0 ) {
		ExportForLoop loop5( run1,0,NU );
		// PART 1: The linear input system
		sensitivitiesInputSystem( &loop5, run1, i, Bh, BT_FALSE );
		// PART 2: The fully implicit system
		sensitivitiesImplicitSystem( &loop5, run1, i, j, tmp_index1, tmp_index2, Ah, Bh, determinant, BT_FALSE, 0 );
		// PART 3: The linear output system
		sensitivitiesOutputSystem( &loop5, run1, i, j, k, tmp_index1, tmp_index2, Ah, Bh, BT_FALSE, 0 );
		// generate sensitivities wrt controls for continuous output:
		sensitivitiesOutputs( &loop5, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, rk_tPrev, time_tmp, BT_FALSE, 0 );
		loop->addStatement( loop5 );
	}

	// update rk_eta:
	for( run5 = 0; run5 < NX; run5++ ) {
		loop->addStatement( rk_eta.getCol( run5 ) += rk_kkk.getRow( run5 )*Bh );
	}
	if( NXA > 0) {
		Matrix tempCoefs( evaluateDerivedPolynomial( 0.0 ), BT_FALSE );
		loop->addStatement( String("if( run == 0 ) {\n") );
		for( run5 = 0; run5 < NXA; run5++ ) {
			loop->addStatement( rk_eta.getCol( NX+run5 ) == rk_kkk.getRow( NX+run5 )*tempCoefs );
		}
		loop->addStatement( String("}\n") );
	}


	// Computation of the sensitivities using the CHAIN RULE:
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( String( "if( run == 0 ) {\n" ) );
	}
	// PART 1
	updateInputSystem(loop, i, j, tmp_index2);
	// PART 2
	updateImplicitSystem(loop, i, j, tmp_index2);
	// PART 3
	updateOutputSystem(loop, i, j, tmp_index2);

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( String( "}\n" ) );
		loop->addStatement( String( "else {\n" ) );
		// PART 1
		propagateInputSystem(loop, i, j, k, tmp_index2);
		// PART 2
		propagateImplicitSystem(loop, i, j, k, tmp_index2);
		// PART 3
		propagateOutputSystem(loop, i, j, k, tmp_index2);
	}

	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !equidistantControlGrid()) ) {
		propagateOutputs( loop, run, run1, i, j, k, tmp_index1, tmp_index2, tmp_index3, tmp_index4, tmp_meas );
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
    // PART 1
    if( NX1 > 0 ) {
    	Matrix zeroR = zeros(1, NX2+NX3);
    	ExportForLoop loop1( i,0,NX1 );
    	loop1.addStatement( rk_eta.getCols( i*NX+NX+NXA+NX1,i*NX+NX+NXA+NX ) == zeroR );
    	integrate.addStatement( loop1 );
    }
    // PART 2
    Matrix zeroR = zeros(1, NX3);
    if( NX2 > 0 ) {
    	ExportForLoop loop2( i,NX1,NX1+NX2 );
    	loop2.addStatement( rk_eta.getCols( i*NX+NX+NXA+NX1+NX2,i*NX+NX+NXA+NX ) == zeroR );
    	integrate.addStatement( loop2 );
    }
    if( NXA > 0 ) {
    	ExportForLoop loop3( i,NX,NX+NXA );
    	loop3.addStatement( rk_eta.getCols( i*NX+NX+NXA+NX1+NX2,i*NX+NX+NXA+NX ) == zeroR );
    	integrate.addStatement( loop3 );
    }

    integrate.addStatement( String( "if( " ) << determinant.getFullName() << " < 1e-12 ) {\n" );
    integrate.addStatement( error_code == 2 );
    integrate.addStatement( String( "} else if( " ) << determinant.getFullName() << " < 1e-6 ) {\n" );
    integrate.addStatement( error_code == 1 );
    integrate.addStatement( String( "} else {\n" ) );
    integrate.addStatement( error_code == 0 );
    integrate.addStatement( String( "}\n" ) );

	code.addFunction( integrate );
    code.addLinebreak( 2 );

	if( NX2 > 0 || NXA > 0 ) solver->getCode( code );
	if( NX2 != NX ) {
		prepareFullRhs();
		code.addFunction( fullRhs );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::updateOutputSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
{
	if( NX3 > 0 ) {
		ExportForLoop loop01( index1,NX1+NX2,NX );
		ExportForLoop loop02( index2,0,NX );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX+index2,NX+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::propagateOutputSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2,
																const ExportIndex& index3, const ExportIndex& tmp_index )
{
	if( NX3 > 0 ) {
		ExportForLoop loop01( index1,NX1+NX2,NX );
		ExportForLoop loop02( index2,0,NX1 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		ExportForLoop loop03( index3,0,NX1 );
		loop03.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop03 );
		ExportForLoop loop04( index3,0,NX2 );
		loop04.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop04 );
		ExportForLoop loop042( index3,0,NX3 );
		loop042.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+NX2+index3,NX1+NX2+index3+1 )*rk_diffsPrev3.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop02.addStatement( loop042 );
		loop01.addStatement( loop02 );

		ExportForLoop loop05( index2,NX1,NX1+NX2 );
		loop05.addStatement( tmp_index == index2+index1*NX );
		loop05.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		ExportForLoop loop06( index3,0,NX2 );
		loop06.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop05.addStatement( loop06 );
		ExportForLoop loop062( index3,0,NX3 );
		loop062.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+NX2+index3,NX1+NX2+index3+1 )*rk_diffsPrev3.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop05.addStatement( loop062 );
		loop01.addStatement( loop05 );

		ExportForLoop loop07( index2,NX1+NX2,NX );
		loop07.addStatement( tmp_index == index2+index1*NX );
		loop07.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) == 0.0 );
		ExportForLoop loop08( index3,0,NX3 );
		loop08.addStatement( rk_eta.getCol( tmp_index+NX+NXA ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+NX2+index3,NX1+NX2+index3+1 )*rk_diffsPrev3.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop07.addStatement( loop08 );
		loop01.addStatement( loop07 );

		if( NU > 0 ) {
			ExportForLoop loop09( index2,0,NU );
			loop09.addStatement( tmp_index == index2+index1*NU );
			loop09.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) == rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX+index2,NX+index2+1 ) );
			ExportForLoop loop10( index3,0,NX1 );
			loop10.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,NX1+index2,NX1+index2+1 ) );
			loop09.addStatement( loop10 );
			ExportForLoop loop11( index3,0,NX2 );
			loop11.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop09.addStatement( loop11 );
			ExportForLoop loop112( index3,0,NX3 );
			loop112.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX) ) += rk_diffsNew3.getSubMatrix( index1-NX1-NX2,index1-NX1-NX2+1,NX1+NX2+index3,NX1+NX2+index3+1 )*rk_diffsPrev3.getSubMatrix( index3,index3+1,NX+index2,NX+index2+1 ) );
			loop09.addStatement( loop112 );
			loop01.addStatement( loop09 );
		}
		block->addStatement( loop01 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::propagateOutputs(	ExportStatementBlock* block, const ExportIndex& index, const ExportIndex& index0, const ExportIndex& index1,
															const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
															const ExportIndex& tmp_index3, const ExportIndex& tmp_index4, const ExportVariable& tmp_meas )
{
	uint i;
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	// chain rule for the sensitivities of the continuous output:
	for( i = 0; i < rk_outputs.size(); i++ ) {
		ExportStatementBlock *loop01;
		ExportForLoop tmpLoop( index0,0,totalMeas[i] );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) loop01 = &tmpLoop;
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop01 = block;
			loop01->addStatement( String("for(") << index0.getName() << " = 0; " << index0.getName() << " < (int)" << numMeasVariables[i].get(0,index) << "; " << index0.getName() << "++) {\n" );
		}
		else { // ONLINE_GRID
			loop01 = block;
			loop01->addStatement( String("for(") << index0.getName() << " = 0; " << index0.getName() << " < (int)" << tmp_meas.get(0,i) << "; " << index0.getName() << "++) {\n" );
		}

		uint numOutputs = getDimOUTPUT( i );
		uint outputDim = numOutputs*(NX+NU+1);
		loop01->addStatement( tmp_index1 == numMeas[i]*outputDim+index0*(numOutputs*(NX+NU+1)) );
		ExportForLoop loop02( index3,0,numOutputs*(NX+NU) );
		loop02.addStatement( tmp_index2 == tmp_index1+index3 );
		loop02.addStatement( rk_diffsOutputTemp.getCol( index3 ) == rk_outputs[i].getCol( tmp_index2+numOutputs ) );
		loop01->addStatement( loop02 );

		loop01->addStatement( tmp_index1 == numMeas[i]*outputDim+index0*(numOutputs*(NX+NU+1)) );
		ExportForLoop loop03( index1,0,numOutputs );
		loop03.addStatement( tmp_index2 == tmp_index1+index1*NX );

		ExportForLoop loop04( index2,0,NX1 );
		loop04.addStatement( tmp_index3 == tmp_index2+index2 );
		loop04.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) == 0.0 );
		ExportForLoop loop05( index3,0,NX1 );
		loop05.addStatement( tmp_index4 == index1*NX+index3 );
		loop05.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		loop04.addStatement( loop05 );
		ExportForLoop loop06( index3,NX1,NX1+NX2 );
		loop06.addStatement( tmp_index4 == index1*NX+index3 );
		loop06.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev2.getSubMatrix( index3-NX1,index3-NX1+1,index2,index2+1 ) );
		loop04.addStatement( loop06 );
		ExportForLoop loop062( index3,NX1+NX2,NX );
		loop062.addStatement( tmp_index4 == index1*NX+index3 );
		loop062.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev3.getSubMatrix( index3-NX1-NX2,index3-NX1-NX2+1,index2,index2+1 ) );
		loop04.addStatement( loop062 );
		loop03.addStatement( loop04 );

		ExportForLoop loop07( index2,NX1,NX1+NX2 );
		loop07.addStatement( tmp_index3 == tmp_index2+index2 );
		loop07.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) == 0.0 );
		ExportForLoop loop08( index3,NX1,NX1+NX2 );
		loop08.addStatement( tmp_index4 == index1*NX+index3 );
		loop08.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev2.getSubMatrix( index3-NX1,index3-NX1+1,index2,index2+1 ) );
		loop07.addStatement( loop08 );
		ExportForLoop loop082( index3,NX1+NX2,NX );
		loop082.addStatement( tmp_index4 == index1*NX+index3 );
		loop082.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev3.getSubMatrix( index3-NX1-NX2,index3-NX1-NX2+1,index2,index2+1 ) );
		loop07.addStatement( loop082 );
		loop03.addStatement( loop07 );

		ExportForLoop loop09( index2,NX1+NX2,NX );
		loop09.addStatement( tmp_index3 == tmp_index2+index2 );
		loop09.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) == 0.0 );
		ExportForLoop loop10( index3,NX1+NX2,NX );
		loop10.addStatement( tmp_index4 == index1*NX+index3 );
		loop10.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev3.getSubMatrix( index3-NX1-NX2,index3-NX1-NX2+1,index2,index2+1 ) );
		loop09.addStatement( loop10 );
		loop03.addStatement( loop09 );

		if( NU > 0 ) {
			loop03.addStatement( tmp_index2 == tmp_index1+index1*NU );
			ExportForLoop loop11( index2,0,NU );
			loop11.addStatement( tmp_index3 == tmp_index2+index2 );
			loop11.addStatement( tmp_index4 == index1*NU+index2 );
			loop11.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs*(1+NX) ) == rk_diffsOutputTemp.getCol( tmp_index4+numOutputs*NX ) );
			ExportForLoop loop12( index3,0,NX1 );
			loop12.addStatement( tmp_index4 == index1*NX+index3 );
			loop12.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs*(1+NX) ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,NX1+index2,NX1+index2+1 ) );
			loop11.addStatement( loop12 );
			ExportForLoop loop13( index3,NX1,NX1+NX2 );
			loop13.addStatement( tmp_index4 == index1*NX+index3 );
			loop13.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs*(1+NX) ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev2.getSubMatrix( index3-NX1,index3-NX1+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop11.addStatement( loop13 );
			ExportForLoop loop132( index3,NX1+NX2,NX );
			loop132.addStatement( tmp_index4 == index1*NX+index3 );
			loop132.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs*(1+NX) ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev3.getSubMatrix( index3-NX1-NX2,index3-NX1-NX2+1,NX+index2,NX+index2+1 ) );
			loop11.addStatement( loop132 );
			loop03.addStatement( loop11 );
		}

		loop01->addStatement( loop03 );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			block->addStatement( *loop01 );
		}
		else {
			loop01->addStatement( "}\n" );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::prepareInputSystem(	ExportStatementBlock& code )
{
	if( NX1 > 0 ) {
		Matrix mat1 = formMatrix( M11, A11 );
		rk_mat1 = ExportVariable( "rk_mat1", mat1, STATIC_CONST_REAL );
		code.addDeclaration( rk_mat1 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_mat1 = ExportVariable( "rk_mat1", numStages*NX1, numStages*NX1, STATIC_CONST_REAL, ACADO_LOCAL );

		Matrix sens = zeros(NX1*(NX1+NU), numStages);
		uint i, j, k;
		for( i = 0; i < NX1; i++ ) {
			Vector vec(NX1*numStages);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					vec(j*NX1+k) = A11(k,i);
				}
			}
			Vector sol = mat1*vec;
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					sens(i*NX1+k,j) = sol(j*NX1+k);
				}
			}
		}
		for( i = 0; i < NU; i++ ) {
			Vector vec(NX1*numStages);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					vec(j*NX1+k) = B11(k,i);
				}
			}
			Vector sol = mat1*vec;
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					sens(NX1*NX1+i*NX1+k,j) = sol(j*NX1+k);
				}
			}
		}
		rk_dk1 = ExportVariable( "rk_dk1", sens, STATIC_CONST_REAL );
		code.addDeclaration( rk_dk1 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_dk1 = ExportVariable( "rk_dk1", NX1*(NX1+NU), numStages, STATIC_CONST_REAL, ACADO_LOCAL );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::prepareOutputSystem(	ExportStatementBlock& code )
{
	if( NX3 > 0 ) {
		Matrix mat3 = formMatrix( M33, A33 );
		rk_mat3 = ExportVariable( "rk_mat3", mat3, STATIC_CONST_REAL );
		code.addDeclaration( rk_mat3 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_mat3 = ExportVariable( "rk_mat3", numStages*NX3, numStages*NX3, STATIC_CONST_REAL, ACADO_LOCAL );

		Matrix sens = zeros(NX3*NX3, numStages);
		uint i, j, k;
		for( i = 0; i < NX3; i++ ) {
			Vector vec(NX3*numStages);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX3; k++ ) {
					vec(j*NX3+k) = A33(k,i);
				}
			}
			Vector sol = mat3*vec;
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX3; k++ ) {
					sens(i*NX3+k,j) = sol(j*NX3+k);
				}
			}
		}
		rk_dk3 = ExportVariable( "rk_dk3", sens, STATIC_CONST_REAL );
		code.addDeclaration( rk_dk3 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_dk3 = ExportVariable( "rk_dk3", NX3*NX3, numStages, STATIC_CONST_REAL, ACADO_LOCAL );
	}

	return SUCCESSFUL_RETURN;
}


Matrix ImplicitRungeKuttaExport::formMatrix( const Matrix& mass, const Matrix& jacobian ) {
	if( jacobian.getNumRows() != jacobian.getNumCols() ) {
		return RET_UNABLE_TO_EXPORT_CODE;
	}
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	uint vars = jacobian.getNumRows();
	uint i1, j1, i2, j2;
	Matrix result = zeros(numStages*vars, numStages*vars);
	for( i1 = 0; i1 < numStages; i1++ ){
		for( j1 = 0; j1 < numStages; j1++ ) {
			for( i2 = 0; i2 < vars; i2++ ){
				for( j2 = 0; j2 < vars; j2++ ) {
					if( i1 == j1 ) {
						result(i1*vars+i2, j1*vars+j2) = mass(i2,j2) - AA(i1,j1)*h*jacobian(i2,j2);
					}
					else {
						result(i1*vars+i2, j1*vars+j2) = -AA(i1,j1)*h*jacobian(i2,j2);
					}
				}
			}
		}
	}

	return result.getInverse();
}


Matrix ImplicitRungeKuttaExport::expandOutputMatrix( const Matrix& A3 ) {
	Matrix result = zeros(NX3,NX);
	uint i,j;
	for( i = 0; i < NX3; i++ ) {
		for( j = NX1+NX2; j < NX; j++ ) {
			result(i,j) = A3(i,j-NX1-NX2);
		}
	}

	return result;
}


returnValue ImplicitRungeKuttaExport::solveInputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportVariable& Ah )
{
	if( NX1 > 0 ) {
		block->addStatement( rk_xxx.getCols(0,NX1) == rk_eta.getCols(0,NX1) );
		block->addFunctionCall( lin_input.getName(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
		ExportForLoop loop( index1,0,numStages );
		ExportForLoop loop1( index2,0,NX1 );
		loop1.addStatement( tmp_index == index1*NX1+index2 );
		loop1.addStatement( rk_b.getRow(tmp_index) == rk_rhsTemp.getRow(index2) );
		loop.addStatement(loop1);
		block->addStatement(loop);

		ExportForLoop loop4( index1,0,numStages );
		ExportForLoop loop5( index2,0,NX1 );
		loop5.addStatement( tmp_index == index1*NX1+index2 );
		loop5.addStatement( rk_kkk.getSubMatrix(index2,index2+1,index1,index1+1) == rk_mat1.getSubMatrix(tmp_index,tmp_index+1,0,1)*rk_b.getRow(0) );
		ExportForLoop loop6( index3,1,numStages*NX1 );
		loop6.addStatement( rk_kkk.getSubMatrix(index2,index2+1,index1,index1+1) += rk_mat1.getSubMatrix(tmp_index,tmp_index+1,index3,index3+1)*rk_b.getRow(index3) );
		loop5.addStatement(loop6);
		loop4.addStatement(loop5);
		block->addStatement(loop4);
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::solveImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportVariable& Ah, const ExportVariable& det )
{
	if( NX2 > 0 || NXA > 0 ) {

		if( REUSE ) block->addStatement( String( "if( " ) << reset_int.getFullName() << " ) {\n" );
		// Initialization iterations:
		ExportForLoop loop1( index1,0,numItsInit+1 ); // NOTE: +1 because 0 will lead to NaNs, so the minimum number of iterations is 1 at the initialization
		ExportForLoop loop11( index2,0,numStages );
		evaluateMatrix( &loop11, index2, index3, tmp_index, Ah, BT_TRUE );
		loop1.addStatement( loop11 );
		loop1.addStatement( det.getFullName() << " = " << solver->getNameSolveFunction() << "( " << rk_A.getFullName() << ", " << rk_b.getFullName() << ", " << rk_auxSolver.getFullName() << " );\n" );
		ExportForLoop loopTemp( index3,0,numStages );
		loopTemp.addStatement( rk_kkk.getSubMatrix( NX1,NX1+NX2,index3,index3+1 ) += rk_b.getRows( index3*NX2,index3*NX2+NX2 ) );											// differential states
		if(NXA > 0) loopTemp.addStatement( rk_kkk.getSubMatrix( NX,NX+NXA,index3,index3+1 ) += rk_b.getRows( index3*NXA+numStages*NX2,index3*NXA+numStages*NX2+NXA ) );		// algebraic states
		loop1.addStatement( loopTemp );
		block->addStatement( loop1 );
		if( REUSE ) block->addStatement( String( "}\n" ) );

		// the rest (numIts) of the Newton iterations with reuse of the Jacobian (no evaluation or factorization needed)
		ExportForLoop loop2( index1,0,numIts );
		ExportForLoop loop21( index2,0,numStages );
		evaluateStatesImplicitSystem( &loop21, Ah, index2, index3, tmp_index );
		evaluateRhsImplicitSystem( &loop21, index2 );
		loop2.addStatement( loop21 );
		loop2.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		loopTemp = ExportForLoop( index3,0,numStages );
		loopTemp.addStatement( rk_kkk.getSubMatrix( NX1,NX1+NX2,index3,index3+1 ) += rk_b.getRows( index3*NX2,index3*NX2+NX2 ) );														// differential states
		if(NXA > 0) loopTemp.addStatement( rk_kkk.getSubMatrix( NX,NX+NXA,index3,index3+1 ) += rk_b.getRows( index3*NXA+numStages*NX2,index3*NXA+numStages*NX2+NXA ) );		// algebraic states
		loop2.addStatement( loopTemp );
		block->addStatement( loop2 );

		// solution calculated --> evaluate and save the necessary derivatives in rk_diffsTemp and update the matrix rk_A:
		ExportForLoop loop3( index2,0,numStages );
		evaluateMatrix( &loop3, index2, index3, tmp_index, Ah, BT_FALSE );
		block->addStatement( loop3 );

		// IF DEBUG MODE:
		int debugMode;
		get( INTEGRATOR_DEBUG_MODE, debugMode );
		if ( (BooleanType)debugMode == BT_TRUE ) {
			block->addStatement( debug_mat == rk_A );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::solveOutputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportVariable& Ah )
{
	if( NX3 > 0 ) {
		ExportForLoop loop( index1,0,numStages );
		evaluateStatesOutputSystem( &loop, Ah, index1 );
		loop.addFunctionCall( getNameOutputRHS(), rk_xxx, rk_b.getAddress(index1*NX3,0) );
		loop.addFunctionCall( getNameOutputDiffs(), rk_xxx, rk_diffsTemp3.getAddress(index1,0) );
		block->addStatement( loop );

		ExportForLoop loop4( index1,0,numStages );
		ExportForLoop loop5( index2,0,NX3 );
		loop5.addStatement( tmp_index == index1*NX3+index2 );
		loop5.addStatement( rk_kkk.getSubMatrix(NX1+NX2+index2,NX1+NX2+index2+1,index1,index1+1) == rk_mat3.getSubMatrix(tmp_index,tmp_index+1,0,1)*rk_b.getRow(0) );
		ExportForLoop loop6( index3,1,numStages*NX3 );
		loop6.addStatement( rk_kkk.getSubMatrix(NX1+NX2+index2,NX1+NX2+index2+1,index1,index1+1) += rk_mat3.getSubMatrix(tmp_index,tmp_index+1,index3,index3+1)*rk_b.getRow(index3) );
		loop5.addStatement(loop6);
		loop4.addStatement(loop5);
		block->addStatement(loop4);
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::sensitivitiesInputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportVariable& Bh, BooleanType STATES )
{
	if( NX1 > 0 ) {
		// update rk_diffK with the new sensitivities:
		if( STATES ) 	block->addStatement( rk_diffK.getRows(0,NX1) == rk_dk1.getRows(index1*NX1,index1*NX1+NX1) );
		else			block->addStatement( rk_diffK.getRows(0,NX1) == rk_dk1.getRows(index1*NX1+NX1*NX1,index1*NX1+NX1+NX1*NX1) );
		// update rk_diffsNew with the new sensitivities:
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( String( "if( run == 0 ) {\n" ) );
		ExportForLoop loop3( index2,0,NX1 );
		if( STATES ) loop3.addStatement( String(rk_diffsNew1.get( index2,index1 )) << " = (" << index2.getName() << " == " << index1.getName() << ");\n" );

		if( STATES ) loop3.addStatement( rk_diffsNew1.getSubMatrix( index2,index2+1,index1,index1+1 ) += rk_diffK.getRow( index2 )*Bh );
		else		 loop3.addStatement( rk_diffsNew1.getSubMatrix( index2,index2+1,index1+NX1,index1+NX1+1 ) == rk_diffK.getRow( index2 )*Bh );
		block->addStatement( loop3 );
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( String( "}\n" ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::sensitivitiesImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportVariable& Ah, const ExportVariable& Bh, const ExportVariable& det, BooleanType STATES, uint number )
{
	if( NX2 > 0 ) {
		Matrix zeroM = zeros( NX2+NXA,1 );
		Matrix tempCoefs( evaluateDerivedPolynomial( 0.0 ), BT_FALSE );
		uint i;

		ExportForLoop loop1( index2,0,numStages );
		if( STATES && number == 1 ) {
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( String(rk_rhsTemp.get( index3,0 )) << " = -(" << index3.getName() << " == " << index1.getName() << ");\n" );
			for( i = 0; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) -= rk_diffK.getSubMatrix( index3,index3+1,i,i+1 )*Ah.getSubMatrix(index2,index2+1,i,i+1) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,0,NX2+NXA );
			loop3.addStatement( tmp_index1 == index2*(NX2+NXA)+index3 );
			loop3.addStatement( rk_b.getRow( tmp_index1 ) == rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2),index3*(NVARS2)+NX1 )*rk_rhsTemp.getRows(0,NX1) );
			if( NDX2 > 0 ) {
				loop3.addStatement( rk_b.getRow( tmp_index1 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2)+NVARS2-NX1-NX2,index3*(NVARS2)+NVARS2-NX2 )*rk_diffK.getSubMatrix( 0,NX1,index2,index2+1 ) );
			}
			loop1.addStatement( loop3 );
		}
		else if( STATES && number == 2 ) {
			for( i = 0; i < NX2+NXA; i++ ) {
				loop1.addStatement( rk_b.getRow( index2*(NX2+NXA)+i ) == zeroM.getRow( 0 ) - rk_diffsTemp2.getSubMatrix( index2,index2+1,index1+i*(NVARS2),index1+i*(NVARS2)+1 ) );
			}
		}
		else { // ~= STATES
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getSubMatrix( index3,index3+1,0,1 )*Ah.getSubMatrix(index2,index2+1,0,1) );
			for( i = 1; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getSubMatrix( index3,index3+1,i,i+1 )*Ah.getSubMatrix(index2,index2+1,i,i+1) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,0,NX2+NXA );
			loop3.addStatement( tmp_index1 == index2*(NX2+NXA)+index3 );
			loop3.addStatement( tmp_index2 == index1+index3*(NVARS2) );
			loop3.addStatement( rk_b.getRow( tmp_index1 ) == zeroM.getRow( 0 ) - rk_diffsTemp2.getSubMatrix( index2,index2+1,tmp_index2+NX1+NX2+NXA,tmp_index2+NX1+NX2+NXA+1 ) );
			loop3.addStatement( rk_b.getRow( tmp_index1 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2),index3*(NVARS2)+NX1 )*rk_rhsTemp.getRows(0,NX1) );
			if( NDX2 > 0 ) {
				loop3.addStatement( rk_b.getRow( tmp_index1 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2)+NVARS2-NX1-NX2,index3*(NVARS2)+NVARS2-NX2 )*rk_diffK.getSubMatrix( 0,NX1,index2,index2+1 ) );
			}
			loop1.addStatement( loop3 );
		}
		block->addStatement( loop1 );
		if( STATES && (number == 1 || NX1 == 0) ) {
			block->addStatement( String( "if( 0 == " ) << index1.getName() << " ) {\n" );	// factorization of the new matrix rk_A not yet calculated!
			block->addStatement( det.getFullName() << " = " << solver->getNameSolveFunction() << "( " << rk_A.getFullName() << ", " << rk_b.getFullName() << ", " << rk_auxSolver.getFullName() << " );\n" );
			block->addStatement( String( "}\n else {\n" ) );
		}
		block->addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		if( STATES && (number == 1 || NX1 == 0) ) block->addStatement( String( "}\n" ) );
		// update rk_diffK with the new sensitivities:
		ExportForLoop loop2( index2,0,numStages );
		loop2.addStatement( rk_diffK.getSubMatrix(NX1,NX1+NX2,index2,index2+1) == rk_b.getRows(index2*NX2,index2*NX2+NX2) );
		loop2.addStatement( rk_diffK.getSubMatrix(NX,NX+NXA,index2,index2+1) == rk_b.getRows(numStages*NX2+index2*NXA,numStages*NX2+index2*NXA+NXA) );
		block->addStatement( loop2 );
		// update rk_diffsNew with the new sensitivities:
		ExportForLoop loop3( index2,0,NX2 );
		if( STATES && number == 2 ) loop3.addStatement( String(rk_diffsNew2.get( index2,index1 )) << " = (" << index2.getName() << " == " << index1.getName() << "-" << String(NX1) << ");\n" );

		if( STATES && number == 2 ) loop3.addStatement( rk_diffsNew2.getSubMatrix( index2,index2+1,index1,index1+1 ) += rk_diffK.getRow( NX1+index2 )*Bh );
		else if( STATES )	loop3.addStatement( rk_diffsNew2.getSubMatrix( index2,index2+1,index1,index1+1 ) == rk_diffK.getRow( NX1+index2 )*Bh );
		else		 		loop3.addStatement( rk_diffsNew2.getSubMatrix( index2,index2+1,index1+NX1+NX2,index1+NX1+NX2+1 ) == rk_diffK.getRow( NX1+index2 )*Bh );
		block->addStatement( loop3 );
		if( NXA > 0 ) {
			block->addStatement( String("if( run == 0 ) {\n") );
			ExportForLoop loop4( index2,0,NXA );
			if( STATES ) loop4.addStatement( rk_diffsNew2.getSubMatrix( index2+NX2,index2+NX2+1,index1,index1+1 ) == rk_diffK.getRow( NX+index2 )*tempCoefs );
			else 		 loop4.addStatement( rk_diffsNew2.getSubMatrix( index2+NX2,index2+NX2+1,index1+NX1+NX2,index1+NX1+NX2+1 ) == rk_diffK.getRow( NX+index2 )*tempCoefs );
			block->addStatement( loop4 );
			block->addStatement( String("}\n") );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::sensitivitiesOutputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& index4, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportVariable& Ah, const ExportVariable& Bh, BooleanType STATES, uint number )
{
	if( NX3 > 0 ) {
		uint i;
		ExportForLoop loop1( index2,0,numStages );
		if( STATES && number == 1 ) {
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( String(rk_rhsTemp.get( index3,0 )) << " = (" << index3.getName() << " == " << index1.getName() << ");\n" );
			for( i = 0; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getSubMatrix( index3,index3+1,i,i+1 )*Ah.getSubMatrix(index2,index2+1,i,i+1) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,NX1,NX1+NX2 );
			loop3.addStatement( rk_rhsTemp.getRow( index3 ) == 0.0 );
			for( i = 0; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getSubMatrix( index3,index3+1,i,i+1 )*Ah.getSubMatrix(index2,index2+1,i,i+1) );
			}
			loop1.addStatement( loop3 );
			ExportForLoop loop4( index3,0,NX3 );
			loop4.addStatement( tmp_index1 == index2*NX3+index3 );
			loop4.addStatement( rk_b.getRow( tmp_index1 ) == rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3),index3*(NVARS3)+NX1+NX2 )*rk_rhsTemp.getRows(0,NX1+NX2) );
			if( NXA3 > 0 ) {
				loop4.addStatement( rk_b.getRow( tmp_index1 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NX1+NX2,index3*(NVARS3)+NX1+NX2+NXA )*rk_diffK.getSubMatrix( NX,NX+NXA,index2,index2+1 ) );
			}
			if( NDX3 > 0 ) {
				loop4.addStatement( rk_b.getRow( tmp_index1 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NVARS3-NX1-NX2,index3*(NVARS3)+NVARS3 )*rk_diffK.getSubMatrix( 0,NX1+NX2,index2,index2+1 ) );
			}
			loop1.addStatement( loop4 );
		}
		else if( STATES && number == 2 ) {
			ExportForLoop loop3( index3,NX1,NX1+NX2 );
			loop3.addStatement( String(rk_rhsTemp.get( index3,0 )) << " = (" << index3.getName() << " == " << index1.getName() << ");\n" );
			for( i = 0; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getSubMatrix( index3,index3+1,i,i+1 )*Ah.getSubMatrix(index2,index2+1,i,i+1) );
			}
			loop1.addStatement( loop3 );
			ExportForLoop loop4( index3,0,NX3 );
			loop4.addStatement( tmp_index1 == index2*NX3+index3 );
			loop4.addStatement( rk_b.getRow( tmp_index1 ) == rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NX1,index3*(NVARS3)+NX1+NX2 )*rk_rhsTemp.getRows(NX1,NX1+NX2) );
			if( NXA3 > 0 ) {
				loop4.addStatement( rk_b.getRow( tmp_index1 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NX1+NX2,index3*(NVARS3)+NX1+NX2+NXA )*rk_diffK.getSubMatrix( NX,NX+NXA,index2,index2+1 ) );
			}
			if( NDX3 > 0 ) {
				loop4.addStatement( rk_b.getRow( tmp_index1 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NVARS3-NX2,index3*(NVARS3)+NVARS3 )*rk_diffK.getSubMatrix( NX1,NX1+NX2,index2,index2+1 ) );
			}
			loop1.addStatement( loop4 );
		}
		else if( !STATES ) {
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getSubMatrix( index3,index3+1,0,1 )*Ah.getSubMatrix(index2,index2+1,0,1) );
			for( i = 1; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getSubMatrix( index3,index3+1,i,i+1 )*Ah.getSubMatrix(index2,index2+1,i,i+1) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,NX1,NX1+NX2 );
			loop3.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getSubMatrix( index3,index3+1,0,1 )*Ah.getSubMatrix(index2,index2+1,0,1) );
			for( i = 1; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getSubMatrix( index3,index3+1,i,i+1 )*Ah.getSubMatrix(index2,index2+1,i,i+1) );
			}
			loop1.addStatement( loop3 );
			ExportForLoop loop4( index3,0,NX3 );
			loop4.addStatement( tmp_index1 == index2*NX3+index3 );
			loop4.addStatement( tmp_index2 == index1+index3*(NVARS3) );
			loop4.addStatement( rk_b.getRow( tmp_index1 ) == rk_diffsTemp3.getSubMatrix( index2,index2+1,tmp_index2+NX1+NX2+NXA3,tmp_index2+NX1+NX2+NXA3+1 ) );
			loop4.addStatement( rk_b.getRow( tmp_index1 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3),index3*(NVARS3)+NX1+NX2 )*rk_rhsTemp.getRows(0,NX1+NX2) );
			if( NXA3 > 0 ) {
				loop4.addStatement( rk_b.getRow( tmp_index1 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NX1+NX2,index3*(NVARS3)+NX1+NX2+NXA )*rk_diffK.getSubMatrix( NX,NX+NXA,index2,index2+1 ) );
			}
			if( NDX3 > 0 ) {
				loop4.addStatement( rk_b.getRow( tmp_index1 ) += rk_diffsTemp3.getSubMatrix( index2,index2+1,index3*(NVARS3)+NVARS3-NX1-NX2,index3*(NVARS3)+NVARS3 )*rk_diffK.getSubMatrix( 0,NX1+NX2,index2,index2+1 ) );
			}
			loop1.addStatement( loop4 );
		}
		if( !STATES || number != 3 ) block->addStatement( loop1 );

		// update rk_diffK with the new sensitivities:
		if( STATES && number == 3 ) {
			block->addStatement( rk_diffK.getRows(NX1+NX2,NX) == rk_dk3.getRows(index1*NX3-(NX1+NX2)*NX3,index1*NX3+NX3-(NX1+NX2)*NX3) );
		}
		else {
			ExportForLoop loop4( index2,0,numStages );
			ExportForLoop loop5( index3,0,NX3 );
			loop5.addStatement( tmp_index1 == index2*NX3+index3 );
			loop5.addStatement( rk_diffK.getSubMatrix(NX1+NX2+index3,NX1+NX2+index3+1,index2,index2+1) == rk_mat3.getSubMatrix(tmp_index1,tmp_index1+1,0,1)*rk_b.getRow(0) );
			ExportForLoop loop6( index4,1,numStages*NX3 );
			loop6.addStatement( rk_diffK.getSubMatrix(NX1+NX2+index3,NX1+NX2+index3+1,index2,index2+1) += rk_mat3.getSubMatrix(tmp_index1,tmp_index1+1,index4,index4+1)*rk_b.getRow(index4) );
			loop5.addStatement(loop6);
			loop4.addStatement(loop5);
			block->addStatement(loop4);
		}
		// update rk_diffsNew with the new sensitivities:
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( String( "if( run == 0 ) {\n" ) );
		ExportForLoop loop8( index2,0,NX3 );
		if( STATES && number == 3 ) loop8.addStatement( String(rk_diffsNew3.get( index2,index1 )) << " = (" << index2.getName() << " == " << index1.getName() << "-" << String(NX1+NX2) << ");\n" );

		if( STATES && number == 3 ) loop8.addStatement( rk_diffsNew3.getSubMatrix( index2,index2+1,index1,index1+1 ) += rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		else if( STATES )	loop8.addStatement( rk_diffsNew3.getSubMatrix( index2,index2+1,index1,index1+1 ) == rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		else		 		loop8.addStatement( rk_diffsNew3.getSubMatrix( index2,index2+1,index1+NX,index1+NX+1 ) == rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		block->addStatement( loop8 );
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( String( "}\n" ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::evaluateStatesImplicitSystem( ExportStatementBlock* block, const ExportVariable& Ah, const ExportIndex& stage, const ExportIndex& i, const ExportIndex& j )
{
	ExportForLoop loop1( i, 0, NX1+NX2 );
	loop1.addStatement( rk_xxx.getCol( i ) == rk_eta.getCol( i ) );
	ExportForLoop loop2( j, 0, numStages );
	loop2.addStatement( rk_xxx.getCol( i ) += Ah.getSubMatrix(stage,stage+1,j,j+1)*rk_kkk.getSubMatrix( i,i+1,j,j+1 ) );
	loop1.addStatement( loop2 );
	block->addStatement( loop1 );

	ExportForLoop loop3( i, 0, NXA );
	loop3.addStatement( rk_xxx.getCol( NX+i ) == rk_kkk.getSubMatrix( NX+i,NX+i+1,stage,stage+1 ) );
	block->addStatement( loop3 );

	ExportForLoop loop4( i, 0, NDX2 );
	loop4.addStatement( rk_xxx.getCol( inputDim-diffsDim+i ) == rk_kkk.getSubMatrix( i,i+1,stage,stage+1 ) );
	block->addStatement( loop4 );

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::evaluateStatesOutputSystem( ExportStatementBlock* block, const ExportVariable& Ah, const ExportIndex& stage )
{
	uint i,j;
	for( i = 0; i < NX1+NX2; i++ ) {
		block->addStatement( rk_xxx.getCol( i ) == rk_eta.getCol( i ) );
		for( j = 0; j < numStages; j++ ) {
			block->addStatement( rk_xxx.getCol( i ) += Ah.getSubMatrix(stage,stage+1,j,j+1)*rk_kkk.getSubMatrix( i,i+1,j,j+1 ) );
		}
	}
	for( i = 0; i < NX3; i++ ) {
		block->addStatement( rk_xxx.getCol( NX1+NX2+i ) == rk_eta.getCol( NX1+NX2+i ) );
	}
	for( i = 0; i < NXA3; i++ ) {
		block->addStatement( rk_xxx.getCol( NX+i ) == rk_kkk.getSubMatrix( NX+i,NX+i+1,stage,stage+1 ) );
	}
	for( i = 0; i < NDX3; i++ ) {
		block->addStatement( rk_xxx.getCol( inputDim-diffsDim+i ) == rk_kkk.getSubMatrix( i,i+1,stage,stage+1 ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::evaluateRhsImplicitSystem( ExportStatementBlock* block, const ExportIndex& stage )
{
	Matrix zeroM = zeros( NX2+NXA,1 );
	block->addFunctionCall( getNameRHS(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
	// matrix rk_b:
	if( NDX2 == 0 ) {
		block->addStatement( rk_b.getRows( stage*(NX2+NXA),stage*(NX2+NXA)+NX2 ) == rk_kkk.getSubMatrix( NX1,NX1+NX2,stage,stage+1 ) - rk_rhsTemp.getRows( 0,NX2 ) );
	}
	else {
		block->addStatement( rk_b.getRows( stage*(NX2+NXA),stage*(NX2+NXA)+NX2 ) == zeroM.getRows( 0,NX2-1 ) - rk_rhsTemp.getRows( 0,NX2 ) );
	}
	if( NXA > 0 ) {
		block->addStatement( rk_b.getRows( stage*(NX2+NXA)+NX2,(stage+1)*(NX2+NXA) ) == zeroM.getRows( 0,NXA-1 ) - rk_rhsTemp.getRows( NX2,NX2+NXA ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::evaluateMatrix( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index, const ExportVariable& Ah, BooleanType evaluateB )
{
	uint i;

	evaluateStatesImplicitSystem( block, Ah, index1, index2, tmp_index );

	block->addFunctionCall( getNameDiffsRHS(), rk_xxx, rk_diffsTemp2.getAddress(index1,0) );
	ExportForLoop loop2( index2,0,NX2+NXA );
	loop2.addStatement( tmp_index == index1*(NX2+NXA)+index2 );
	for( i = 0; i < numStages; i++ ) { // differential states
		if( NDX2 == 0 ) {
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) == Ah.getSubMatrix( index1,index1+1,i,i+1 )*rk_diffsTemp2.getSubMatrix( index1,index1+1,index2*(NVARS2)+NX1,index2*(NVARS2)+NX1+NX2 ) );
			loop2.addStatement( String( "if( " ) << i << " == " << index1.getName() << " ) " );
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,index2+i*NX2,index2+i*NX2+1 ) -= 1 );
		}
		else {
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) == Ah.getSubMatrix( index1,index1+1,i,i+1 )*rk_diffsTemp2.getSubMatrix( index1,index1+1,index2*(NVARS2)+NX1,index2*(NVARS2)+NX1+NX2 ) );
			loop2.addStatement( String( "if( " ) << i << " == " << index1.getName() << " ) {\n" );
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) += rk_diffsTemp2.getSubMatrix( index1,index1+1,index2*(NVARS2)+NVARS2-NX2,index2*(NVARS2)+NVARS2 ) );
			loop2.addStatement( String( "}\n" ) );
		}
	}
	if( NXA > 0 ) {
		Matrix zeroM = zeros( 1,NXA );
		for( i = 0; i < numStages; i++ ) { // algebraic states
			loop2.addStatement( String( "if( " ) << i << " == " << index1.getName() << " ) {\n" );
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,numStages*NX2+i*NXA,numStages*NX2+i*NXA+NXA ) == rk_diffsTemp2.getSubMatrix( index1,index1+1,index2*(NVARS2)+NX1+NX2,index2*(NVARS2)+NX1+NX2+NXA ) );
			loop2.addStatement( String( "}\n else {\n" ) );
			loop2.addStatement( rk_A.getSubMatrix( tmp_index,tmp_index+1,numStages*NX2+i*NXA,numStages*NX2+i*NXA+NXA ) == zeroM );
			loop2.addStatement( String( "}\n" ) );
		}
	}
	block->addStatement( loop2 );
	if( evaluateB ) {
		evaluateRhsImplicitSystem( block, index1 );
	}

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
		ExportStatementBlock *loop1;
		ExportForLoop tmpLoop2( index1,0,totalMeas[i] );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop1 = &tmpLoop2;
			if( equidistant ) {
				for( j = 0; j < numStages; j++ ) {
					loop1->addStatement( rk_outH.getRow(j) == polynVariables[i].getSubMatrix( index1,index1+1,j,j+1 ) );
				}
			}
			else {
				for( j = 0; j < numStages; j++ ) {
					loop1->addStatement( rk_outH.getRow(j) == stepsH.getCol( index0 )*polynVariables[i].getSubMatrix( index1,index1+1,j,j+1 ) );
				}
			}
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) {
				for( j = 0; j < numStages; j++ ) {
					loop1->addStatement( rk_out.getRow(j) == polynDerVariables[i].getSubMatrix( index1,index1+1,j,j+1 ) );
				}
			}
		}
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop1 = block;
			loop1->addStatement( String("for(") << index1.getName() << " = 0; " << index1.getName() << " < (int)" << numMeasVariables[i].get(0,index0) << "; " << index1.getName() << "++) {\n" );
			loop1->addStatement( tmp_index1 == numMeas[i]+index1 );
			if( equidistant ) {
				for( j = 0; j < numStages; j++ ) {
					loop1->addStatement( rk_outH.getRow(j) == polynVariables[i].getSubMatrix( tmp_index1,tmp_index1+1,j,j+1 ) );
				}
			}
			else {
				for( j = 0; j < numStages; j++ ) {
					loop1->addStatement( rk_outH.getRow(j) == stepsH.getCol( index0 )*polynVariables[i].getSubMatrix( tmp_index1,tmp_index1+1,j,j+1 ) );
				}
			}
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) {
				for( j = 0; j < numStages; j++ ) {
					loop1->addStatement( rk_out.getRow(j) == polynDerVariables[i].getSubMatrix( tmp_index1,tmp_index1+1,j,j+1 ) );
				}
			}
		}
		else { // ONLINE_GRID
			loop1 = block;
			loop1->addStatement( String(tmp_index2.getName()) << " = " << tmp_meas.get( 0,i ) << ";\n" );
			loop1->addStatement( String("for(") << index1.getName() << " = 0; " << index1.getName() << " < (int)" << tmp_index2.getName() << "; " << index1.getName() << "++) {\n" );
			loop1->addStatement( tmp_index1 == numMeas[i]+index1 );

			if( equidistant ) {
				uint scale = grid.getNumIntervals();
				double scale2 = 1.0/grid.getNumIntervals();
				loop1->addStatement( time_tmp.getName() << " = " << String(scale) << "*(" << gridVariables[i].get(0,tmp_index1) << "-" << String(scale2) << "*" << index0.getName() << ");\n" );
			}
			else {
				loop1->addStatement( time_tmp.getName() << " = " << (grid.getLastTime()-grid.getFirstTime()) << "*(" << gridVariables[i].get(0,tmp_index1) << "-" << rk_tPrev.getName() << ")/" << stepsH.get( 0,index0 ) << ";\n" );
			}

			String h;
			if( equidistant ) 	h = String((grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals());
			else				h = stepsH.get( 0,index0 );
			evaluatePolynomial( *loop1, rk_outH, time_tmp, h );
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) evaluateDerivedPolynomial( *loop1, rk_out, time_tmp );
		}

		Vector dependencyX, dependencyZ, dependencyDX;
		if( exportRhs || crsFormat ) {
			dependencyX = sumRow( outputDependencies[i].getCols( 0,NX-1 ) );
			if( numXA_output(i) > 0 ) dependencyZ = sumRow( outputDependencies[i].getCols( NX,NX+NXA-1 ) );
			if( numDX_output(i) > 0 ) dependencyDX = sumRow( outputDependencies[i].getCols( NX+NXA+NU,NX+NXA+NU+NDX-1 ) );
		}
		for( j = 0; j < NX; j++ ) {
			if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyX(j)) != 0 ) {
				loop1->addStatement( rk_xxx.getCol( j ) == rk_eta.getCol( j ) + rk_kkk.getRow( j )*rk_outH );
			}
		}
		for( j = 0; j < numXA_output(i); j++ ) {
			if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyZ(j)) != 0 ) {
				loop1->addStatement( rk_xxx.getCol( NX+j ) == rk_kkk.getRow( NX+j )*rk_out );
			}
		}
		for( j = 0; j < numDX_output(i); j++ ) {
			if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyDX(j)) != 0 ) {
				loop1->addStatement( rk_xxx.getCol( inputDim-diffsDim+j ) == rk_kkk.getRow( j )*rk_out );
			}
		}
		loop1->addFunctionCall( getNameOUTPUT( i ), rk_xxx, rk_rhsOutputTemp.getAddress(0,0) );
		uint numOutputs = getDimOUTPUT( i );
		uint outputDim = numOutputs*(NX+NU+1);
		loop1->addStatement( tmp_index1 == numMeas[i]*outputDim+index1*(numOutputs*(NX+NU+1)) );
		for( j = 0; j < numOutputs; j++ ) {
			loop1->addStatement( rk_outputs[i].getCol( tmp_index1+j ) == rk_rhsOutputTemp.getCol( j ) );
		}
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			block->addStatement( *loop1 );
		}
		else {
			loop1->addStatement( "}\n" );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::sensitivitiesOutputs( ExportStatementBlock* block, const ExportIndex& index0,
		const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
		const ExportIndex& tmp_index3, const ExportVariable& tmp_meas, const ExportVariable& rk_tPrev, const ExportVariable& time_tmp, BooleanType STATES, uint base )
{
	uint i, j, k;
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	for( i = 0; i < rk_outputs.size(); i++ ) {
		uint NVARS = numVARS_output(i);
		ExportStatementBlock *loop;
		ExportForLoop tmpLoop( index2,0,totalMeas[i] );
		if( (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) {
			loop = &tmpLoop;
			if( equidistant ) {
				for( j = 0; j < numStages; j++ ) {
					loop->addStatement( rk_outH.getRow(j) == polynVariables[i].getSubMatrix( index2,index2+1,j,j+1 ) );
				}
			}
			else {
				for( j = 0; j < numStages; j++ ) {
					loop->addStatement( rk_outH.getRow(j) == stepsH.getCol( index0 )*polynVariables[i].getSubMatrix( index2,index2+1,j,j+1 ) );
				}
			}
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) {
				for( j = 0; j < numStages; j++ ) {
					loop->addStatement( rk_out.getRow(j) == polynDerVariables[i].getSubMatrix( index2,index2+1,j,j+1 ) );
				}
			}
		}
		else if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
			loop = block;
			loop->addStatement( String("for(") << index2.getName() << " = 0; " << index2.getName() << " < (int)" << numMeasVariables[i].get(0,index0) << "; " << index2.getName() << "++) {\n" );
			loop->addStatement( tmp_index2 == numMeas[i]+index2 );
			if( equidistant ) {
				for( j = 0; j < numStages; j++ ) {
					loop->addStatement( rk_outH.getRow(j) == polynVariables[i].getSubMatrix( tmp_index2,tmp_index2+1,j,j+1 ) );
				}
			}
			else {
				for( j = 0; j < numStages; j++ ) {
					loop->addStatement( rk_outH.getRow(j) == stepsH.getCol( index0 )*polynVariables[i].getSubMatrix( tmp_index2,tmp_index2+1,j,j+1 ) );
				}
			}
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) {
				for( j = 0; j < numStages; j++ ) {
					loop->addStatement( rk_out.getRow(j) == polynDerVariables[i].getSubMatrix( tmp_index2,tmp_index2+1,j,j+1 ) );
				}
			}
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
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) evaluateDerivedPolynomial( *loop, rk_out, time_tmp );
		}

		Vector dependencyX, dependencyZ, dependencyDX;
		if( exportRhs || crsFormat ) {
			dependencyX = sumRow( outputDependencies[i].getCols( 0,NX-1 ) );
			if( numXA_output(i) > 0 ) dependencyZ = sumRow( outputDependencies[i].getCols( NX,NX+NXA-1 ) );
			if( numDX_output(i) > 0 ) dependencyDX = sumRow( outputDependencies[i].getCols( NX+NXA+NU,NX+NXA+NU+NDX-1 ) );
		}
		for( j = 0; j < NX; j++ ) {
			if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyX(j)) != 0 ) {
				if( STATES && j >= base ) {
					loop->addStatement( String(rk_rhsTemp.get( j,0 )) << " = (" << j << " == " << index1.getName() << ");\n" );
				}
				else if( j >= base ) {
					loop->addStatement( rk_rhsTemp.getRow( j ) == 0.0 );
				}
				if( j >= base ) loop->addStatement( rk_rhsTemp.getRow( j ) += rk_diffK.getRows( j,j+1 )*rk_outH );
				loop->addStatement( rk_xxx.getCol( j ) == rk_eta.getCol( j ) + rk_kkk.getRow( j )*rk_outH );
			}
		}
		for( j = 0; j < numXA_output(i); j++ ) {
			if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyZ(j)) != 0 ) {
				if( base < NX1+NX2 ) loop->addStatement( rk_rhsTemp.getRow( NX+j ) == rk_diffK.getRows( NX+j,NX+j+1 )*rk_out );
				loop->addStatement( rk_xxx.getCol( NX+j ) == rk_kkk.getRow( NX+j )*rk_out );
			}
		}
		for( j = 0; j < numDX_output(i); j++ ) {
			if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyDX(j)) != 0 ) {
				if( j >= base ) loop->addStatement( rk_rhsTemp.getRow( NX+NXA+j ) == rk_diffK.getRows( j,j+1 )*rk_out );
				loop->addStatement( rk_xxx.getCol( inputDim-diffsDim+j ) == rk_kkk.getRow( j )*rk_out );
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
				if( NDX > 0 ) dependencyDX = (outputDependencies[i].getCols( NX+NXA+NU,NX+NXA+NU+NDX-1 )).getRow( j );
			}
			uint offset;
			if( STATES ) {
				offset = numOutputs+j*NX;
				loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) == 0.0 );
			}
			else {
				offset = numOutputs*(1+NX)+j*NU;
				loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) == rk_diffsOutputTemp.getCol( index1+j*NVARS+NX+NXA ) );
			}

			for( k = base; k < NX; k++ ) {
				if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyX(k)) != 0 ) {
					loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) += rk_diffsOutputTemp.getCol( j*NVARS+k )*rk_rhsTemp.getRow( k ) );
				}
			}
			if( base < NX1+NX2 ) {
				for( k = 0; k < numXA_output(i); k++ ) {
					if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyZ(k)) != 0 ) {
						loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) += rk_diffsOutputTemp.getCol( j*NVARS+NX+k )*rk_rhsTemp.getRow( NX+k ) );
					}
				}
			}
			for( k = base; k < numDX_output(i); k++ ) {
				if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyDX(k)) != 0 ) {
					loop->addStatement( rk_outputs[i].getCol( tmp_index2+offset ) += rk_diffsOutputTemp.getCol( j*NVARS+NX+NXA+NU+k )*rk_rhsTemp.getRow( NX+NXA+k ) );
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
				block.addStatement( variable.getRow( j ) == polynEvalVar*coeffs( numStages-1-i,j ) );
			}
			else {
				block.addStatement( variable.getRow( j ) += polynEvalVar*coeffs( numStages-1-i,j ) );
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
	if( CONTINUOUS_OUTPUT && !equidistantControlGrid() ) return ACADOERROR( RET_INVALID_OPTION );
	if( !equidistant && !equidistantControlGrid() ) return ACADOERROR( RET_INVALID_OPTION );

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );
	if( !equidistant && (MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID ) return ACADOERROR( RET_INVALID_OPTION );

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
	
	int debugMode;
	get( INTEGRATOR_DEBUG_MODE, debugMode );

	DifferentialStateDerivative dummy4;
	dummy4.clearStaticCounters();
	dx = DifferentialStateDerivative(NDX);

	AlgebraicState 	  dummy3;
	dummy3.clearStaticCounters();
	z = AlgebraicState(NXA);

	uint Xmax = NX1;
	if( NX2 > Xmax ) Xmax = NX2;
	if( NX3 > Xmax ) Xmax = NX3;
	NVARS2 = NX1+NX2+NXA+NU+NDX2;
	NVARS3 = NX1+NX2+NXA3+NU+NDX3;
	diffsDim = (NX+NXA)*(NX+NU);
	inputDim = (NX+NXA)*(NX+NU+1) + NU + NP;
	
	uint numDX = NDX2;
	if( NDX3 > numDX ) numDX = NDX3;
	if( NDX > numDX ) numDX = NDX;

	uint numXA = NXA;
	if( NXA3 > numXA ) numXA = NXA3;

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	rk_ttt = ExportVariable( "rk_ttt", 1, 1, REAL, structWspace, BT_TRUE );
	rk_xxx = ExportVariable( "rk_xxx", 1, inputDim-diffsDim+numDX, REAL, structWspace );
	rk_kkk = ExportVariable( "rk_kkk", NX+NXA, numStages, REAL, structWspace );
	rk_A = ExportVariable( "rk_A", numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, structWspace );
	if ( (BooleanType)debugMode == BT_TRUE && useOMP ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	else {
		debug_mat = ExportVariable( "debug_mat", numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, ACADO_VARIABLES );
	}
	rk_b = ExportVariable( "rk_b", numStages*(Xmax+NXA), 1, REAL, structWspace );
	rk_diffK = ExportVariable( "rk_diffK", NX+NXA, numStages, REAL, structWspace );
	rk_rhsTemp = ExportVariable( "rk_rhsTemp", NX+NXA+numDX, 1, REAL, structWspace );
	rk_diffsTemp2 = ExportVariable( "rk_diffsTemp2", numStages, (NX2+NXA)*(NVARS2), REAL, structWspace );
	rk_diffsTemp3 = ExportVariable( "rk_diffsTemp3", numStages, NX3*NVARS3, REAL, structWspace );
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		rk_diffsPrev1 = ExportVariable( "rk_diffsPrev1", NX1, NX1+NU, REAL, structWspace );
		rk_diffsPrev2 = ExportVariable( "rk_diffsPrev2", NX2, NX1+NX2+NU, REAL, structWspace );
		rk_diffsPrev3 = ExportVariable( "rk_diffsPrev3", NX3, NX+NU, REAL, structWspace );
	}
	rk_diffsNew1 = ExportVariable( "rk_diffsNew1", NX1, NX1+NU, REAL, structWspace );
	rk_diffsNew2 = ExportVariable( "rk_diffsNew2", NX2+NXA, NX1+NX2+NU, REAL, structWspace );
	rk_diffsNew3 = ExportVariable( "rk_diffsNew3", NX3, NX+NU, REAL, structWspace );
	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, BT_TRUE );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
	integrate = ExportFunction( "integrate", rk_eta );
	uint i;
	for( i = 0; i < rk_outputs.size(); i++ ) {
		integrate.addArgument( rk_outputs[i] );
	}
	integrate.addArgument( reset_int );
	if( !equidistantControlGrid() ) integrate.addArgument( rk_index );
	integrate.setReturnValue( error_code );
	integrate.addLinebreak( );	// TO MAKE SURE IT GETS EXPORTED
	
	rhs_in = ExportVariable( "x", inputDim-diffsDim+NX, 1, REAL, ACADO_LOCAL );
	rhs_out = ExportVariable( "f", NX+NXA, 1, REAL, ACADO_LOCAL );
	fullRhs = ExportFunction( "full_rhs", rhs_in, rhs_out );

	if( NX2 > 0 || NXA > 0 ) {
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
		solver->init( (NX2+NXA)*numStages );
		solver->setup();
		rk_auxSolver = solver->getGlobalExportVariable( 1 );
	}

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

	numDX_output = Vector(outputGrids.size());
	numXA_output = Vector(outputGrids.size());
	numVARS_output = Vector(outputGrids.size());

	uint i;
	uint maxOutputs = 0;
	uint maxVARS = 0;
	rk_outputs.clear();
	outputs.clear();
	diffs_outputs.clear();
	for( i = 0; i < outputGrids.size(); i++ ) {
		uint numOutputs = outputExpressions_[i].getDim();
		uint outputDim = outputGrids[i].getNumIntervals( )*numOutputs*(NX+NU+1);
		
		if( numOutputs > maxOutputs ) maxOutputs = numOutputs;
		
		OutputFcn f_Output;
		f_Output << outputExpressions_[i];

		if( f_Output.getNDX() > 0 ) numDX_output(i) = NDX;
		else						numDX_output(i) = 0;
		if( f_Output.getNXA() > 0 ) numXA_output(i) = NXA;
		else						numXA_output(i) = 0;
	
		OutputFcn g_Output;
		for( uint j = 0; j < outputExpressions_[i].getDim(); j++ ) {
			g_Output << forwardDerivative( outputExpressions_[i](j), x );
			g_Output << forwardDerivative( outputExpressions_[i](j), z );
			g_Output << forwardDerivative( outputExpressions_[i](j), u );
			if( numDX_output(i) > 0 ) g_Output << forwardDerivative( outputExpressions_[i](j), dx );
		}
		if( numDX_output(i) > 0 ) {
			numVARS_output(i) = NX+NXA+NU+NDX;
		}
		else {
			numVARS_output(i) = NX+NXA+NU;
		}
		if( numVARS_output(i) > maxVARS ) maxVARS = numVARS_output(i);
	
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
	
	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	setup();
	rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, structWspace );
	rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(maxVARS), REAL, structWspace );
	rk_outH = ExportVariable( "rk_outH", numStages, 1, REAL, structWspace );
	rk_out = ExportVariable( "rk_out2", numStages, 1, REAL, structWspace );
	polynEvalVar = ExportVariable( "tmp_polyn", 1, 1, REAL, ACADO_LOCAL, BT_TRUE );

	return ( val );
}


returnValue ImplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<String> _outputNames,
									  	  const std::vector<String> _diffs_outputNames,
										  const std::vector<uint> _dims_output ) {
	if( (rhs.getFunctionDim()) == 0 && (rk_outputs.size() + outputs.size() + diffs_outputs.size()) == 0) {
		CONTINUOUS_OUTPUT = BT_TRUE;
		if( outputGrids_.size() != _outputNames.size() || outputGrids_.size() != _diffs_outputNames.size() || outputGrids_.size() != _dims_output.size() ) {
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		}
		outputGrids = outputGrids_;
		name_outputs = _outputNames;
		name_diffs_outputs = _diffs_outputNames;
		num_outputs = _dims_output;

		int measGrid;
		get( MEASUREMENT_GRID, measGrid );

		numDX_output = Vector(outputGrids.size());
		numXA_output = Vector(outputGrids.size());
		numVARS_output = Vector(outputGrids.size());

		uint i;
		uint maxOutputs = 0;
		rk_outputs.clear();
		outputs.clear();
		diffs_outputs.clear();
		for( i = 0; i < outputGrids.size(); i++ ) {
			uint numOutputs = num_outputs[i];
			uint outputDim = outputGrids[i].getNumIntervals( )*numOutputs*(NX+NU+1);

			if( numOutputs > maxOutputs ) maxOutputs = numOutputs;

			numDX_output(i) = NDX;	// worst-case scenario
			numXA_output(i) = NXA;	// worst-case scenario
			numVARS_output(i) = NX+NXA+NU+NDX;

			ExportVariable rk_output( String("rk_output")<<String(i), 1, outputDim, REAL );
			rk_outputs.push_back( rk_output );

			if((MeasurementGrid)measGrid == EQUIDISTANT_SUBGRID) totalMeas.push_back( (int)ceil((double)outputGrids[i].getNumIntervals()/((double) grid.getNumIntervals()) - 10.0*EPS) );
			else totalMeas.push_back( outputGrids[i].getNumIntervals() );

			// Export output grids
			ExportVariable gridVariable( (String)"gridOutput" << i, 1, totalMeas[i], REAL, ACADO_VARIABLES );
			gridVariables.push_back( gridVariable );
		}
		uint maxVARS = NX+NXA+NU+NDX;

		int useOMP;
		get(CG_USE_OPENMP, useOMP);
		ExportStruct structWspace;
		structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

		setup();
		rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, structWspace );
		rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(maxVARS), REAL, structWspace );
		rk_outH = ExportVariable( "rk_outH", numStages, 1, REAL, structWspace );
		rk_out = ExportVariable( "rk_out2", numStages, 1, REAL, structWspace );
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


returnValue ImplicitRungeKuttaExport::prepareOutputEvaluation( ExportStatementBlock& code )
{
	uint i;
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	for( i = 0; i < outputGrids.size(); i++ ) {
		if( (MeasurementGrid)measGrid != ONLINE_GRID ) {
			Matrix polynV = evaluatePolynomial( i );
			Matrix polynDerV = evaluateDerivedPolynomial( i );
			Vector measurements = divideMeasurements( i );

			ExportVariable polynVariable;
			if( equidistant ) {
				double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
				polynVariable = ExportVariable( (String)"polynOutput" << i, polynV*=h, STATIC_CONST_REAL );
			}
			else {
				polynVariable = ExportVariable( (String)"polynOutput" << i, polynV, STATIC_CONST_REAL );
			}
			code.addDeclaration( polynVariable );
			polynVariable = ExportVariable( (String)"polynOutput" << i, totalMeas[i], numStages, STATIC_CONST_REAL );
			polynVariables.push_back( polynVariable );

			if( NXA > 0 || NDX > 0 ) {
				ExportVariable polynDerVariable( (String)"polynDerOutput" << i, polynDerV, STATIC_CONST_REAL );
				code.addDeclaration( polynDerVariable );
				polynDerVariable = ExportVariable( (String)"polynDerOutput" << i, totalMeas[i], numStages, STATIC_CONST_REAL );
				polynDerVariables.push_back( polynDerVariable );
			}

			ExportVariable numMeasVariable( (String)"numMeas" << i, measurements, STATIC_CONST_INT );
			if( (MeasurementGrid)measGrid == EQUIDISTANT_GRID ) {
				code.addDeclaration( numMeasVariable );
			}
			numMeasVariables.push_back( numMeasVariable );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::prepareFullRhs( ) {

	uint i, j;
	// PART 1:
	for( i = 0; i < NX1; i++ ) {
		fullRhs.addStatement( rhs_out.getRow(i) == A11(i,0)*rhs_in.getRow(0) );
		for( j = 1; j < NX1; j++ ) {
			if( acadoRoundAway(A11(i,j)) != 0 ) {
				fullRhs.addStatement( rhs_out.getRow(i) += A11(i,j)*rhs_in.getRow(j) );
			}
		}
		for( j = 0; j < NU; j++ ) {
			if( acadoRoundAway(B11(i,j)) != 0 ) {
				fullRhs.addStatement( rhs_out.getRow(i) += B11(i,j)*rhs_in.getRow(NX+NXA+j) );
			}
		}
		for( j = 0; j < NX1; j++ ) {
			if( acadoRoundAway(M11(i,j)) != 0 ) {
				fullRhs.addStatement( rhs_out.getRow(i) -= M11(i,j)*rhs_in.getRow(NX+NXA+NU+j) );
			}
		}
	}

	// PART 2:
	if( NX2 > 0 ) {
		fullRhs.addFunctionCall( getNameRHS(), rhs_in, rhs_out.getAddress(NX1,0) );
		if( NDX2 == 0 ) {
			fullRhs.addStatement( rhs_out.getRows(NX1,NX1+NX2) -= rhs_in.getRows(NX+NXA+NU+NX1,NX+NXA+NU+NX1+NX2) );
		}
		if( NX3 > 0 ) {
			for( i = 0; i < NXA; i++ ) {
				fullRhs.addStatement( rhs_out.getRow(NX+i) == rhs_out.getRow(NX1+NX2+i) );
			}
		}
	}

	// PART 3:
	if( NX3 > 0 ) {
		fullRhs.addFunctionCall( getNameOutputRHS(), rhs_in, rhs_out.getAddress(NX1+NX2,0) );
		for( i = 0; i < NX3; i++ ) {
			for( j = 0; j < NX3; j++ ) {
				if( acadoRoundAway(M33(i,j)) != 0 ) {
					fullRhs.addStatement( rhs_out.getRow(NX1+NX2+i) -= M33(i,j)*rhs_in.getRow(NX+NXA+NU+NX1+NX2+j) );
				}
			}
		}
	}


	return SUCCESSFUL_RETURN;
}


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
	debug_mat = arg.debug_mat;
	rk_b = arg.rk_b;
	rk_diffK = arg.rk_diffK;
	rk_rhsTemp = arg.rk_rhsTemp;
	rk_diffsTemp2 = arg.rk_diffsTemp2;
	rk_diffsTemp3 = arg.rk_diffsTemp3;
	rk_diffsNew1 = arg.rk_diffsNew1;
	rk_diffsPrev2 = arg.rk_diffsPrev1;
	rk_diffsNew2 = arg.rk_diffsNew2;
	rk_diffsPrev2 = arg.rk_diffsPrev2;
	rk_diffsNew3 = arg.rk_diffsNew3;
	rk_diffsPrev2 = arg.rk_diffsPrev3;
	rk_eta = arg.rk_eta;
	rk_rhsOutputTemp = arg.rk_rhsOutputTemp;
	rk_diffsOutputTemp = arg.rk_diffsOutputTemp;
	rk_outH = arg.rk_outH;
	rk_out = arg.rk_out;
	polynEvalVar = arg.polynEvalVar;
	rk_outputs = arg.rk_outputs;
	
	gridVariables = arg.gridVariables;
	totalMeas = arg.totalMeas;

	// ExportFunctions
	integrate = arg.integrate;
	fullRhs = arg.fullRhs;
	
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


const String ImplicitRungeKuttaExport::getNameOutputRHS() const{
	if( exportRhs ) {
		return rhs3.getName();
	}
	else {
		return name_rhs3;
	}
}


const String ImplicitRungeKuttaExport::getNameOutputDiffs() const{
	if( exportRhs ) {
		return diffs_rhs3.getName();
	}
	else {
		return name_diffs_rhs3;
	}
}


const String ImplicitRungeKuttaExport::getNameFullRHS() const {
	if( NX2 == NX ) {
		return getNameRHS();
	}
	else {
		return fullRhs.getName();
	}
}


CLOSE_NAMESPACE_ACADO

// end of file.
