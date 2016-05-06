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
 *    \file src/code_generation/integrators/irk_export.cpp
 *    \author Rien Quirynen
 *    \date 2012
 */

#include <acado/code_generation/integrators/irk_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ImplicitRungeKuttaExport::ImplicitRungeKuttaExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
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
	REUSE = true;
	CONTINUOUS_OUTPUT = false;

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


returnValue ImplicitRungeKuttaExport::setDifferentialEquation(	const Expression& rhs_ )
{
	if( rhs_.getDim() > 0 ) {
		OnlineData        dummy0;
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
		x = DifferentialState("", NX1+NX2, 1);
		z = AlgebraicState("", NXA, 1);
		u = Control("", NU, 1);
		od = OnlineData("", NOD, 1);

		DifferentialEquation f;
		f << rhs_;

		NDX2 = f.getNDX();
		if( NDX2 > 0 && (NDX2 < NX2 || NDX2 > (NX1+NX2)) ) {
			return ACADOERROR( RET_INVALID_OPTION );
		}
		else if( NDX2 > 0 ) NDX2 = NX1+NX2;
		dx = DifferentialStateDerivative("", NDX2, 1);

		DifferentialEquation g;
		for( uint i = 0; i < rhs_.getDim(); i++ ) {
			g << forwardDerivative( rhs_(i), x );
			g << forwardDerivative( rhs_(i), z );
			g << forwardDerivative( rhs_(i), u );
			g << forwardDerivative( rhs_(i), dx );
		}

		if( f.getNT() > 0 ) timeDependant = true;

		return (rhs.init( f,"rhs", NX,NXA,NU,NP,NDX,NOD ) &
				diffs_rhs.init( g,"diffs", NX,NXA,NU,NP,NDX,NOD ) );
	}
	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setModel(	const std::string& _rhs, const std::string& _diffs_rhs ) {

	IntegratorExport::setModel( _rhs, _diffs_rhs );

	NDX2 = NDX;

	setup();

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
	}
	uint i;
	for( i = 0; i < outputs.size(); i++ ) {
		if( outputs[i].getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = outputs[i].getGlobalExportVariable();
		}
	}
	return max;
}


returnValue ImplicitRungeKuttaExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	if( solver->getDim() ) solver->getDataDeclarations( declarations,dataStruct );
	
	if( NX1 > 0 || exportRhs ) {
		ExportVariable max = getAuxVariable();
		declarations.addDeclaration( max,dataStruct );
	}

	int debugMode;
	get( INTEGRATOR_DEBUG_MODE, debugMode );
	if ( (bool)debugMode == true ) {
		declarations.addDeclaration( debug_mat,dataStruct );
	}
	declarations.addDeclaration( rk_ttt,dataStruct );
	declarations.addDeclaration( rk_xxx,dataStruct );
	declarations.addDeclaration( rk_kkk,dataStruct );

	declarations.addDeclaration( rk_A,dataStruct );
	declarations.addDeclaration( rk_b,dataStruct );
	declarations.addDeclaration( rk_auxSolver,dataStruct );
	declarations.addDeclaration( rk_rhsTemp,dataStruct );
	declarations.addDeclaration( rk_diffsTemp2,dataStruct );
	
	if( CONTINUOUS_OUTPUT ) {
		declarations.addDeclaration( rk_rhsOutputTemp,dataStruct );
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

	if( NX2 != NX ) 	declarations.addDeclaration( fullRhs );
	declarations.addDeclaration( rhs );
	declarations.addDeclaration( diffs_rhs );

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::getCode(	ExportStatementBlock& code )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	if ( (ExportSensitivityType)sensGen != NO_SENSITIVITY ) ACADOERROR( RET_INVALID_OPTION );

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
		}
		for( uint i = 0; i < outputs.size(); i++ ) {
			outputs[i].setGlobalExportVariable( max );
		}

		getDataDeclarations( code, ACADO_LOCAL );

		stringstream s;
		s << "#pragma omp threadprivate( "
				<< max.getFullName() << ", "
				<< rk_ttt.getFullName() << ", "
				<< rk_xxx.getFullName() << ", "
				<< rk_kkk.getFullName() << ", "
				<< rk_rhsTemp.getFullName() << ", "
				<< rk_auxSolver.getFullName();
		if( NX2 > 0 || NXA > 0 ) {
			s << ", " << rk_A.getFullName();
			s << ", " << rk_b.getFullName();
			s << ", " << rk_diffsTemp2.getFullName();
			solver->appendVariableNames( s );
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
		}

		if( CONTINUOUS_OUTPUT ) {
			uint i;
			for( i = 0; i < outputs.size(); i++ ) {
				code.addFunction( outputs[i] );
				code.addStatement( "\n\n" );
			}
		}
	}
	returnValue ret;
	if( NX2 > 0 || NXA > 0 ) ret = solver->getCode( code );
	if( ret != SUCCESSFUL_RETURN ) return ret;
	code.addLinebreak(2);

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	// export RK scheme
	uint run5;
	std::string tempString;
	
	initializeDDMatrix();
	initializeCoefficients();
    
    string moduleName;
	get(CG_MODULE_NAME, moduleName);

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	DMatrix tmp = AA;
	ExportVariable Ah( moduleName+"_Ah_mat", tmp*=h, STATIC_CONST_REAL );
	code.addDeclaration( Ah );
	code.addLinebreak( 2 );
	// TODO: Ask Milan why this does NOT work properly !!
	Ah = ExportVariable( moduleName+"_Ah_mat", numStages, numStages, STATIC_CONST_REAL, ACADO_LOCAL );

	DVector BB( bb );
	ExportVariable Bh( moduleName+"_Bh_mat", DMatrix( BB*=h ) );

	DVector CC( cc );
	ExportVariable C;
	if( timeDependant ) {
		C = ExportVariable( moduleName+"_C_mat", DMatrix( CC*=(1.0/grid.getNumIntervals()) ), STATIC_CONST_REAL );
		code.addDeclaration( C );
		code.addLinebreak( 2 );
		C = ExportVariable( moduleName+"_C_mat", 1, numStages, STATIC_CONST_REAL, ACADO_LOCAL );
	}

	code.addComment(std::string("Fixed step size:") + toString(h));

	ExportVariable determinant( "det", 1, 1, REAL, ACADO_LOCAL, true );
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

	ExportVariable numInt( moduleName+"_numInts", 1, 1, INT );
	if( !equidistantControlGrid() ) {
		ExportVariable numStepsV( moduleName+"_numSteps", numSteps, STATIC_CONST_INT );
		code.addDeclaration( numStepsV );
		code.addLinebreak( 2 );
		integrate.addStatement( std::string( "int " ) + numInt.getName() + " = " + numStepsV.getName() + "[" + rk_index.getName() + "];\n" );
	}

	prepareOutputEvaluation( code );

	integrate.addIndex( i );
	integrate.addIndex( j );
	integrate.addIndex( k );
	integrate.addIndex( run );
	integrate.addIndex( run1 );
	integrate.addIndex( tmp_index1 );
	integrate.addIndex( tmp_index2 );
	if( rk_outputs.size() > 0 ) integrate.addIndex( tmp_index3 );
	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !equidistantControlGrid()) ) {
		integrate.addIndex( tmp_index4 );
	}
	ExportVariable time_tmp( "time_tmp", 1, 1, REAL, ACADO_LOCAL, true );
	if( CONTINUOUS_OUTPUT ) {
		for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
			ExportIndex numMeasTmp( (std::string)"numMeasTmp" + toString(run5) );
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
	integrate.addStatement( rk_ttt == DMatrix(grid.getFirstTime()) );
	if( inputDim > NX+NXA ) {
		integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim ) == rk_eta.getCols( NX+NXA,inputDim ) );
	}
	integrate.addLinebreak( );

	if( NXA > 0 ) {
		integrate.addStatement( std::string( "if( " ) + reset_int.getFullName() + " ) {\n" );
		for( run5 = 0; run5 < NXA; run5++ ) {
			for( uint iStage = 0; iStage < numStages; iStage++ ) {
				integrate.addStatement( rk_kkk.getElement(NX+run5,iStage) == rk_eta.getCol(NX+run5) );
			}
		}
		integrate.addStatement( std::string( "}\n" ) );
	}

    // integrator loop:
	ExportForLoop tmpLoop( run, 0, grid.getNumIntervals() );
	ExportStatementBlock *loop;
	if( equidistantControlGrid() ) {
		loop = &tmpLoop;
	}
	else {
	    loop = &integrate;
		loop->addStatement( std::string("for(") + run.getName() + " = 0; " + run.getName() + " < " + numInt.getName() + "; " + run.getName() + "++ ) {\n" );
	}

	if( CONTINUOUS_OUTPUT && (MeasurementGrid)measGrid == ONLINE_GRID ) {
		for( run5 = 0; run5 < outputGrids.size(); run5++ ) {
			loop->addStatement( tmp_index1 == numMeas[run5] );
			loop->addStatement( std::string("while( ") + tmp_index1.getName() + " < " + toString(totalMeas[run5]) + " && " + gridVariables[run5].get(0,tmp_index1) + " <= (" + rk_ttt.getFullName() + "+" + toString(1.0/grid.getNumIntervals()) + ") ) {\n" );
			loop->addStatement( tmp_index1 == tmp_index1+1 );
			loop->addStatement( std::string("}\n") );
			loop->addStatement( std::string(tmp_meas.get( 0,run5 )) + " = " + tmp_index1.getName() + " - " + numMeas[run5].getName() + ";\n" );
		}
	}

	// PART 1: The linear input system
	prepareInputSystem( code );
	solveInputSystem( loop, i, run1, j, tmp_index1, Ah );

	// PART 2: The fully implicit system
	solveImplicitSystem( loop, i, run1, j, tmp_index1, ExportIndex(0), Ah, C, determinant );

	// PART 3: The linear output system
	prepareOutputSystem( code );
	solveOutputSystem( loop, i, run1, j, tmp_index1, Ah );

	// generate continuous OUTPUT:
	generateOutput( loop, run, i, tmp_index2, tmp_index3, tmp_meas, time_tmp, 0 );

	// update rk_eta:
	for( run5 = 0; run5 < NX; run5++ ) {
		loop->addStatement( rk_eta.getCol( run5 ) += rk_kkk.getRow( run5 )*Bh );
	}
	if( NXA > 0) {
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );
		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
			loop->addStatement( std::string("if( run == 0 ) {\n") );
		}
		for( run5 = 0; run5 < NXA; run5++ ) {
			loop->addStatement( rk_eta.getCol( NX+run5 ) == rk_kkk.getRow( NX+run5 )*tempCoefs );
		}
		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
			loop->addStatement( std::string("}\n") );
		}
	}

	loop->addStatement( std::string( reset_int.get(0,0) ) + " = 0;\n" );

	for( run5 = 0; run5 < rk_outputs.size(); run5++ ) {
		if( (MeasurementGrid)measGrid == OFFLINE_GRID ) {
			loop->addStatement( numMeas[run5].getName() + " += " + numMeasVariables[run5].get(0,run) + ";\n" );
		}
		else { // ONLINE_GRID
			loop->addStatement( numMeas[run5].getName() + " += " + tmp_meas.get(0,run5) + ";\n" );
		}
	}
	loop->addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );

    // end of the integrator loop.
    if( !equidistantControlGrid() ) {
		loop->addStatement( "}\n" );
	}
    else {
    	integrate.addStatement( *loop );
    }

    integrate.addStatement( std::string( "if( " ) + determinant.getFullName() + " < 1e-12 ) {\n" );
    integrate.addStatement( error_code == 2 );
    integrate.addStatement( std::string( "} else if( " ) + determinant.getFullName() + " < 1e-6 ) {\n" );
    integrate.addStatement( error_code == 1 );
    integrate.addStatement( std::string( "} else {\n" ) );
    integrate.addStatement( error_code == 0 );
    integrate.addStatement( std::string( "}\n" ) );

	code.addFunction( integrate );
    code.addLinebreak( 2 );

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::prepareInputSystem(	ExportStatementBlock& code )
{
	if( NX1 > 0 ) {
		DMatrix mat1 = formMatrix( M11, A11 );
		rk_mat1 = ExportVariable( "rk_mat1", mat1, STATIC_CONST_REAL );
		code.addDeclaration( rk_mat1 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_mat1 = ExportVariable( "rk_mat1", numStages*NX1, numStages*NX1, STATIC_CONST_REAL, ACADO_LOCAL );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::prepareOutputSystem(	ExportStatementBlock& code )
{
	if( NX3 > 0 ) {
		DMatrix mat3 = formMatrix( M33, A33 );
		rk_mat3 = ExportVariable( "rk_mat3", mat3, STATIC_CONST_REAL );
		code.addDeclaration( rk_mat3 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_mat3 = ExportVariable( "rk_mat3", numStages*NX3, numStages*NX3, STATIC_CONST_REAL, ACADO_LOCAL );
	}

	return SUCCESSFUL_RETURN;
}


DMatrix ImplicitRungeKuttaExport::formMatrix( const DMatrix& mass, const DMatrix& jacobian ) {
	if( jacobian.getNumRows() != jacobian.getNumCols() ) {
		return RET_UNABLE_TO_EXPORT_CODE;
	}
	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	uint vars = jacobian.getNumRows();
	uint i1, j1, i2, j2;
	DMatrix result = zeros<double>(numStages*vars, numStages*vars);
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

	return result.inverse();
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
		loop5.addStatement( rk_kkk.getElement(index2,index1) == rk_mat1.getElement(tmp_index,0)*rk_b.getRow(0) );
		ExportForLoop loop6( index3,1,numStages*NX1 );
		loop6.addStatement( rk_kkk.getElement(index2,index1) += rk_mat1.getElement(tmp_index,index3)*rk_b.getRow(index3) );
		loop5.addStatement(loop6);
		loop4.addStatement(loop5);
		block->addStatement(loop4);
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::solveImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportIndex& k_index, const ExportVariable& Ah, const ExportVariable& C, const ExportVariable& det, bool DERIVATIVES )
{
	if( NX2 > 0 || NXA > 0 ) {

		if( DERIVATIVES && REUSE ) block->addStatement( std::string( "if( " ) + reset_int.getFullName() + " ) {\n" );
		// Initialization iterations:
		ExportForLoop loop1( index1,0,numItsInit+1 ); // NOTE: +1 because 0 will lead to NaNs, so the minimum number of iterations is 1 at the initialization
		ExportForLoop loop11( index2,0,numStages );
		evaluateMatrix( &loop11, index2, index3, tmp_index, k_index, rk_A, Ah, C, true, DERIVATIVES );
		loop1.addStatement( loop11 );
		loop1.addStatement( det.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_b.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
		ExportForLoop loopTemp( index3,0,numStages );
		loopTemp.addStatement( rk_kkk.getSubMatrix( k_index+NX1,k_index+NX1+NX2,index3,index3+1 ) += rk_b.getRows( index3*NX2,index3*NX2+NX2 ) );											// differential states
		if(NXA > 0) loopTemp.addStatement( rk_kkk.getSubMatrix( k_index+NX,k_index+NX+NXA,index3,index3+1 ) += rk_b.getRows( index3*NXA+numStages*NX2,index3*NXA+numStages*NX2+NXA ) );		// algebraic states
		loop1.addStatement( loopTemp );
		block->addStatement( loop1 );
		if( DERIVATIVES && REUSE ) block->addStatement( std::string( "}\n" ) );

		// the rest (numIts) of the Newton iterations with reuse of the Jacobian (no evaluation or factorization needed)
		ExportForLoop loop2( index1,0,numIts );
		ExportForLoop loop21( index2,0,numStages );
		evaluateStatesImplicitSystem( &loop21, k_index, Ah, C, index2, index3, tmp_index );
		evaluateRhsImplicitSystem( &loop21, k_index, index2 );
		loop2.addStatement( loop21 );
		loop2.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		loopTemp = ExportForLoop( index3,0,numStages );
		loopTemp.addStatement( rk_kkk.getSubMatrix( k_index+NX1,k_index+NX1+NX2,index3,index3+1 ) += rk_b.getRows( index3*NX2,index3*NX2+NX2 ) );														// differential states
		if(NXA > 0) loopTemp.addStatement( rk_kkk.getSubMatrix( k_index+NX,k_index+NX+NXA,index3,index3+1 ) += rk_b.getRows( index3*NXA+numStages*NX2,index3*NXA+numStages*NX2+NXA ) );		// algebraic states
		loop2.addStatement( loopTemp );
		block->addStatement( loop2 );

		if( DERIVATIVES ) {
			// solution calculated --> evaluate and save the necessary derivatives in rk_diffsTemp and update the matrix rk_A:
			ExportForLoop loop3( index2,0,numStages );
			evaluateMatrix( &loop3, index2, index3, tmp_index, k_index, rk_A, Ah, C, false, DERIVATIVES );
			block->addStatement( loop3 );
		}

		// IF DEBUG MODE:
		int debugMode;
		get( INTEGRATOR_DEBUG_MODE, debugMode );
		if ( (bool)debugMode == true ) {
			block->addStatement( debug_mat == rk_A );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::solveOutputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportVariable& Ah, bool DERIVATIVES )
{
	if( NX3 > 0 ) {
		ExportForLoop loop( index1,0,numStages );
		evaluateStatesOutputSystem( &loop, Ah, index1 );
		loop.addFunctionCall( getNameOutputRHS(), rk_xxx, rk_b.getAddress(index1*NX3,0) );
		if( DERIVATIVES )	loop.addFunctionCall( getNameOutputDiffs(), rk_xxx, rk_diffsTemp3.getAddress(index1,0) );
		block->addStatement( loop );

		ExportForLoop loop4( index1,0,numStages );
		ExportForLoop loop5( index2,0,NX3 );
		loop5.addStatement( tmp_index == index1*NX3+index2 );
		loop5.addStatement( rk_kkk.getElement(NX1+NX2+index2,index1) == rk_mat3.getElement(tmp_index,0)*rk_b.getRow(0) );
		ExportForLoop loop6( index3,1,numStages*NX3 );
		loop6.addStatement( rk_kkk.getElement(NX1+NX2+index2,index1) += rk_mat3.getElement(tmp_index,index3)*rk_b.getRow(index3) );
		loop5.addStatement(loop6);
		loop4.addStatement(loop5);
		block->addStatement(loop4);
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::evaluateStatesImplicitSystem( ExportStatementBlock* block, const ExportIndex& k_index, const ExportVariable& Ah, const ExportVariable& C, const ExportIndex& stage, const ExportIndex& i, const ExportIndex& tmp_index )
{
	ExportForLoop loop1( i, 0, NX1+NX2 );
	loop1.addStatement( rk_xxx.getCol( i ) == rk_eta.getCol( i ) );
	loop1.addStatement( tmp_index == k_index + i );
	for( uint j = 0; j < numStages; j++ ) {
		loop1.addStatement( rk_xxx.getCol( i ) += Ah.getElement(stage,j)*rk_kkk.getElement( tmp_index,j ) );
	}
	block->addStatement( loop1 );

	ExportForLoop loop3( i, 0, NXA );
	loop3.addStatement( tmp_index == k_index + i + NX );
	loop3.addStatement( rk_xxx.getCol( NX+i ) == rk_kkk.getElement( tmp_index,stage ) );
	block->addStatement( loop3 );

	ExportForLoop loop4( i, 0, NDX2 );
	loop4.addStatement( tmp_index == k_index + i );
	loop4.addStatement( rk_xxx.getCol( inputDim-diffsDim+i ) == rk_kkk.getElement( tmp_index,stage ) );
	block->addStatement( loop4 );

	if( C.getDim() > 0 ) {	// There is a time dependence, so it must be set
		block->addStatement( rk_xxx.getCol( inputDim-diffsDim+NDX ) == rk_ttt + C.getCol(stage) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::evaluateStatesOutputSystem( ExportStatementBlock* block, const ExportVariable& Ah, const ExportIndex& stage )
{
	uint i,j;
	for( i = 0; i < NX1+NX2; i++ ) {
		block->addStatement( rk_xxx.getCol( i ) == rk_eta.getCol( i ) );
		for( j = 0; j < numStages; j++ ) {
			block->addStatement( rk_xxx.getCol( i ) += Ah.getElement(stage,j)*rk_kkk.getElement( i,j ) );
		}
	}
	for( i = 0; i < NX3; i++ ) {
		block->addStatement( rk_xxx.getCol( NX1+NX2+i ) == rk_eta.getCol( NX1+NX2+i ) );
	}
	for( i = 0; i < NXA3; i++ ) {
		block->addStatement( rk_xxx.getCol( NX+i ) == rk_kkk.getElement( NX+i,stage ) );
	}
	for( i = 0; i < NDX3; i++ ) {
		block->addStatement( rk_xxx.getCol( inputDim-diffsDim+i ) == rk_kkk.getElement( i,stage ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::evaluateRhsImplicitSystem( ExportStatementBlock* block, const ExportIndex& k_index, const ExportIndex& stage )
{
	DMatrix zeroM = zeros<double>( NX2+NXA,1 );
	block->addFunctionCall( getNameRHS(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
	// matrix rk_b:
	if( NDX2 == 0 ) {
		block->addStatement( rk_b.getRows( stage*(NX2+NXA),stage*(NX2+NXA)+NX2 ) == rk_kkk.getSubMatrix( k_index+NX1,k_index+NX1+NX2,stage,stage+1 ) - rk_rhsTemp.getRows( 0,NX2 ) );
	}
	else {
		block->addStatement( rk_b.getRows( stage*(NX2+NXA),stage*(NX2+NXA)+NX2 ) == zeroM.getRows( 0,NX2-1 ) - rk_rhsTemp.getRows( 0,NX2 ) );
	}
	if( NXA > 0 ) {
		block->addStatement( rk_b.getRows( stage*(NX2+NXA)+NX2,(stage+1)*(NX2+NXA) ) == zeroM.getRows( 0,NXA-1 ) - rk_rhsTemp.getRows( NX2,NX2+NXA ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::evaluateMatrix( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index, const ExportIndex& k_index, const ExportVariable& _rk_A, const ExportVariable& Ah, const ExportVariable& C, bool evaluateB, bool DERIVATIVES )
{
	uint i;

	evaluateStatesImplicitSystem( block, k_index, Ah, C, index1, index2, tmp_index );

	ExportIndex indexDiffs(index1);
	if( !DERIVATIVES ) indexDiffs = ExportIndex(0);

	block->addFunctionCall( getNameDiffsRHS(), rk_xxx, rk_diffsTemp2.getAddress(indexDiffs,0) );
	ExportForLoop loop2( index2,0,NX2+NXA );
	loop2.addStatement( tmp_index == index1*(NX2+NXA)+index2 );
	for( i = 0; i < numStages; i++ ) { // differential states
		if( NDX2 == 0 ) {
			loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) == Ah.getElement( index1,i )*rk_diffsTemp2.getSubMatrix( indexDiffs,indexDiffs+1,index2*(NVARS2)+NX1,index2*(NVARS2)+NX1+NX2 ) );
			loop2.addStatement( std::string( "if( " ) + toString(i) + " == " + index1.getName() + " ) " );
			loop2.addStatement( _rk_A.getElement( tmp_index,index2+i*NX2 ) -= 1 );
		}
		else {
			loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) == Ah.getElement( index1,i )*rk_diffsTemp2.getSubMatrix( indexDiffs,indexDiffs+1,index2*(NVARS2)+NX1,index2*(NVARS2)+NX1+NX2 ) );
			loop2.addStatement( std::string( "if( " ) + toString(i) + " == " + index1.getName() + " ) {\n" );
			loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) += rk_diffsTemp2.getSubMatrix( indexDiffs,indexDiffs+1,index2*(NVARS2)+NVARS2-NX2,index2*(NVARS2)+NVARS2 ) );
			loop2.addStatement( std::string( "}\n" ) );
		}
	}
	if( NXA > 0 ) {
		DMatrix zeroM = zeros<double>( 1,NXA );
		for( i = 0; i < numStages; i++ ) { // algebraic states
			loop2.addStatement( std::string( "if( " ) + toString(i) + " == " + index1.getName() + " ) {\n" );
			loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,numStages*NX2+i*NXA,numStages*NX2+i*NXA+NXA ) == rk_diffsTemp2.getSubMatrix( indexDiffs,indexDiffs+1,index2*(NVARS2)+NX1+NX2,index2*(NVARS2)+NX1+NX2+NXA ) );
			loop2.addStatement( std::string( "}\n else {\n" ) );
			loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,numStages*NX2+i*NXA,numStages*NX2+i*NXA+NXA ) == zeroM );
			loop2.addStatement( std::string( "}\n" ) );
		}
	}
	block->addStatement( loop2 );
	if( evaluateB ) {
		evaluateRhsImplicitSystem( block, k_index, index1 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::generateOutput( ExportStatementBlock* block, const ExportIndex& index0,
		const ExportIndex& index1, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
		const ExportVariable& tmp_meas, const ExportVariable& time_tmp, const uint directions )
{
	uint i, j;
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	for( i = 0; i < rk_outputs.size(); i++ ) {
		ExportStatementBlock *loop1;
		if( (MeasurementGrid)measGrid == OFFLINE_GRID ) {
			loop1 = block;
			loop1->addStatement( std::string("for(") + index1.getName() + " = 0; " + index1.getName() + " < (int)" + numMeasVariables[i].get(0,index0) + "; " + index1.getName() + "++) {\n" );
			loop1->addStatement( tmp_index1 == numMeas[i]+index1 );
			for( j = 0; j < numStages; j++ ) {
				loop1->addStatement( rk_outH.getRow(j) == polynVariables[i].getElement( tmp_index1,j ) );
			}
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) {
				for( j = 0; j < numStages; j++ ) {
					loop1->addStatement( rk_out.getRow(j) == polynDerVariables[i].getElement( tmp_index1,j ) );
				}
			}
		}
		else { // ONLINE_GRID
			loop1 = block;
			loop1->addStatement( std::string(tmp_index2.getName()) + " = " + tmp_meas.get( 0,i ) + ";\n" );
			loop1->addStatement( std::string("for(") + index1.getName() + " = 0; " + index1.getName() + " < (int)" + tmp_index2.getName() + "; " + index1.getName() + "++) {\n" );
			loop1->addStatement( tmp_index1 == numMeas[i]+index1 );

			uint scale = grid.getNumIntervals();
			double scale2 = 1.0/grid.getNumIntervals();
			loop1->addStatement( time_tmp.getName() + " = " + toString(scale) + "*(" + gridVariables[i].get(0,tmp_index1) + "-" + toString(scale2) + "*" + index0.getName() + ");\n" );

			std::string h = toString((grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals());
			evaluatePolynomial( *loop1, rk_outH, time_tmp, h );
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) evaluateDerivedPolynomial( *loop1, rk_out, time_tmp );
		}

		DVector dependencyX, dependencyZ, dependencyDX;
		if( exportRhs || crsFormat ) {
			dependencyX = outputDependencies[i].getCols( 0,NX-1 ).sumRow();
			if( numXA_output(i) > 0 ) dependencyZ = outputDependencies[i].getCols( NX,NX+NXA-1 ).sumRow();
			if( numDX_output(i) > 0 ) dependencyDX = outputDependencies[i].getCols( NX+NXA+NU,NX+NXA+NU+NDX-1 ).sumRow();
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
		uint outputDim = numOutputs*(1+directions);
		loop1->addStatement( tmp_index1 == numMeas[i]*outputDim+index1*(numOutputs*(1+directions)) );
		for( j = 0; j < numOutputs; j++ ) {
			loop1->addStatement( rk_outputs[i].getCol( tmp_index1+j ) == rk_rhsOutputTemp.getCol( j ) );
		}
		loop1->addStatement( "}\n" );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::initializeDDMatrix( )
{
	uint i, j, k;
	DD = DMatrix( numStages, numStages );
	
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
	DVector cVec( numStages-1 );
	DVector products;
	coeffs = DMatrix( numStages, numStages );
	
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


DVector ImplicitRungeKuttaExport::computeCombinations( const DVector& cVec, uint index, uint numEls ) {
	uint k, l;
	DVector products;
	
	if( numEls == 0 ) {
		products = DVector(1);
		products(0) = 1;
		return products;
	}
	products = DVector();
	for( k = index; k < cVec.getDim()-numEls+1; k++ ) {
		DVector temp = computeCombinations( cVec, k+1, numEls-1 );
		for( l = 0; l < temp.getDim(); l++ ) {
			temp(l) *= cVec(k);
		}
		products.append(temp);
	}
	
	return products;
}


DMatrix ImplicitRungeKuttaExport::evaluatePolynomial( uint index )
{
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );
	DMatrix polynV(totalMeas[index],numStages);

	double scale2 = 1.0/(grid.getLastTime() - grid.getFirstTime());
	for( uint i = 0; i < totalMeas[index]; i++ ) {
		double time = outputGrids[index].getTime(i);
		if( (MeasurementGrid)measGrid == OFFLINE_GRID ) {
			uint interv = getIntegrationInterval( time );
			time = (time-scale2*grid.getTime(interv))/(scale2*(grid.getTime(interv+1)-grid.getTime(interv)));
		}
		polynV.setRow( i, evaluatePolynomial( time ) );
	}

	return polynV;
}


DVector ImplicitRungeKuttaExport::evaluatePolynomial( double time )
{
	uint i, j;
	DVector coeffsPolyn( numStages );
	
	for( j = 0; j < numStages; j++ ) {
		coeffsPolyn( j ) = 0.0;
		for( i = 0; i < numStages; i++ ) {
			coeffsPolyn( j ) += pow( time, static_cast<int>(numStages-i) )*coeffs( i,j );
		}
	} 
	
    return coeffsPolyn;
}


returnValue ImplicitRungeKuttaExport::evaluatePolynomial( ExportStatementBlock& block, const ExportVariable& variable, const ExportVariable& gridVariable, const std::string& h )
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
		block.addStatement( (std::string)variable.getFullName() + "[" + toString(j) + "] *= " + toString(h) + ";\n" );
	}

    return SUCCESSFUL_RETURN;
}


DMatrix ImplicitRungeKuttaExport::evaluateDerivedPolynomial( uint index )
{
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );
	DMatrix polynDerV(totalMeas[index],numStages);

	double scale2 = 1.0/(grid.getLastTime() - grid.getFirstTime());
	for( uint i = 0; i < totalMeas[index]; i++ ) {
		double time = outputGrids[index].getTime(i);
		if( (MeasurementGrid)measGrid == OFFLINE_GRID ) {
			uint interv = getIntegrationInterval( time );
			time = (time-scale2*grid.getTime(interv))/(scale2*(grid.getTime(interv+1)-grid.getTime(interv)));
		}
		polynDerV.setRow( i, evaluateDerivedPolynomial( time ) );
	}

	return polynDerV;
}


DVector ImplicitRungeKuttaExport::evaluateDerivedPolynomial( double time )
{
	uint i, j;
	DVector coeffsPolyn( numStages );

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
				block.addStatement( (std::string)variable.getFullName() + "[" + toString(i) + "] = (" + gridVariable.getName() + " - " + toString(cc(j)) + ")*" + toString(1/(cc(i)-cc(j))) + ";\n" );
			}
			else if( i != j ) {
				block.addStatement( (std::string)variable.getFullName() + "[" + toString(i) + "] *= (" + gridVariable.getName() + " - " + toString(cc(j)) + ")*" + toString(1/(cc(i)-cc(j))) + ";\n" );
			}
		}
	}

    return SUCCESSFUL_RETURN;
}


DVector ImplicitRungeKuttaExport::divideMeasurements( uint index )
{
	DVector meas( grid.getNumIntervals() );
	meas.setZero();

	for( uint i = 0; i < outputGrids[index].getNumIntervals(); i++ ) {
		uint interv = getIntegrationInterval( outputGrids[index].getTime(i) );
		meas(interv) = meas(interv)+1;
	}

	return meas;
}


returnValue ImplicitRungeKuttaExport::setup( )
{
	if( CONTINUOUS_OUTPUT && !equidistantControlGrid() ) return ACADOERROR( RET_INVALID_OPTION );

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	int intMode;
	userInteraction->get( IMPLICIT_INTEGRATOR_MODE,intMode ); 
	switch( (ImplicitIntegratorMode) intMode ) {
		case IFTR:
			REUSE = true;
			break;
		case IFT:
			REUSE = false;
			break;
		case LIFTED:
			REUSE = true;
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
	dx = DifferentialStateDerivative("", NDX, 1);

	AlgebraicState 	  dummy3;
	dummy3.clearStaticCounters();
	z = AlgebraicState("", NXA, 1);

	uint Xmax = NX1;
	if( NX2 > Xmax ) Xmax = NX2;
	if( NX3 > Xmax ) Xmax = NX3;
	NVARS2 = NX1+NX2+NXA+NU+NDX2;
	NVARS3 = 0;
	diffsDim = 0;
	inputDim = NX+NXA + NU + NOD;

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;

	rk_ttt = ExportVariable( "rk_ttt", 1, 1, REAL, structWspace, true );
	rk_xxx = ExportVariable( "rk_xxx", 1, inputDim+NDX+timeDep, REAL, structWspace );
	rk_kkk = ExportVariable( "rk_kkk", NX+NXA, numStages, REAL, structWspace );
	rk_A = ExportVariable( "rk_A", numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, structWspace );
	if ( (bool)debugMode == true && useOMP ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	else {
		debug_mat = ExportVariable( "debug_mat", numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, ACADO_VARIABLES );
	}
	rk_b = ExportVariable( "rk_b", numStages*(Xmax+NXA), 1, REAL, structWspace );
	rk_rhsTemp = ExportVariable( "rk_rhsTemp", NX+NXA+NDX, 1, REAL, structWspace );
	rk_diffsTemp2 = ExportVariable( "rk_diffsTemp2", 1, (NX2+NXA)*(NVARS2), REAL, structWspace );
	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, true );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
	integrate = ExportFunction( "integrate", rk_eta );
	uint i;
	for( i = 0; i < rk_outputs.size(); i++ ) {
		integrate.addArgument( rk_outputs[i] );
	}
	integrate.addArgument( reset_int );
	if( !equidistantControlGrid() || (ImplicitIntegratorMode) intMode == LIFTED ) integrate.addArgument( rk_index );
	integrate.setReturnValue( error_code );

	rk_eta.setDoc( "Working array of size " + toString( rk_eta.getDim() ) + " to pass the input values and return the results." );
	for( i = 0; i < rk_outputs.size(); i++ ) {
		rk_outputs[i].setDoc( "Working array of size " + toString( rk_outputs[ i ].getDim() ) + " to return the extra output results." );
	}
	reset_int.setDoc( "The internal memory of the integrator can be reset." );
	rk_index.setDoc( "Number of the shooting interval." );
	error_code.setDoc( "Status code of the integrator." );
	integrate.doc( "Performs the integration and sensitivity propagation for one shooting interval." );
	integrate.addLinebreak( );	// TO MAKE SURE IT GETS EXPORTED
	
	rhs_in = ExportVariable( "x", inputDim+NX, 1, REAL, ACADO_LOCAL );
	rhs_out = ExportVariable( "f", NX+NXA, 1, REAL, ACADO_LOCAL );
	fullRhs = ExportFunction( "full_rhs", rhs_in, rhs_out );
	rhs_in.setDoc( "The state and parameter values." );
	rhs_out.setDoc( "Right-hand side evaluation." );
	fullRhs.doc( "Evaluates the right-hand side of the full model." );
	fullRhs.addLinebreak( );	// FIX: TO MAKE SURE IT GETS EXPORTED

	int gradientUp;
	get( LIFTED_GRADIENT_UPDATE, gradientUp );
	bool gradientUpdate = (bool) gradientUp;

	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
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
			if( (ImplicitIntegratorMode) intMode == LIFTED ) {
				solver->init( (NX2+NXA)*numStages, NX+NU+1 );
			}
			else {
				solver->init( (NX2+NXA)*numStages );
			}
			if( (ExportSensitivityType)sensGen == SYMMETRIC || (ExportSensitivityType)sensGen == FORWARD_OVER_BACKWARD || (ExportSensitivityType)sensGen == BACKWARD || gradientUpdate ) solver->setTranspose( true ); // BACKWARD propagation
			solver->setReuse( true ); 	// IFTR method
			solver->setup();
			rk_auxSolver = solver->getGlobalExportVariable( 1 );
			break;
		case SIMPLIFIED_IRK_NEWTON:
			if( numStages == 3 || numStages == 4 ) {
				if( numStages == 3 ) solver = new ExportIRK3StageSimplifiedNewton( userInteraction,commonHeaderName );
				if( numStages == 4 ) solver = new ExportIRK4StageSimplifiedNewton( userInteraction,commonHeaderName );
				solver->init( NX2+NXA, NX+NU+1 );
				if( (ExportSensitivityType)sensGen == SYMMETRIC || (ExportSensitivityType)sensGen == FORWARD_OVER_BACKWARD || (ExportSensitivityType)sensGen == BACKWARD || gradientUpdate ) solver->setTranspose( true ); // BACKWARD propagation
				solver->setReuse( true ); 	// IFTR method
				solver->setup();
				rk_auxSolver = solver->getGlobalExportVariable( 2 );

				if( numStages == 3 ){
					ExportIRK3StageSimplifiedNewton* IRKsolver = dynamic_cast<ExportIRK3StageSimplifiedNewton *>(solver);
					IRKsolver->setEigenvalues(eig);
//					IRKsolver->setTransformations(simplified_transf1, simplified_transf2);
					IRKsolver->setTransformations(simplified_transf1, simplified_transf2, simplified_transf1_T, simplified_transf2_T);

					double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
					IRKsolver->setStepSize(h);
					if( NDX2 > 0 || NXA > 0 ) {
						IRKsolver->setImplicit( true );
						solver->setup();
					}
				}
				else if( numStages == 4 ) {
					ExportIRK4StageSimplifiedNewton* IRKsolver = dynamic_cast<ExportIRK4StageSimplifiedNewton *>(solver);
					IRKsolver->setEigenvalues(eig);
//					IRKsolver->setTransformations(simplified_transf1, simplified_transf2);
					IRKsolver->setTransformations(simplified_transf1, simplified_transf2, simplified_transf1_T, simplified_transf2_T);

					double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
					IRKsolver->setStepSize(h);
					if( NDX2 > 0 || NXA > 0 ) {
						IRKsolver->setImplicit( true );
						solver->setup();
					}
				}
			}
			else {
				return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
			}
			break;
		case SINGLE_IRK_NEWTON:
			if( numStages == 3 || numStages == 4 ) {
				if( numStages == 3 ) solver = new ExportIRK3StageSingleNewton( userInteraction,commonHeaderName );
				if( numStages == 4 ) solver = new ExportIRK4StageSingleNewton( userInteraction,commonHeaderName );
				solver->init( NX2+NXA, NX+NU+1 );
				if( (ExportSensitivityType)sensGen == SYMMETRIC || (ExportSensitivityType)sensGen == FORWARD_OVER_BACKWARD || (ExportSensitivityType)sensGen == BACKWARD || gradientUpdate ) solver->setTranspose( true ); // BACKWARD propagation
				solver->setReuse( true ); 	// IFTR method
				solver->setup();
				rk_auxSolver = solver->getGlobalExportVariable( 1 );

				if( numStages == 3 ) {
					ExportIRK3StageSingleNewton* IRKsolver = dynamic_cast<ExportIRK3StageSingleNewton *>(solver);
					IRKsolver->setTransformations(tau, low_tria, single_transf1, single_transf2, single_transf1_T, single_transf2_T);

					double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
					IRKsolver->setStepSize(h);
					if( NDX2 > 0 || NXA > 0 ) {
						IRKsolver->setImplicit( true );
						solver->setup();
					}
				}
				else if( numStages == 4 ) {
					ExportIRK4StageSingleNewton* IRKsolver = dynamic_cast<ExportIRK4StageSingleNewton *>(solver);
					IRKsolver->setTransformations(tau, low_tria, single_transf1, single_transf2, single_transf1_T, single_transf2_T);

					double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
					IRKsolver->setStepSize(h);
					if( NDX2 > 0 || NXA > 0 ) {
						IRKsolver->setImplicit( true );
						solver->setup();
					}
				}
			}
			else {
				return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
			}
			break;
		case HOUSEHOLDER_QR:
			solver = new ExportHouseholderQR( userInteraction,commonHeaderName );
			solver->init( (NX2+NXA)*numStages );
			solver->setReuse( true ); 	// IFTR method
			solver->setup();
			rk_auxSolver = solver->getGlobalExportVariable( 1 );
			if( (ImplicitIntegratorMode) intMode == LIFTED ) return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
			break;
		default:
			return ACADOERROR( RET_INVALID_OPTION );
		}
	}

    return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setEigenvalues( const DMatrix& _eig ) {
	eig = _eig;

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setSimplifiedTransformations( const DMatrix& _transf1, const DMatrix& _transf2, const DMatrix& _transf1_T, const DMatrix& _transf2_T ) {
	simplified_transf1 = _transf1;
	simplified_transf2 = _transf2;
	simplified_transf1_T = _transf1_T;
	simplified_transf2_T = _transf2_T;

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setSimplifiedTransformations( const DMatrix& _transf1, const DMatrix& _transf2 ) {
	simplified_transf1 = _transf1;
	simplified_transf2 = _transf2;

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setSingleTransformations( const double _tau, const DVector& _low_tria, const DMatrix& _transf1, const DMatrix& _transf2 ) {
	tau = _tau;
	low_tria = _low_tria;
	single_transf1 = _transf1;
	single_transf2 = _transf2;

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setSingleTransformations( const double _tau, const DVector& _low_tria, const DMatrix& _transf1, const DMatrix& _transf2, const DMatrix& _transf1_T, const DMatrix& _transf2_T ) {
	tau = _tau;
	low_tria = _low_tria;
	single_transf1 = _transf1;
	single_transf2 = _transf2;
	single_transf1_T = _transf1_T;
	single_transf2_T = _transf2_T;

	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setupOutput( const std::vector<Grid> outputGrids_, const std::vector<Expression> outputExpressions_ ) {

	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	sensGen = ( (ExportSensitivityType)sensGen != NO_SENSITIVITY );

	returnValue val = SUCCESSFUL_RETURN;
	CONTINUOUS_OUTPUT = true;
	if( outputGrids_.size() != outputExpressions_.size() ) return ACADOERROR( RET_INVALID_ARGUMENTS ); 
	outputGrids = outputGrids_;
	outputExpressions = outputExpressions_;

	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	numDX_output = DVector(outputGrids.size());
	numXA_output = DVector(outputGrids.size());
	numVARS_output = DVector(outputGrids.size());

	uint i;
	uint maxOutputs = 0;
	uint maxVARS = 0;
	rk_outputs.clear();
	outputs.clear();
	diffs_outputs.clear();
	for( i = 0; i < outputGrids.size(); i++ ) {
		uint numOutputs = outputExpressions_[i].getDim();
		uint outputDim = outputGrids[i].getNumIntervals( )*numOutputs*(NX+NU+1);
		if( !sensGen ) outputDim = outputGrids[i].getNumIntervals( )*numOutputs;
		
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
	
		ExportAcadoFunction OUTPUT, diffs_OUTPUT;
		val = val & OUTPUT.init( f_Output,std::string("acado_output")+toString(i)+"_rhs",NX,NXA,NU,NP,NDX,NOD ) & diffs_OUTPUT.init( g_Output,std::string("acado_output")+toString(i)+"_diffs",NX,NXA,NU,NP,NDX,NOD );
		
		ExportVariable rk_output( std::string("rk_output")+toString(i), 1, outputDim, REAL );
		rk_outputs.push_back( rk_output );
		
		outputs.push_back( OUTPUT );
		if( sensGen ) diffs_outputs.push_back( diffs_OUTPUT );

		DMatrix dependencyMat = outputExpressions[i].getDependencyPattern( x );
		if(z.getDim()>0)  dependencyMat.appendCols( outputExpressions[i].getDependencyPattern( z ) );
		if(u.getDim()>0)  dependencyMat.appendCols( outputExpressions[i].getDependencyPattern( u ) );
		if(dx.getDim()>0) dependencyMat.appendCols( outputExpressions[i].getDependencyPattern( dx ) );

		outputDependencies.push_back( dependencyMat );
		totalMeas.push_back( outputGrids[i].getNumIntervals() );

		// Export output grids
		ExportVariable gridVariable( (std::string)"gridOutput" + toString( i ), 1, totalMeas[i], REAL, ACADO_VARIABLES );
		gridVariables.push_back( gridVariable );
	}
	
	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	setup();
	rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, structWspace );
	if( sensGen ) rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(maxVARS), REAL, structWspace );
	rk_outH = ExportVariable( "rk_outH", numStages, 1, REAL, structWspace );
	rk_out = ExportVariable( "rk_out2", numStages, 1, REAL, structWspace );
	polynEvalVar = ExportVariable( "tmp_polyn", 1, 1, REAL, ACADO_LOCAL, true );

	return ( val );
}


returnValue ImplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<std::string> _outputNames,
									  	  const std::vector<std::string> _diffs_outputNames,
										  const std::vector<uint> _dims_output ) {

	if(rhs.isExternal() == true && (rk_outputs.size() + outputs.size() + diffs_outputs.size()) == 0) {
		int sensGen;
		get( DYNAMIC_SENSITIVITY, sensGen );
		sensGen = ( (ExportSensitivityType)sensGen != NO_SENSITIVITY );

		CONTINUOUS_OUTPUT = true;
		if( outputGrids_.size() != _outputNames.size() || outputGrids_.size() != _diffs_outputNames.size() || outputGrids_.size() != _dims_output.size() ) {
			return ACADOERROR( RET_INVALID_ARGUMENTS );
		}
		outputGrids = outputGrids_;
		num_outputs = _dims_output;

		int measGrid;
		get( MEASUREMENT_GRID, measGrid );

		numDX_output = DVector(outputGrids.size());
		numXA_output = DVector(outputGrids.size());
		numVARS_output = DVector(outputGrids.size());

		uint i;
		uint maxOutputs = 0;
		rk_outputs.clear();
		outputs.clear();
		diffs_outputs.clear();
		for( i = 0; i < outputGrids.size(); i++ ) {
			uint numOutputs = num_outputs[i];
			uint outputDim = outputGrids[i].getNumIntervals( )*numOutputs*(NX+NU+1);
			if( !sensGen ) outputDim = outputGrids[i].getNumIntervals( )*numOutputs;

			if( numOutputs > maxOutputs ) maxOutputs = numOutputs;

			numDX_output(i) = NDX;	// worst-case scenario
			numXA_output(i) = NXA;	// worst-case scenario
			numVARS_output(i) = NX+NXA+NU+NDX;

			ExportAcadoFunction OUTPUT(_outputNames[i]);
			ExportAcadoFunction diffs_OUTPUT(_diffs_outputNames[i]);

			outputs.push_back( OUTPUT );
			if( sensGen ) diffs_outputs.push_back( diffs_OUTPUT );

			ExportVariable rk_output( std::string("rk_output")+toString(i), 1, outputDim, REAL );
			rk_outputs.push_back( rk_output );

			totalMeas.push_back( outputGrids[i].getNumIntervals() );

			// Export output grids
			ExportVariable gridVariable( (std::string)"gridOutput" + toString(i), 1, totalMeas[i], REAL, ACADO_VARIABLES );
			gridVariables.push_back( gridVariable );
		}
		uint maxVARS = NX+NXA+NU+NDX;

		int useOMP;
		get(CG_USE_OPENMP, useOMP);
		ExportStruct structWspace;
		structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

		setup();
		rk_rhsOutputTemp = ExportVariable( "rk_rhsOutputTemp", 1, maxOutputs, REAL, structWspace );
		if( sensGen ) rk_diffsOutputTemp = ExportVariable( "rk_diffsOutputTemp", 1, maxOutputs*(maxVARS), REAL, structWspace );
		rk_outH = ExportVariable( "rk_outH", numStages, 1, REAL, structWspace );
		rk_out = ExportVariable( "rk_out2", numStages, 1, REAL, structWspace );
		polynEvalVar = ExportVariable( "tmp_polyn", 1, 1, REAL, ACADO_LOCAL, true );

		exportRhs = false;
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}


	return SUCCESSFUL_RETURN;
}


returnValue ImplicitRungeKuttaExport::setupOutput(  const std::vector<Grid> outputGrids_,
									  	  const std::vector<std::string> _outputNames,
									  	  const std::vector<std::string> _diffs_outputNames,
										  const std::vector<uint> _dims_output,
										  const std::vector<DMatrix> _outputDependencies ) {

	outputDependencies = _outputDependencies;
	crsFormat = true;

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
			DMatrix polynV = evaluatePolynomial( i );
			DMatrix polynDerV = evaluateDerivedPolynomial( i );
			DVector measurements = divideMeasurements( i );

			double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
			ExportVariable polynVariable = ExportVariable( (std::string)"polynOutput" + toString( i ), polynV*=h, STATIC_CONST_REAL );
			code.addDeclaration( polynVariable );
			polynVariable = ExportVariable( (std::string)"polynOutput" + toString( i ), totalMeas[i], numStages, STATIC_CONST_REAL );
			polynVariables.push_back( polynVariable );

			if( NXA > 0 || NDX > 0 ) {
				ExportVariable polynDerVariable( (std::string)"polynDerOutput" + toString( i ), polynDerV, STATIC_CONST_REAL );
				code.addDeclaration( polynDerVariable );
				polynDerVariable = ExportVariable( (std::string)"polynDerOutput" + toString( i ), totalMeas[i], numStages, STATIC_CONST_REAL );
				polynDerVariables.push_back( polynDerVariable );
			}

			ExportVariable numMeasVariable( (std::string)"numMeas" + toString( i ), measurements, STATIC_CONST_INT );
			if( (MeasurementGrid)measGrid == OFFLINE_GRID ) {
				code.addDeclaration( numMeasVariable );
			}
			numMeasVariables.push_back( numMeasVariable );
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
	num_outputs = arg.num_outputs;
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
	
	eig = arg.eig;
	simplified_transf1 = arg.simplified_transf1;
	simplified_transf2 = arg.simplified_transf2;
	simplified_transf1_T = arg.simplified_transf1_T;
	simplified_transf2_T = arg.simplified_transf2_T;

	tau = arg.tau;
	low_tria = arg.low_tria;
	single_transf1 = arg.single_transf1;
	single_transf2 = arg.single_transf2;
	single_transf1_T = arg.single_transf1_T;
	single_transf2_T = arg.single_transf2_T;

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
