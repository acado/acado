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
 *    \file src/code_generation/integrators/irk_forward_export.cpp
 *    \author Rien Quirynen
 *    \date 2013
 */

#include <acado/code_generation/integrators/irk_export.hpp>
#include <acado/code_generation/integrators/irk_forward_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ForwardIRKExport::ForwardIRKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ImplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
}

ForwardIRKExport::ForwardIRKExport( const ForwardIRKExport& arg ) : ImplicitRungeKuttaExport( arg )
{
}


ForwardIRKExport::~ForwardIRKExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}


ForwardIRKExport& ForwardIRKExport::operator=( const ForwardIRKExport& arg ){

    if( this != &arg ){

		ImplicitRungeKuttaExport::operator=( arg );
		copy( arg );
    }
    return *this;
}


ExportVariable ForwardIRKExport::getAuxVariable() const
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


returnValue ForwardIRKExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	ImplicitRungeKuttaExport::getDataDeclarations( declarations, dataStruct );
	
	declarations.addDeclaration( rk_diffK,dataStruct );

	declarations.addDeclaration( rk_diffsTemp3,dataStruct );

//	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		declarations.addDeclaration( rk_diffsPrev1,dataStruct );
		declarations.addDeclaration( rk_diffsPrev2,dataStruct );
		declarations.addDeclaration( rk_diffsPrev3,dataStruct );
//	}

	declarations.addDeclaration( rk_diffsNew1,dataStruct );
	declarations.addDeclaration( rk_diffsNew2,dataStruct );
	declarations.addDeclaration( rk_diffsNew3,dataStruct );
	
	if( CONTINUOUS_OUTPUT ) {
		declarations.addDeclaration( rk_diffsOutputTemp,dataStruct );
	}

    return SUCCESSFUL_RETURN;
}


returnValue ForwardIRKExport::getFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	ImplicitRungeKuttaExport::getFunctionDeclarations( declarations );

    return SUCCESSFUL_RETURN;
}


returnValue ForwardIRKExport::getCode(	ExportStatementBlock& code )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	if ( (ExportSensitivityType)sensGen != FORWARD ) ACADOERROR( RET_INVALID_OPTION );

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
				<< max.getFullName() << ", "
				<< rk_ttt.getFullName() << ", "
				<< rk_xxx.getFullName() << ", "
				<< rk_kkk.getFullName() << ", "
				<< rk_diffK.getFullName() << ", "
				<< rk_rhsTemp.getFullName() << ", "
				<< rk_auxSolver.getFullName();
		if( NX1 > 0 ) {
			if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) s << ", " << rk_diffsPrev1.getFullName();
			s << ", " << rk_diffsNew1.getFullName();
		}
		if( NX2 > 0 || NXA > 0 ) {
			s << ", " << rk_A.getFullName();
			s << ", " << rk_b.getFullName();
			if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) s << ", " << rk_diffsPrev2.getFullName();
			s << ", " << rk_diffsNew2.getFullName();
			s << ", " << rk_diffsTemp2.getFullName();
			solver->appendVariableNames( s );
		}
		if( NX3 > 0 ) {
			if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) s << ", " << rk_diffsPrev3.getFullName();
			s << ", " << rk_diffsNew3.getFullName();
			s << ", " << rk_diffsTemp3.getFullName();
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
	if( NX2 > 0 || NXA > 0 ) solver->getCode( code );
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
	if( (inputDim-diffsDim) > NX+NXA ) {
		integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
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

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		// Set rk_diffsPrev:
		loop->addStatement( std::string("if( run > 0 ) {\n") );
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
		loop->addStatement( std::string("}\n") );
	}

	// PART 1: The linear input system
	prepareInputSystem( code );
	solveInputSystem( loop, i, run1, j, tmp_index1, Ah );

	// PART 2: The fully implicit system
	solveImplicitSystem( loop, i, run1, j, tmp_index1, ExportIndex(0), Ah, C, determinant, true );

	// PART 3: The linear output system
	prepareOutputSystem( code );
	solveOutputSystem( loop, i, run1, j, tmp_index1, Ah, true );

	// generate continuous OUTPUT:
	generateOutput( loop, run, i, tmp_index2, tmp_index3, tmp_meas, time_tmp, NX+NU );

	// DERIVATIVES wrt the states (IFT):
	if( NX1 > 0 ) {
		ExportForLoop loop4( run1,0,NX1 );
		// PART 1: The linear input system
		sensitivitiesInputSystem( &loop4, run1, i, Bh, true );
		// PART 2: The fully implicit system
		sensitivitiesImplicitSystem( &loop4, run1, i, j, tmp_index1, tmp_index2, Ah, Bh, determinant, true, 1 );
		// PART 3: The linear output system
		sensitivitiesOutputSystem( &loop4, run1, i, j, k, tmp_index1, tmp_index2, Ah, Bh, true, 1 );
		// generate sensitivities wrt states for continuous output:
		sensitivitiesOutputs( &loop4, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, time_tmp, true, 0 );
		loop->addStatement( loop4 );
	}
	if( NX2 > 0 ) {
		ExportForLoop loop4( run1,NX1,NX1+NX2 );
		// PART 2: The fully implicit system
		sensitivitiesImplicitSystem( &loop4, run1, i, j, tmp_index1, tmp_index2, Ah, Bh, determinant, true, 2 );
		// PART 3: The linear output system
		sensitivitiesOutputSystem( &loop4, run1, i, j, k, tmp_index1, tmp_index2, Ah, Bh, true, 2 );
		// generate sensitivities wrt states for continuous output:
		sensitivitiesOutputs( &loop4, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, time_tmp, true, NX1 );
		loop->addStatement( loop4 );
	}
	if( NX3 > 0 ) {
		ExportForLoop loop4( run1,NX1+NX2,NX );
		// PART 3: The linear output system
		sensitivitiesOutputSystem( &loop4, run1, i, j, k, tmp_index1, tmp_index2, Ah, Bh, true, 3 );
		// generate sensitivities wrt states for continuous output:
		sensitivitiesOutputs( &loop4, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, time_tmp, true, NX1+NX2 );
		loop->addStatement( loop4 );
	}


	// DERIVATIVES wrt the control inputs (IFT):
	if( NU > 0 ) {
		ExportForLoop loop5( run1,0,NU );
		// PART 1: The linear input system
		sensitivitiesInputSystem( &loop5, run1, i, Bh, false );
		// PART 2: The fully implicit system
		sensitivitiesImplicitSystem( &loop5, run1, i, j, tmp_index1, tmp_index2, Ah, Bh, determinant, false, 0 );
		// PART 3: The linear output system
		sensitivitiesOutputSystem( &loop5, run1, i, j, k, tmp_index1, tmp_index2, Ah, Bh, false, 0 );
		// generate sensitivities wrt controls for continuous output:
		sensitivitiesOutputs( &loop5, run, run1, i, tmp_index1, tmp_index2, tmp_index3, tmp_meas, time_tmp, false, 0 );
		loop->addStatement( loop5 );
	}

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


	// Computation of the sensitivities using the CHAIN RULE:
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( std::string( "if( run == 0 ) {\n" ) );
	}
	// PART 1
	updateInputSystem(loop, i, j, tmp_index2);
	// PART 2
	updateImplicitSystem(loop, i, j, tmp_index2);
	// PART 3
	updateOutputSystem(loop, i, j, tmp_index2);

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( std::string( "}\n" ) );
		loop->addStatement( std::string( "else {\n" ) );
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
		loop->addStatement( std::string( "}\n" ) );
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
    // PART 1
    if( NX1 > 0 ) {
    	DMatrix zeroR = zeros<double>(1, NX2+NX3);
    	ExportForLoop loop1( i,0,NX1 );
    	loop1.addStatement( rk_eta.getCols( i*NX+NX+NXA+NX1,i*NX+NX+NXA+NX ) == zeroR );
    	integrate.addStatement( loop1 );
    }
    // PART 2
    DMatrix zeroR = zeros<double>(1, NX3);
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


returnValue ForwardIRKExport::propagateOutputs(	ExportStatementBlock* block, const ExportIndex& index, const ExportIndex& index0, const ExportIndex& index1,
															const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
															const ExportIndex& tmp_index3, const ExportIndex& tmp_index4, const ExportVariable& tmp_meas )
{
	uint i;
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	// chain rule for the sensitivities of the continuous output:
	for( i = 0; i < rk_outputs.size(); i++ ) {
		ExportStatementBlock *loop01;
		if( (MeasurementGrid)measGrid == OFFLINE_GRID ) {
			loop01 = block;
			loop01->addStatement( std::string("for(") + index0.getName() + " = 0; " + index0.getName() + " < (int)" + numMeasVariables[i].get(0,index) + "; " + index0.getName() + "++) {\n" );
		}
		else { // ONLINE_GRID
			loop01 = block;
			loop01->addStatement( std::string("for(") + index0.getName() + " = 0; " + index0.getName() + " < (int)" + tmp_meas.get(0,i) + "; " + index0.getName() + "++) {\n" );
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
		loop05.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev1.getElement( index3,index2 ) );
		loop04.addStatement( loop05 );
		ExportForLoop loop06( index3,NX1,NX1+NX2 );
		loop06.addStatement( tmp_index4 == index1*NX+index3 );
		loop06.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev2.getElement( index3-NX1,index2 ) );
		loop04.addStatement( loop06 );
		ExportForLoop loop062( index3,NX1+NX2,NX );
		loop062.addStatement( tmp_index4 == index1*NX+index3 );
		loop062.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev3.getElement( index3-NX1-NX2,index2 ) );
		loop04.addStatement( loop062 );
		loop03.addStatement( loop04 );

		ExportForLoop loop07( index2,NX1,NX1+NX2 );
		loop07.addStatement( tmp_index3 == tmp_index2+index2 );
		loop07.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) == 0.0 );
		ExportForLoop loop08( index3,NX1,NX1+NX2 );
		loop08.addStatement( tmp_index4 == index1*NX+index3 );
		loop08.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev2.getElement( index3-NX1,index2 ) );
		loop07.addStatement( loop08 );
		ExportForLoop loop082( index3,NX1+NX2,NX );
		loop082.addStatement( tmp_index4 == index1*NX+index3 );
		loop082.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev3.getElement( index3-NX1-NX2,index2 ) );
		loop07.addStatement( loop082 );
		loop03.addStatement( loop07 );

		ExportForLoop loop09( index2,NX1+NX2,NX );
		loop09.addStatement( tmp_index3 == tmp_index2+index2 );
		loop09.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) == 0.0 );
		ExportForLoop loop10( index3,NX1+NX2,NX );
		loop10.addStatement( tmp_index4 == index1*NX+index3 );
		loop10.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev3.getElement( index3-NX1-NX2,index2 ) );
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
			loop12.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs*(1+NX) ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev1.getElement( index3,NX1+index2 ) );
			loop11.addStatement( loop12 );
			ExportForLoop loop13( index3,NX1,NX1+NX2 );
			loop13.addStatement( tmp_index4 == index1*NX+index3 );
			loop13.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs*(1+NX) ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev2.getElement( index3-NX1,NX1+NX2+index2 ) );
			loop11.addStatement( loop13 );
			ExportForLoop loop132( index3,NX1+NX2,NX );
			loop132.addStatement( tmp_index4 == index1*NX+index3 );
			loop132.addStatement( rk_outputs[i].getCol( tmp_index3+numOutputs*(1+NX) ) += rk_diffsOutputTemp.getCol( tmp_index4 )*rk_diffsPrev3.getElement( index3-NX1-NX2,NX+index2 ) );
			loop11.addStatement( loop132 );
			loop03.addStatement( loop11 );
		}

		loop01->addStatement( loop03 );
		loop01->addStatement( "}\n" );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardIRKExport::prepareInputSystem(	ExportStatementBlock& code )
{
	if( NX1 > 0 ) {
		DMatrix mat1 = formMatrix( M11, A11 );
		rk_mat1 = ExportVariable( "rk_mat1", mat1, STATIC_CONST_REAL );
		code.addDeclaration( rk_mat1 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_mat1 = ExportVariable( "rk_mat1", numStages*NX1, numStages*NX1, STATIC_CONST_REAL, ACADO_LOCAL );

		DMatrix sens = zeros<double>(NX1*(NX1+NU), numStages);
		uint i, j, k;
		for( i = 0; i < NX1; i++ ) {
			DVector vec(NX1*numStages);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					vec(j*NX1+k) = A11(k,i);
				}
			}
			DVector sol = mat1*vec;
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					sens(i*NX1+k,j) = sol(j*NX1+k);
				}
			}
		}
		for( i = 0; i < NU; i++ ) {
			DVector vec(NX1*numStages);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX1; k++ ) {
					vec(j*NX1+k) = B11(k,i);
				}
			}
			DVector sol = mat1*vec;
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


returnValue ForwardIRKExport::prepareOutputSystem(	ExportStatementBlock& code )
{
	if( NX3 > 0 ) {
		DMatrix mat3 = formMatrix( M33, A33 );
		rk_mat3 = ExportVariable( "rk_mat3", mat3, STATIC_CONST_REAL );
		code.addDeclaration( rk_mat3 );
		// TODO: Ask Milan why this does NOT work properly !!
		rk_mat3 = ExportVariable( "rk_mat3", numStages*NX3, numStages*NX3, STATIC_CONST_REAL, ACADO_LOCAL );

		DMatrix sens = zeros<double>(NX3*NX3, numStages);
		uint i, j, k;
		for( i = 0; i < NX3; i++ ) {
			DVector vec(NX3*numStages);
			for( j = 0; j < numStages; j++ ) {
				for( k = 0; k < NX3; k++ ) {
					vec(j*NX3+k) = A33(k,i);
				}
			}
			DVector sol = mat3*vec;
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


returnValue ForwardIRKExport::sensitivitiesInputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportVariable& Bh, bool STATES )
{
	if( NX1 > 0 ) {
		// update rk_diffK with the new sensitivities:
		if( STATES ) 	block->addStatement( rk_diffK.getRows(0,NX1) == rk_dk1.getRows(index1*NX1,index1*NX1+NX1) );
		else			block->addStatement( rk_diffK.getRows(0,NX1) == rk_dk1.getRows(index1*NX1+NX1*NX1,index1*NX1+NX1+NX1*NX1) );
		// update rk_diffsNew with the new sensitivities:
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( std::string( "if( run == 0 ) {\n" ) );
		ExportForLoop loop3( index2,0,NX1 );
		if( STATES ) loop3.addStatement( std::string(rk_diffsNew1.get( index2,index1 )) + " = (" + index2.getName() + " == " + index1.getName() + ");\n" );

		if( STATES ) loop3.addStatement( rk_diffsNew1.getElement( index2,index1 ) += rk_diffK.getRow( index2 )*Bh );
		else		 loop3.addStatement( rk_diffsNew1.getElement( index2,index1+NX1 ) == rk_diffK.getRow( index2 )*Bh );
		block->addStatement( loop3 );
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( std::string( "}\n" ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardIRKExport::sensitivitiesImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportVariable& Ah, const ExportVariable& Bh, const ExportVariable& det, bool STATES, uint number )
{
	if( NX2 > 0 ) {
		DMatrix zeroM = zeros<double>( NX2+NXA,1 );
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );
		uint i;

		ExportForLoop loop1( index2,0,numStages );
		if( STATES && number == 1 ) {
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( std::string(rk_rhsTemp.get( index3,0 )) + " = -(" + index3.getName() + " == " + index1.getName() + ");\n" );
			for( i = 0; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) -= rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
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
				loop1.addStatement( rk_b.getRow( index2*(NX2+NXA)+i ) == zeroM.getRow( 0 ) - rk_diffsTemp2.getElement( index2,index1+i*(NVARS2) ) );
			}
		}
		else { // ~= STATES
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getElement( index3,0 )*Ah.getElement(index2,0) );
			for( i = 1; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,0,NX2+NXA );
			loop3.addStatement( tmp_index1 == index2*(NX2+NXA)+index3 );
			loop3.addStatement( tmp_index2 == index1+index3*(NVARS2) );
			loop3.addStatement( rk_b.getRow( tmp_index1 ) == zeroM.getRow( 0 ) - rk_diffsTemp2.getElement( index2,tmp_index2+NX1+NX2+NXA ) );
			loop3.addStatement( rk_b.getRow( tmp_index1 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2),index3*(NVARS2)+NX1 )*rk_rhsTemp.getRows(0,NX1) );
			if( NDX2 > 0 ) {
				loop3.addStatement( rk_b.getRow( tmp_index1 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2)+NVARS2-NX1-NX2,index3*(NVARS2)+NVARS2-NX2 )*rk_diffK.getSubMatrix( 0,NX1,index2,index2+1 ) );
			}
			loop1.addStatement( loop3 );
		}
		block->addStatement( loop1 );
		if( STATES && (number == 1 || NX1 == 0) ) {
			block->addStatement( std::string( "if( 0 == " ) + index1.getName() + " ) {\n" );	// factorization of the new matrix rk_A not yet calculated!
			block->addStatement( det.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_b.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
			block->addStatement( std::string( "}\n else {\n" ) );
		}
		block->addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		if( STATES && (number == 1 || NX1 == 0) ) block->addStatement( std::string( "}\n" ) );
		// update rk_diffK with the new sensitivities:
		ExportForLoop loop2( index2,0,numStages );
		loop2.addStatement( rk_diffK.getSubMatrix(NX1,NX1+NX2,index2,index2+1) == rk_b.getRows(index2*NX2,index2*NX2+NX2) );
		loop2.addStatement( rk_diffK.getSubMatrix(NX,NX+NXA,index2,index2+1) == rk_b.getRows(numStages*NX2+index2*NXA,numStages*NX2+index2*NXA+NXA) );
		block->addStatement( loop2 );
		// update rk_diffsNew with the new sensitivities:
		ExportForLoop loop3( index2,0,NX2 );
		if( STATES && number == 2 ) loop3.addStatement( std::string(rk_diffsNew2.get( index2,index1 )) + " = (" + index2.getName() + " == " + index1.getName() + "-" + toString(NX1) + ");\n" );

		if( STATES && number == 2 ) loop3.addStatement( rk_diffsNew2.getElement( index2,index1 ) += rk_diffK.getRow( NX1+index2 )*Bh );
		else if( STATES )	loop3.addStatement( rk_diffsNew2.getElement( index2,index1 ) == rk_diffK.getRow( NX1+index2 )*Bh );
		else		 		loop3.addStatement( rk_diffsNew2.getElement( index2,index1+NX1+NX2 ) == rk_diffK.getRow( NX1+index2 )*Bh );
		block->addStatement( loop3 );
		if( NXA > 0 ) {
			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
				block->addStatement( std::string("if( run == 0 ) {\n") );
			}
			ExportForLoop loop4( index2,0,NXA );
			if( STATES ) loop4.addStatement( rk_diffsNew2.getElement( index2+NX2,index1 ) == rk_diffK.getRow( NX+index2 )*tempCoefs );
			else 		 loop4.addStatement( rk_diffsNew2.getElement( index2+NX2,index1+NX1+NX2 ) == rk_diffK.getRow( NX+index2 )*tempCoefs );
			block->addStatement( loop4 );
			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
				block->addStatement( std::string("}\n") );
			}
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardIRKExport::sensitivitiesOutputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& index4, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportVariable& Ah, const ExportVariable& Bh, bool STATES, uint number )
{
	if( NX3 > 0 ) {
		uint i;
		ExportForLoop loop1( index2,0,numStages );
		if( STATES && number == 1 ) {
			ExportForLoop loop2( index3,0,NX1 );
			loop2.addStatement( std::string(rk_rhsTemp.get( index3,0 )) + " = (" + index3.getName() + " == " + index1.getName() + ");\n" );
			for( i = 0; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,NX1,NX1+NX2 );
			loop3.addStatement( rk_rhsTemp.getRow( index3 ) == 0.0 );
			for( i = 0; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
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
			loop3.addStatement( std::string(rk_rhsTemp.get( index3,0 )) + " = (" + index3.getName() + " == " + index1.getName() + ");\n" );
			for( i = 0; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
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
			loop2.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getElement( index3,0 )*Ah.getElement(index2,0) );
			for( i = 1; i < numStages; i++ ) {
				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,NX1,NX1+NX2 );
			loop3.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getElement( index3,0 )*Ah.getElement(index2,0) );
			for( i = 1; i < numStages; i++ ) {
				loop3.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
			}
			loop1.addStatement( loop3 );
			ExportForLoop loop4( index3,0,NX3 );
			loop4.addStatement( tmp_index1 == index2*NX3+index3 );
			loop4.addStatement( tmp_index2 == index1+index3*(NVARS3) );
			loop4.addStatement( rk_b.getRow( tmp_index1 ) == rk_diffsTemp3.getElement( index2,tmp_index2+NX1+NX2+NXA3 ) );
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
			loop5.addStatement( rk_diffK.getElement(NX1+NX2+index3,index2) == rk_mat3.getElement(tmp_index1,0)*rk_b.getRow(0) );
			ExportForLoop loop6( index4,1,numStages*NX3 );
			loop6.addStatement( rk_diffK.getElement(NX1+NX2+index3,index2) += rk_mat3.getElement(tmp_index1,index4)*rk_b.getRow(index4) );
			loop5.addStatement(loop6);
			loop4.addStatement(loop5);
			block->addStatement(loop4);
		}
		// update rk_diffsNew with the new sensitivities:
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( std::string( "if( run == 0 ) {\n" ) );
		ExportForLoop loop8( index2,0,NX3 );
		if( STATES && number == 3 ) loop8.addStatement( std::string(rk_diffsNew3.get( index2,index1 )) + " = (" + index2.getName() + " == " + index1.getName() + "-" + toString(NX1+NX2) + ");\n" );

		if( STATES && number == 3 ) loop8.addStatement( rk_diffsNew3.getElement( index2,index1 ) += rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		else if( STATES )	loop8.addStatement( rk_diffsNew3.getElement( index2,index1 ) == rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		else		 		loop8.addStatement( rk_diffsNew3.getElement( index2,index1+NX ) == rk_diffK.getRow( NX1+NX2+index2 )*Bh );
		block->addStatement( loop8 );
		if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) block->addStatement( std::string( "}\n" ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardIRKExport::sensitivitiesOutputs( ExportStatementBlock* block, const ExportIndex& index0,
		const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
		const ExportIndex& tmp_index3, const ExportVariable& tmp_meas, const ExportVariable& time_tmp, bool STATES, uint base )
{
	uint i, j, k;
	int measGrid;
	get( MEASUREMENT_GRID, measGrid );

	for( i = 0; i < rk_outputs.size(); i++ ) {
		uint NVARS = numVARS_output(i);
		ExportStatementBlock *loop;
		if( (MeasurementGrid)measGrid == OFFLINE_GRID ) {
			loop = block;
			loop->addStatement( std::string("for(") + index2.getName() + " = 0; " + index2.getName() + " < (int)" + numMeasVariables[i].get(0,index0) + "; " + index2.getName() + "++) {\n" );
			loop->addStatement( tmp_index2 == numMeas[i]+index2 );
			for( j = 0; j < numStages; j++ ) {
				loop->addStatement( rk_outH.getRow(j) == polynVariables[i].getElement( tmp_index2,j ) );
			}
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) {
				for( j = 0; j < numStages; j++ ) {
					loop->addStatement( rk_out.getRow(j) == polynDerVariables[i].getElement( tmp_index2,j ) );
				}
			}
		}
		else { // ONLINE_GRID
			loop = block;
			loop->addStatement( std::string(tmp_index3.getName()) + " = " + tmp_meas.get( 0,i ) + ";\n" );
			loop->addStatement( std::string("for(") + index2.getName() + " = 0; " + index2.getName() + " < (int)" + tmp_index3.getName() + "; " + index2.getName() + "++) {\n" );
			loop->addStatement( tmp_index2 == numMeas[i]+index2 );

			uint scale = grid.getNumIntervals();
			double scale2 = 1.0/grid.getNumIntervals();
			loop->addStatement( time_tmp.getName() + " = " + toString(scale) + "*(" + gridVariables[i].get(0,tmp_index2) + "-" + toString(scale2) + "*" + index0.getName() + ");\n" );

			std::string h = toString((grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals());
			evaluatePolynomial( *loop, rk_outH, time_tmp, h );
			if( numXA_output(i) > 0 || numDX_output(i) > 0 ) evaluateDerivedPolynomial( *loop, rk_out, time_tmp );
		}

		DVector dependencyX, dependencyZ, dependencyDX;
		if( exportRhs || crsFormat ) {
			dependencyX = outputDependencies[i].getCols( 0,NX-1 ).sumRow();
			if( numXA_output(i) > 0 ) dependencyZ = outputDependencies[i].getCols( NX,NX+NXA-1 ).sumRow();
			if( numDX_output(i) > 0 ) dependencyDX = outputDependencies[i].getCols( NX+NXA+NU,NX+NXA+NU+NDX-1 ).sumRow();
		}
		for( j = 0; j < NX; j++ ) {
			if( (!exportRhs && !crsFormat) || acadoRoundAway(dependencyX(j)) != 0 ) {
				if( STATES && j >= base ) {
					loop->addStatement( std::string(rk_rhsTemp.get( j,0 )) + " = (" + toString(j) + " == " + index1.getName() + ");\n" );
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
		loop->addStatement( "}\n" );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardIRKExport::setup( )
{
	ImplicitRungeKuttaExport::setup();

	NVARS3 = NX1+NX2+NXA3+NU+NDX3;
	diffsDim = (NX+NXA)*(NX+NU);
	inputDim = (NX+NXA)*(NX+NU+1) + NU + NOD;

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	rk_diffK = ExportVariable( "rk_diffK", NX+NXA, numStages, REAL, structWspace );
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
	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );

    return SUCCESSFUL_RETURN;
}



// PROTECTED:


CLOSE_NAMESPACE_ACADO

// end of file.
