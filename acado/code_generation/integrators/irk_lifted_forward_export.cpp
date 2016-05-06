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
 *    \file src/code_generation/integrators/irk_lifted_forward_export.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/integrators/irk_export.hpp>
#include <acado/code_generation/integrators/irk_lifted_forward_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ForwardLiftedIRKExport::ForwardLiftedIRKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ForwardIRKExport( _userInteraction,_commonHeaderName )
{
}

ForwardLiftedIRKExport::ForwardLiftedIRKExport( const ForwardLiftedIRKExport& arg ) : ForwardIRKExport( arg )
{
}


ForwardLiftedIRKExport::~ForwardLiftedIRKExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}


ForwardLiftedIRKExport& ForwardLiftedIRKExport::operator=( const ForwardLiftedIRKExport& arg ){

    if( this != &arg ){

    	ForwardIRKExport::operator=( arg );
		copy( arg );
    }
    return *this;
}


ExportVariable ForwardLiftedIRKExport::getAuxVariable() const
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
		if( forward_sweep.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = forward_sweep.getGlobalExportVariable();
		}
		if( adjoint_sweep.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = adjoint_sweep.getGlobalExportVariable();
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


returnValue ForwardLiftedIRKExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	ForwardIRKExport::getDataDeclarations( declarations, dataStruct );
	
	declarations.addDeclaration( rk_seed,dataStruct );
	declarations.addDeclaration( rk_I,dataStruct );
	declarations.addDeclaration( rk_diffSweep,dataStruct );
	declarations.addDeclaration( rk_stageValues,dataStruct );

	declarations.addDeclaration( rk_Xprev,dataStruct );
	declarations.addDeclaration( rk_Uprev,dataStruct );
	declarations.addDeclaration( rk_delta,dataStruct );

	declarations.addDeclaration( rk_xxx_lin,dataStruct );
	declarations.addDeclaration( rk_Khat_traj,dataStruct );
	declarations.addDeclaration( rk_Xhat_traj,dataStruct );

	declarations.addDeclaration( rk_diffK_local,dataStruct );

	declarations.addDeclaration( rk_b_trans,dataStruct );

	declarations.addDeclaration( rk_adj_traj,dataStruct );

	declarations.addDeclaration( rk_adj_diffs_tmp,dataStruct );
	declarations.addDeclaration( rk_seed2,dataStruct );
	declarations.addDeclaration( rk_xxx_traj,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::getFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	ForwardIRKExport::getFunctionDeclarations( declarations );



    return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::setDifferentialEquation(	const Expression& rhs_ )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
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

		DifferentialEquation h;
		if( (ExportSensitivityType)sensGen == INEXACT ) { // ONLY FOR INEXACT LIFTING
			DifferentialState sX("", NX,NX+NU);
			DifferentialState dKX("", NDX2,NX+NU);
			DifferentialState dKZ("", NXA,NX+NU);

			Expression tmp = zeros<double>(NX+NXA,NX);
			tmp.appendCols(forwardDerivative( rhs_, u ));

			if(NXA > 0 && NDX2 == 0) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);

			// forward sweep
			if(NXA == 0 && NDX2 == 0) {
				h << multipleForwardDerivative( rhs_, x, sX ) + tmp;
			}
			else {
				Expression tmp2 = tmp + multipleForwardDerivative( rhs_, dx, dKX );
				Expression tmp3 = tmp2 + multipleForwardDerivative( rhs_, z, dKZ );
				h << multipleForwardDerivative( rhs_, x, sX ) + tmp3;
			}

		}

		int gradientUp;
		get( LIFTED_GRADIENT_UPDATE, gradientUp );
		bool gradientUpdate = (bool) gradientUp;

		DifferentialEquation backward;
		if( gradientUpdate ) {
			if(NXA > 0 || NDX2 > 0) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);

			Expression arg;
			arg << x;
			arg << u;

			DifferentialState lambda("", NX,1);
			backward << backwardDerivative( rhs_, arg, lambda );
		}

		if( f.getNT() > 0 ) timeDependant = true;

		return (rhs.init( f,"acado_rhs",NX,NXA,NU,NP,NDX,NOD ) &
				diffs_rhs.init( g,"acado_diffs",NX,NXA,NU,NP,NDX,NOD ) &
				forward_sweep.init( h,"acado_forward",NX+(NX+NDX2+NXA)*(NX+NU),NXA,NU,NP,NDX,NOD ) &
				adjoint_sweep.init( backward,"acado_backward",NX*(2+NX+NU),NXA,NU,NP,NDX,NOD ) );
	}
	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::getCode(	ExportStatementBlock& code )
{
	int mode;
	get( IMPLICIT_INTEGRATOR_MODE, mode );

	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );
	int liftMode = 1;
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) liftMode = 4;

	if ( (ExportSensitivityType)sensGen != FORWARD && (ExportSensitivityType)sensGen != INEXACT ) ACADOERROR( RET_INVALID_OPTION );
	if( (ImplicitIntegratorMode)mode != LIFTED ) ACADOERROR( RET_INVALID_OPTION );
//	if( liftMode != 1 && liftMode != 4 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
//	if( (ExportSensitivityType)sensGen == INEXACT && liftMode != 4 ) ACADOERROR( RET_INVALID_OPTION );

	if( CONTINUOUS_OUTPUT || NX1 > 0 || NX3 > 0 || !equidistantControlGrid() ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	uint grad = 0;
	int gradientUp;
	get( LIFTED_GRADIENT_UPDATE, gradientUp );
	bool gradientUpdate = (bool) gradientUp;
	if( gradientUpdate ) grad = NX;

	if( gradientUpdate && (ExportSensitivityType)sensGen != INEXACT ) ACADOERROR( RET_INVALID_OPTION );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP ) {
		ExportVariable max = getAuxVariable();
		max.setName( "auxVar" );
		max.setDataStruct( ACADO_LOCAL );
		if( NX2 > 0 || NXA > 0 ) {
			rhs.setGlobalExportVariable( max );
			diffs_rhs.setGlobalExportVariable( max );
			forward_sweep.setGlobalExportVariable( max );
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
			if( (ExportSensitivityType)sensGen == INEXACT ) s << ", " << rk_seed.getFullName();
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
			code.addFunction( forward_sweep );
			code.addStatement( "\n\n" );
			code.addFunction( adjoint_sweep );
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
	uint run5, run6;
	std::string tempString;
	
	initializeDDMatrix();
	initializeCoefficients();

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	DMatrix tmp = AA;
	ExportVariable Ah( "Ah_mat", tmp*=h, STATIC_CONST_REAL );
	code.addDeclaration( Ah );
	code.addLinebreak( 2 );
	// TODO: Ask Milan why this does NOT work properly !!
	Ah = ExportVariable( "Ah_mat", numStages, numStages, STATIC_CONST_REAL, ACADO_LOCAL );

	DVector BB( bb );
	ExportVariable Bh( "Bh_mat", DMatrix( BB*=h ) );

	DVector CC( cc );
	ExportVariable C;
	if( timeDependant ) {
		C = ExportVariable( "C_mat", DMatrix( CC*=(1.0/grid.getNumIntervals()) ), STATIC_CONST_REAL );
		code.addDeclaration( C );
		code.addLinebreak( 2 );
		C = ExportVariable( "C_mat", 1, numStages, STATIC_CONST_REAL, ACADO_LOCAL );
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
	ExportIndex k_index("k_index");
	ExportIndex shooting_index("shoot_index");
	ExportVariable tmp_meas("tmp_meas", 1, outputGrids.size(), INT, ACADO_LOCAL);

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !equidistantControlGrid() ) {
		ExportVariable numStepsV( "numSteps", numSteps, STATIC_CONST_INT );
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
	integrate.addIndex( tmp_index3 );
	integrate.addIndex( shooting_index );
	integrate.addIndex( k_index );
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
	integrate << shooting_index.getFullName() << " = " << rk_index.getFullName() << ";\n";
	integrate.addStatement( rk_ttt == DMatrix(grid.getFirstTime()) );
	if( (inputDim-diffsDim) > NX+NXA ) {
		integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
		if( (ExportSensitivityType)sensGen == INEXACT ) {
			integrate.addStatement( rk_seed.getCols( NX+NXA+(NX+NDX2+NXA)*(NX+NU),(NX+NDX2+NXA)*(NX+NU)+inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
			if( gradientUpdate ) {
				integrate.addStatement( rk_seed2.getCols( 2*NX+NXA+(NX+NDX2+NXA)*(NX+NU),NX+(NX+NDX2+NXA)*(NX+NU)+inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
			}
		}
	}
	integrate.addLinebreak( );
	if( liftMode == 1 || liftMode == 4 ) {
		integrate.addStatement( rk_delta.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) - rk_Xprev.getRow(shooting_index) );
		integrate.addStatement( rk_Xprev.getRow(shooting_index) == rk_eta.getCols( 0,NX ) );

		integrate.addStatement( rk_delta.getCols( NX,NX+NU ) == rk_eta.getCols( NX+NXA+diffsDim,NX+NXA+diffsDim+NU ) - rk_Uprev.getRow(shooting_index) );
		integrate.addStatement( rk_Uprev.getRow(shooting_index) == rk_eta.getCols( NX+NXA+diffsDim,NX+NXA+diffsDim+NU ) );
	}
	integrate.addStatement( rk_xxx_lin == rk_eta.getCols(0,NX) );

	if( gradientUpdate ) {
		    DMatrix zeroL = zeros<double>(1,NX+NU);
		    integrate.addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) == zeroL );
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

	//	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
	// Set rk_diffsPrev:
	loop->addStatement( std::string("if( run > 0 ) {\n") );
//	if( NX1 > 0 ) {
//		ExportForLoop loopTemp1( i,0,NX1 );
//		loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,0,NX1 ) == rk_eta.getCols( i*NX+2*NX+NXA,i*NX+2*NX+NXA+NX1 ) );
//		if( NU > 0 ) loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,NX1,NX1+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+NX,i*NU+(NX+NXA)*(NX+1)+NX+NU ) );
//		loop->addStatement( loopTemp1 );
//	}
	if( NX2 > 0 ) {
		ExportForLoop loopTemp2( i,0,NX2 );
		loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,0,NX1+NX2 ) == rk_eta.getCols( i*NX+NX+grad+NXA+NX1*NX,i*NX+NX+grad+NXA+NX1*NX+NX1+NX2 ) );
		if( NU > 0 ) loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,NX1+NX2,NX1+NX2+NU ) == rk_eta.getCols( i*NU+grad+(NX+NXA)*(NX+1)+NX1*NU,i*NU+grad+(NX+NXA)*(NX+1)+NX1*NU+NU ) );
		loop->addStatement( loopTemp2 );
	}
//	if( NX3 > 0 ) {
//		ExportForLoop loopTemp3( i,0,NX3 );
//		loopTemp3.addStatement( rk_diffsPrev3.getSubMatrix( i,i+1,0,NX ) == rk_eta.getCols( i*NX+2*NX+NXA+(NX1+NX2)*NX,i*NX+2*NX+NXA+(NX1+NX2)*NX+NX ) );
//		if( NU > 0 ) loopTemp3.addStatement( rk_diffsPrev3.getSubMatrix( i,i+1,NX,NX+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+(NX1+NX2)*NU+NX,i*NU+(NX+NXA)*(NX+1)+(NX1+NX2)*NU+NU+NX ) );
//		loop->addStatement( loopTemp3 );
//	}
	loop->addStatement( std::string("}\nelse{\n") );
	DMatrix eyeM = eye<double>(NX);
	eyeM.appendCols(zeros<double>(NX,NU));
	loop->addStatement( rk_diffsPrev2 == eyeM );
	loop->addStatement( std::string("}\n") );
	//	}

	loop->addStatement( k_index == (shooting_index*grid.getNumIntervals()+run)*(NX+NXA) );

	// FIRST update using term from optimization variables:
	if( liftMode == 1 || (liftMode == 4 && (ExportSensitivityType)sensGen == INEXACT) ) {
		ExportForLoop loopTemp1( i,0,NX+NXA );
		loopTemp1.addStatement( j == k_index+i );
		loopTemp1.addStatement( tmp_index1 == j*(NX+NU) );
		ExportForLoop loopTemp2( run1,0,numStages );
		loopTemp2.addStatement( rk_kkk.getElement( j,run1 ) += rk_delta*rk_diffK.getSubMatrix( tmp_index1,tmp_index1+NX+NU,run1,run1+1 ) );
		loopTemp1.addStatement( loopTemp2 );
		loop->addStatement( loopTemp1 );
	}

	// Evaluate all stage values for reuse:
	evaluateAllStatesImplicitSystem( loop, k_index, Ah, C, run1, j, tmp_index1 );

	// SAVE rk_stageValues in the rk_xxx_traj variable:
	loop->addStatement( rk_xxx_traj.getCols(run*numStages*(NX2+NXA),(run+1)*numStages*(NX2+NXA)) == rk_stageValues );

	solveImplicitSystem( loop, i, run1, j, tmp_index1, k_index, Ah, C, determinant, true );

	// NEW: UPDATE RK_B WITH THE CONSTANT COMING FROM THE PREVIOUS INTEGRATION STEP
	if( liftMode == 1 ) { // EXACT LIFTING
		loop->addStatement( std::string("if( run > 0 ) {\n") );
		loop->addStatement( rk_Xhat_traj.getRows(run*NX,(run+1)*NX) == rk_Xhat_traj.getRows((run-1)*NX,run*NX) );
		for( run5 = 0; run5 < numStages; run5++ ) {
			loop->addStatement( rk_Xhat_traj.getRows(run*NX,(run+1)*NX) += rk_Khat_traj.getSubMatrix( run-1,run,run5*(NX2+NXA),run5*(NX2+NXA)+NX2 ).getTranspose()*Bh.getElement(run5,0) );
		}
		ExportForLoop deltaLoop( i,0,numStages );
		ExportForLoop deltaLoop2( j,0,NX );
		if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			deltaLoop2.addStatement( tmp_index1 == i*(NX2+NXA)+j );
			deltaLoop2.addStatement( rk_b.getElement(tmp_index1,0) -= rk_diffsTemp2.getSubMatrix(i,i+1,j*NVARS2,j*NVARS2+NX)*rk_Xhat_traj.getRows(run*NX,(run+1)*NX) );
		}
		else {
			deltaLoop2.addStatement( tmp_index1 == i*NX2+j );
			deltaLoop2.addStatement( rk_b.getElement(tmp_index1,0) -= rk_diffsTemp2.getSubMatrix(i,i+1,j*NVARS2,j*NVARS2+NX)*rk_Xhat_traj.getRows(run*NX,(run+1)*NX) );
		}
		deltaLoop.addStatement( deltaLoop2 );
		loop->addStatement( deltaLoop );
		loop->addStatement( std::string("}\nelse{\n") );
		loop->addStatement( rk_Xhat_traj.getRows(0,NX) == zeros<double>(NX,1) );
		loop->addStatement( std::string("}\n") );
	}


	// Evaluate sensitivities:
	if( (ExportSensitivityType)sensGen != INEXACT ) {
		evaluateRhsSensitivities( loop, run1, i, j, tmp_index1, tmp_index2 );
		if( liftMode == 1 ) {
			allSensitivitiesImplicitSystem( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Bh, false );
		}
		else {
			allSensitivitiesImplicitSystem( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, ExportIndex(0), Bh, false );
		}
	}
	else {
		evaluateRhsInexactSensitivities( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Ah );

		// GRADIENT UPDATE INIS SCHEME:
		if( gradientUpdate ) {
			loop->addStatement( tmp_index1 == shooting_index*grid.getNumIntervals()+run );
			loop->addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) += rk_adj_traj.getRow(tmp_index1)*rk_b.getCols(1,1+NX+NU) );
		}

		allSensitivitiesImplicitSystem( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Bh, true );
	}
	if( liftMode == 1 ) { // EXACT LIFTING
		// !!! update rk_xxx_lin (YOU NEED TO UPDATE THE LINEARIZATION POINT BEFORE YOU UPDATE RK_KKK): !!!
		// (YOU SHOULD DO THIS AFTER YOU COMPUTE THE NEXT LINEARIZATION POINT IF YOU WANT TO BE FULLY EQUIVALENT WITH DIRECT COLLOCATION, see SYMMETRIC AND FOB VERSIONS):
		for( run5 = 0; run5 < NX; run5++ ) {
			loop->addStatement( rk_xxx_lin.getCol( run5 ) += rk_kkk.getRow( k_index+run5 )*Bh );
		}
	}

	ExportForLoop loopTemp( j,0,numStages );
	for( run5 = 0; run5 < NX2; run5++ ) {
		if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			loopTemp.addStatement( rk_kkk.getElement( k_index+NX1+run5,j ) += rk_b.getElement( j*(NX2+NXA)+run5,0 ) );		// differential states
		}
		else {
			loopTemp.addStatement( rk_kkk.getElement( k_index+NX1+run5,j ) += rk_b.getElement( j*NX2+run5,0 ) );			// differential states
		}
	}
	for( run5 = 0; run5 < NXA; run5++ ) {
		if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			loopTemp.addStatement( rk_kkk.getElement( k_index+NX+run5,j ) += rk_b.getElement( j*(NX2+NXA)+NX2+run5,0 ) );		// algebraic states
		}
		else {
			loopTemp.addStatement( rk_kkk.getElement( k_index+NX+run5,j ) += rk_b.getElement( numStages*NX2+j*NXA+run5,0 ) );	// algebraic states
		}
	}
	loop->addStatement( loopTemp );
	if( liftMode == 1 ) {	// EXACT LIFTING
		if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			loop->addStatement( rk_Khat_traj.getRow(run) == rk_b.getCol(0).getTranspose() );
		}
		else {
			for( run5 = 0; run5 < numStages; run5++ ) {
				loop->addStatement( rk_Khat_traj.getSubMatrix(run,run+1,run5*(NX2+NXA),run5*(NX2+NXA)+NX2) == rk_b.getSubMatrix(run5*NX2,run5*NX2+NX2,0,1).getTranspose() );
			}
			for( run5 = 0; run5 < numStages; run5++ ) {
				loop->addStatement( rk_Khat_traj.getSubMatrix(run,run+1,run5*(NX2+NXA)+NX2,(run5+1)*(NX2+NXA)) == rk_b.getSubMatrix(numStages*NX2+run5*NXA,numStages*NX2+run5*NXA+NXA,0,1).getTranspose() );
			}
		}
	}

	// update rk_eta:
	for( run5 = 0; run5 < NX; run5++ ) {
		loop->addStatement( rk_eta.getCol( run5 ) += rk_kkk.getRow( k_index+run5 )*Bh );
	}
	if( NXA > 0) {
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );
		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
			loop->addStatement( std::string("if( run == 0 ) {\n") );
		}
		for( run5 = 0; run5 < NXA; run5++ ) {
			loop->addStatement( rk_eta.getCol( NX+run5 ) == rk_kkk.getRow( k_index+NX+run5 )*tempCoefs );
		}
		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
			loop->addStatement( std::string("}\n") );
		}
	}
	if( liftMode != 1 ) { // INEXACT LIFTING
		// !!! update rk_xxx_lin (YOU NEED TO UPDATE THE LINEARIZATION POINT AFTER YOU UPDATE RK_KKK): !!!
		loop->addStatement( rk_xxx_lin == rk_eta.getCols(0,NX) );
	}


//	// Computation of the sensitivities using the CHAIN RULE:
//	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
//		loop->addStatement( std::string( "if( run == 0 ) {\n" ) );
//	}
//	// PART 1
//	updateInputSystem(loop, i, j, tmp_index2);

	// PART 2
	updateImplicitSystem(loop, i, j, tmp_index2);

//	// PART 3
//	updateOutputSystem(loop, i, j, tmp_index2);

//	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
//		loop->addStatement( std::string( "}\n" ) );
//		loop->addStatement( std::string( "else {\n" ) );
////		// PART 1
////		propagateInputSystem(loop, i, j, k, tmp_index2);
//
//		// PART 2
//		propagateImplicitSystem(loop, i, j, k, tmp_index2);
//
////		// PART 3
////		propagateOutputSystem(loop, i, j, k, tmp_index2);
//	}
//
//	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !equidistantControlGrid()) ) {
//		propagateOutputs( loop, run, run1, i, j, k, tmp_index1, tmp_index2, tmp_index3, tmp_index4, tmp_meas );
//	}

//	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
//		loop->addStatement( std::string( "}\n" ) );
//	}

//	loop->addStatement( std::string( reset_int.get(0,0) ) + " = 0;\n" );

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

    if( gradientUpdate ) {
    	// COMPUTE THE INEXACT ADJOINT BASED ON LAMBDA AND THE INEXACT SENSITIVITIES:
//    	DMatrix zeroL = zeros<double>(1,NX+NU);
//    	integrate.addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) == zeroL );
//    	for( run5 = 0; run5 < NX; run5 ++ ) {
//    		integrate.addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) += rk_eta.getCol(NX+run5)*rk_diffsNew2.getRow(run5) );
//    	}
//    	integrate.addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim-NU) -= rk_eta.getCols(NX,2*NX) );

    	// integrator BACKWARD loop:
    	integrate.addComment("------------ BACKWARD loop ------------:");
    	ExportForLoop tmpLoop2( run, grid.getNumIntervals()-1, -1, -1 );
    	ExportStatementBlock *loop2;
    	if( equidistantControlGrid() ) {
    		loop2 = &tmpLoop2;
    	}
    	else {
    		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
    	}

    	loop2->addStatement( k_index == run*(NX+NXA)*(NX+NU) );


    	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) { // INEXACT
    		// Compute \hat{lambda}:
    		// vec(j*NX+1:j*NX+NX) = -Bh_vec(j+1)*dir_tmp;
    		for( run5 = 0; run5 < numStages; run5++ ) {
    			DMatrix zeroV = zeros<double>(1,NX);
    			loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) == zeroV );
    			loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) -= Bh.getRow(run5)*rk_eta.getCols(NX,2*NX) );
    		}
    		loop2->addStatement( tmp_index1 == shooting_index*grid.getNumIntervals()+run );
    		for( run5 = 0; run5 < numStages; run5++ ) {
    			loop2->addStatement( rk_seed2.getCols(0,NX) == rk_xxx_traj.getCols((run*numStages+run5)*(NX2+NXA),(run*numStages+run5+1)*(NX2+NXA)) );
    			loop2->addStatement( rk_seed2.getCols(NX,2*NX) == rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*(NX+NXA),(run5+1)*(NX+NXA)) );
    			loop2->addFunctionCall( adjoint_sweep.getName(), rk_seed2, rk_adj_diffs_tmp );
    			for( run6 = 0; run6 < numStages; run6++ ) {
    				loop2->addStatement( rk_b_trans.getCols(run6*NX,(run6+1)*NX) -= Ah.getElement(run5,run6)*rk_adj_diffs_tmp.getCols(0,NX) );
    			}
    			loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) += rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*(NX+NXA),(run5+1)*(NX+NXA)) ); // BECAUSE EXPLICIT ODE
    		}
    		loop2->addFunctionCall( solver->getNameSolveTransposeReuseFunction(),rk_A.getAddress(0,0),rk_b_trans.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
    		loop2->addStatement( rk_adj_traj.getRow(tmp_index1) += rk_b_trans );
    		//		loop2->addStatement( rk_b_trans == rk_adj_traj.getRow(tmp_index1) );
    	}
    	else return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

    	for( run5 = 0; run5 < numStages; run5++ ) {
    		loop2->addStatement( rk_seed2.getCols(0,NX) == rk_xxx_traj.getCols((run*numStages+run5)*(NX2+NXA),(run*numStages+run5+1)*(NX2+NXA)) );
    		loop2->addStatement( rk_seed2.getCols(NX,2*NX) == rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*NX,(run5+1)*NX) );
    		loop2->addFunctionCall( adjoint_sweep.getName(), rk_seed2, rk_adj_diffs_tmp.getAddress(0,0) );
    		loop2->addStatement( rk_eta.getCols(NX,2*NX) += rk_adj_diffs_tmp.getCols(0,NX) );
//    		loop2->addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) -= rk_adj_diffs_tmp.getCols(0,NX+NU) );
    	}

    	loop2->addStatement( rk_ttt -= DMatrix(1.0/grid.getNumIntervals()) );
    	// end of the backward integrator loop.
    	if( !equidistantControlGrid() ) {
    		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
    	}
    	else {
    		integrate.addStatement( *loop2 );
    	}
    }

    // PART 1
    if( NX1 > 0 ) {
    	DMatrix zeroR = zeros<double>(1, NX2+NX3);
    	ExportForLoop loop1( i,0,NX1 );
    	loop1.addStatement( rk_eta.getCols( i*NX+NX+grad+NXA+NX1,i*NX+NX+grad+NXA+NX ) == zeroR );
    	integrate.addStatement( loop1 );
    }
    // PART 2
    DMatrix zeroR = zeros<double>(1, NX3);
    if( NX2 > 0 && NX3 > 0 ) {
    	ExportForLoop loop2( i,NX1,NX1+NX2 );
    	loop2.addStatement( rk_eta.getCols( i*NX+NX+grad+NXA+NX1+NX2,i*NX+NX+grad+NXA+NX ) == zeroR );
    	integrate.addStatement( loop2 );
    }
    if( NXA > 0 && NX3 > 0 ) {
    	ExportForLoop loop3( i,NX,NX+NXA );
    	loop3.addStatement( rk_eta.getCols( i*NX+NX+grad+NXA+NX1+NX2,i*NX+NX+grad+NXA+NX ) == zeroR );
    	integrate.addStatement( loop3 );
    }

//    integrate.addStatement( std::string( "if( " ) + determinant.getFullName() + " < 1e-12 ) {\n" );
//    integrate.addStatement( error_code == 2 );
//    integrate.addStatement( std::string( "} else if( " ) + determinant.getFullName() + " < 1e-6 ) {\n" );
//    integrate.addStatement( error_code == 1 );
//    integrate.addStatement( std::string( "} else {\n" ) );
    integrate.addStatement( error_code == 0 );
//    integrate.addStatement( std::string( "}\n" ) );

	code.addFunction( integrate );
    code.addLinebreak( 2 );

    return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::solveImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index, const ExportIndex& k_index, const ExportVariable& Ah, const ExportVariable& C, const ExportVariable& det, bool DERIVATIVES )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen);
	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );

	int liftMode = 1;
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) liftMode = 4;

	if( NX2 > 0 || NXA > 0 ) {

		// Perform iteration by system solve:
		if( liftMode == 4 ) {
			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
				block->addStatement( "if( run == 0 ) {\n" );
			}

			// evaluate rk_J (only explicit ODE for now!)
			if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
				evaluateStatesImplicitSystem( block, k_index, Ah, C, ExportIndex(0), index3, tmp_index );
				block->addFunctionCall( getNameDiffsRHS(), rk_xxx, rk_diffsTemp2 );

				DMatrix zeroZ = zeros<double>(1,NXA);
				ExportForLoop loop0( index2,0,NX+NXA );
				loop0.addStatement( rk_A.getSubMatrix( index2,index2+1,0,NX ) == rk_diffsTemp2.getSubMatrix( index2,index2+1,NX1,NX1+NX2 ) );
				loop0.addStatement( rk_A.getSubMatrix( index2,index2+1,NX,NX+NXA ) == zeroZ );
				block->addStatement( loop0 );

				if( NDX2 > 0 ) {
					ExportForLoop loop01( index2,0,NX+NXA );
					loop01.addStatement( rk_I.getSubMatrix( index2,index2+1,0,NX ) == rk_diffsTemp2.getSubMatrix( index2,index2+1,NX+NXA+NU,NVARS2 ) );
					loop01.addStatement( rk_I.getSubMatrix( index2,index2+1,NX,NX+NXA ) == rk_diffsTemp2.getSubMatrix( index2,index2+1,NX,NX+NXA ) );
					block->addStatement( loop01 );
				}

				ExportForLoop loop01( index2,0,numStages );
				evaluateInexactMatrix( &loop01, index2, index3, tmp_index, k_index, rk_A, Ah, C, true, DERIVATIVES );
				block->addStatement( loop01 );

				if( NDX2 > 0 ) {
					block->addStatement( det.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_I.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
				}
				else {
					block->addStatement( det.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
				}
			}
			else {
				ExportForLoop loop01( index2,0,numStages );
				evaluateMatrix( &loop01, index2, index3, tmp_index, k_index, rk_A, Ah, C, true, false );
				block->addStatement( loop01 );

				block->addStatement( det.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
			}

			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
				block->addStatement( "}\n else {\n" );
				ExportForLoop loop02( index2,0,numStages );
				evaluateStatesImplicitSystem( &loop02, k_index, Ah, C, index2, index3, tmp_index );
				evaluateRhsImplicitSystem( &loop02, k_index, index2 );
				block->addStatement( loop02 );
				block->addStatement( "}\n" );
			}
		}
		else { // EXACT LIFTING
			ExportForLoop loop1( index2,0,numStages );
			evaluateMatrix( &loop1, index2, index3, tmp_index, k_index, rk_A, Ah, C, true, DERIVATIVES );
//			if( liftMode == 1 ) {  // Right-hand side term from update optimization variables:
//				for( uint i = 0; i < NX2+NXA; i++ ) {
//					loop1.addStatement( rk_b.getRow( index2*(NX2+NXA)+i ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,i*(NVARS2)+NX1,i*(NVARS2)+NX1+NX2 )*(rk_delta.getCols( NX1,NX1+NX2 ).getTranspose()) );
//					loop1.addStatement( rk_b.getRow( index2*(NX2+NXA)+i ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,i*(NVARS2)+NX1+NX2+NXA,i*(NVARS2)+NX1+NX2+NXA+NU )*(rk_delta.getCols( NX,NX+NU ).getTranspose()) );
//				}
//			}
			block->addStatement( loop1 );
			block->addStatement( det.getFullName() + " = " + ExportStatement::fcnPrefix + "_" + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
		}

//		// IF DEBUG MODE:
//		int debugMode;
//		get( INTEGRATOR_DEBUG_MODE, debugMode );
//		if ( (bool)debugMode == true ) {
//			block->addStatement( debug_mat == rk_A );
//		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::evaluateInexactMatrix( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index, const ExportIndex& k_index, const ExportVariable& _rk_A, const ExportVariable& Ah, const ExportVariable& C, bool evaluateB, bool DERIVATIVES )
{
	uint i;
	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );

	if( (LinearAlgebraSolver) linSolver != SIMPLIFIED_IRK_NEWTON && (LinearAlgebraSolver) linSolver != SINGLE_IRK_NEWTON ) {
		ExportForLoop loop2( index2,0,NX2+NXA );
		loop2.addStatement( tmp_index == index1*(NX2+NXA)+index2 );
		for( i = 0; i < numStages; i++ ) { // differential states
			if( NDX2 == 0 ) {
				loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) == Ah.getElement( index1,i )*rk_diffsTemp2.getSubMatrix( index2,index2+1,NX1,NX1+NX2 ) );
				loop2.addStatement( std::string( "if( " ) + toString(i) + " == " + index1.getName() + " ) " );
				loop2.addStatement( _rk_A.getElement( tmp_index,index2+i*NX2 ) -= 1 );
			}
			else {
				loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) == Ah.getElement( index1,i )*rk_diffsTemp2.getSubMatrix( index2,index2+1,NX1,NX1+NX2 ) );
				loop2.addStatement( std::string( "if( " ) + toString(i) + " == " + index1.getName() + " ) {\n" );
				loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,i*NX2,i*NX2+NX2 ) += rk_diffsTemp2.getSubMatrix( index2,index2+1,NVARS2-NX2,NVARS2 ) );
				loop2.addStatement( std::string( "}\n" ) );
			}
		}
		if( NXA > 0 ) {
			DMatrix zeroM = zeros<double>( 1,NXA );
			for( i = 0; i < numStages; i++ ) { // algebraic states
				loop2.addStatement( std::string( "if( " ) + toString(i) + " == " + index1.getName() + " ) {\n" );
				loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,numStages*NX2+i*NXA,numStages*NX2+i*NXA+NXA ) == rk_diffsTemp2.getSubMatrix( index2,index2+1,NX1+NX2,NX1+NX2+NXA ) );
				loop2.addStatement( std::string( "}\n else {\n" ) );
				loop2.addStatement( _rk_A.getSubMatrix( tmp_index,tmp_index+1,numStages*NX2+i*NXA,numStages*NX2+i*NXA+NXA ) == zeroM );
				loop2.addStatement( std::string( "}\n" ) );
			}
		}
		block->addStatement( loop2 );
	}
	if( evaluateB ) {
		evaluateStatesImplicitSystem( block, k_index, Ah, C, index1, index2, tmp_index );
		evaluateRhsImplicitSystem( block, k_index, index1 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::propagateOutputs(	ExportStatementBlock* block, const ExportIndex& index, const ExportIndex& index0, const ExportIndex& index1,
															const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
															const ExportIndex& tmp_index3, const ExportIndex& tmp_index4, const ExportVariable& tmp_meas )
{

	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue ForwardLiftedIRKExport::prepareInputSystem(	ExportStatementBlock& code )
{

	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue ForwardLiftedIRKExport::prepareOutputSystem(	ExportStatementBlock& code )
{

	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue ForwardLiftedIRKExport::sensitivitiesInputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportVariable& Bh, bool STATES )
{

	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue ForwardLiftedIRKExport::evaluateAllStatesImplicitSystem( ExportStatementBlock* block, const ExportIndex& k_index, const ExportVariable& Ah, const ExportVariable& C, const ExportIndex& stage, const ExportIndex& i, const ExportIndex& tmp_index )
{
	ExportForLoop loop0( stage,0,numStages );
	ExportForLoop loop1( i, 0, NX1+NX2 );
	loop1.addStatement( rk_stageValues.getCol( stage*(NX+NXA)+i ) == rk_xxx_lin.getCol( i ) );
	loop1.addStatement( tmp_index == k_index + i );
	for( uint j = 0; j < numStages; j++ ) {
		loop1.addStatement( rk_stageValues.getCol( stage*(NX+NXA)+i ) += Ah.getElement(stage,j)*rk_kkk.getElement( tmp_index,j ) );
	}
	loop0.addStatement( loop1 );

	ExportForLoop loop3( i, 0, NXA );
	loop3.addStatement( tmp_index == k_index + i + NX );
	loop3.addStatement( rk_stageValues.getCol( stage*(NX+NXA)+NX+i ) == rk_kkk.getElement( tmp_index,stage ) );
	loop0.addStatement( loop3 );
	block->addStatement( loop0 );

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::evaluateStatesImplicitSystem( ExportStatementBlock* block, const ExportIndex& k_index, const ExportVariable& Ah, const ExportVariable& C, const ExportIndex& stage, const ExportIndex& i, const ExportIndex& tmp_index )
{
	ExportForLoop loop( i, 0, NX+NXA );
	loop.addStatement( rk_xxx.getCol( i ) == rk_stageValues.getCol( stage*(NX+NXA)+i ) );
	block->addStatement( loop );

	ExportForLoop loop4( i, 0, NDX2 );
	loop4.addStatement( tmp_index == k_index + i );
	loop4.addStatement( rk_xxx.getCol( inputDim-diffsDim+i ) == rk_kkk.getElement( tmp_index,stage ) );
	block->addStatement( loop4 );

	if( C.getDim() > 0 ) {	// There is a time dependence, so it must be set
		block->addStatement( rk_xxx.getCol( inputDim-diffsDim+NDX ) == rk_ttt + C.getCol(stage) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::evaluateRhsImplicitSystem( ExportStatementBlock* block, const ExportIndex& k_index, const ExportIndex& stage )
{
	uint i;
	DMatrix zeroM = zeros<double>( NX2+NXA,1 );
	block->addFunctionCall( getNameRHS(), rk_xxx, rk_rhsTemp.getAddress(0,0) );
	// matrix rk_b:
	if( NDX2 == 0 ) {
		for( i = 0; i < NX2; i++ ) {
			block->addStatement( rk_b.getElement( stage*(NX2+NXA)+i,0 ) == rk_kkk.getElement( k_index+NX1+i,stage ) - rk_rhsTemp.getRow( i ) );
		}
	}
	else {
		for( i = 0; i < NX2; i++ ) {
			block->addStatement( rk_b.getElement( stage*(NX2+NXA)+i,0 ) == zeroM.getRow( i ) - rk_rhsTemp.getRow( i ) );
		}
	}
	for( i = 0; i < NXA; i++ ) {
		block->addStatement( rk_b.getElement( stage*(NX2+NXA)+NX2+i,0 ) == zeroM.getRow( i ) - rk_rhsTemp.getRow( NX2+i ) );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::evaluateRhsInexactSensitivities( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportIndex& tmp_index3, const ExportIndex& k_index, const ExportVariable& Ah )
{
	if( NX2 > 0 ) {
		uint j;

		ExportForLoop loop1( index2,0,numStages );
		loop1.addStatement( rk_seed.getCols(0,NX) == rk_stageValues.getCols(index2*(NX+NXA),index2*(NX+NXA)+NX) );
		loop1.addStatement( rk_seed.getCols(NX,NX+NX*(NX+NU)) == rk_diffsPrev2.makeRowVector() );

		ExportForLoop loop2( index3,0,NX2 );
		loop2.addStatement( tmp_index1 == k_index + index3 );
		ExportForLoop loop3( index1,0,NX2 );
		loop3.addStatement( tmp_index2 == tmp_index1*(NX+NU) + index1 );
		for( j = 0; j < numStages; j++ ) {
			loop3.addStatement( rk_seed.getCol( NX+index3*(NX+NU)+index1 ) += Ah.getElement(index2,j)*rk_diffK.getElement(tmp_index2,j) );
		}
		loop2.addStatement( loop3 );

		ExportForLoop loop4( index1,0,NU );
		loop4.addStatement( tmp_index2 == tmp_index1*(NX+NU) + NX + index1 );
		for( j = 0; j < numStages; j++ ) {
			loop4.addStatement( rk_seed.getCol( NX+index3*(NX+NU)+NX+index1 ) += Ah.getElement(index2,j)*rk_diffK.getElement(tmp_index2,j) );
		}
		loop2.addStatement( loop4 );
		loop1.addStatement( loop2 );

		if( NDX2 > 0 ) {
			ExportForLoop loop5( index3,0,NDX2 );
			loop5.addStatement( tmp_index1 == k_index + index3 );
			ExportForLoop loop51( index1,0,NX2 );
			loop51.addStatement( tmp_index2 == tmp_index1*(NX+NU) + index1 );
			loop51.addStatement( rk_seed.getCol( NX+(NX+index3)*(NX+NU)+index1 ) == rk_diffK.getElement(tmp_index2,index2) );
			loop5.addStatement( loop51 );

			ExportForLoop loop52( index1,0,NU );
			loop52.addStatement( tmp_index2 == tmp_index1*(NX+NU) + NX + index1 );
			loop52.addStatement( rk_seed.getCol( NX+(NX+index3)*(NX+NU)+NX+index1 ) == rk_diffK.getElement(tmp_index2,index2) );
			loop5.addStatement( loop52 );
			loop1.addStatement( loop5 );

			loop1.addStatement( rk_seed.getCols(NX+(NX+NDX2+NXA)*(NX+NU)+NXA+NU+NOD,rk_seed.getNumCols()) == rk_kkk.getSubMatrix( k_index,k_index+NX,index2,index2+1 ).getTranspose() );
		}

		if( NXA > 0 ) {
			ExportForLoop loop6( index3,0,NXA );
			loop6.addStatement( tmp_index1 == k_index + NX + index3 );
			ExportForLoop loop61( index1,0,NX2 );
			loop61.addStatement( tmp_index2 == tmp_index1*(NX+NU) + index1 );
			loop61.addStatement( rk_seed.getCol( NX+(NX+NDX2+index3)*(NX+NU)+index1 ) == rk_diffK.getElement(tmp_index2,index2) );
			loop6.addStatement( loop61 );

			ExportForLoop loop62( index1,0,NU );
			loop62.addStatement( tmp_index2 == tmp_index1*(NX+NU) + NX + index1 );
			loop62.addStatement( rk_seed.getCol( NX+(NX+NDX2+index3)*(NX+NU)+NX+index1 ) == rk_diffK.getElement(tmp_index2,index2) );
			loop6.addStatement( loop62 );
			loop1.addStatement( loop6 );

			loop1.addStatement( rk_seed.getCols(NX+(NX+NDX2+NXA)*(NX+NU),NX+(NX+NDX2+NXA)*(NX+NU)+NXA) == rk_stageValues.getCols(index2*(NX+NXA)+NX,index2*(NX+NXA)+NX+NXA) );
		}

		loop1.addFunctionCall( forward_sweep.getName(), rk_seed, rk_diffSweep );

		ExportForLoop loop02( index3,0,NX2+NXA );
		loop02.addStatement( tmp_index1 == k_index + index3 );
		loop02.addStatement( tmp_index3 == index2*(NX2+NXA)+index3 );
		ExportForLoop loop03( index1,0,NX2 );
		loop03.addStatement( tmp_index2 == tmp_index1*(NX+NU) + index1 );
		loop03.addStatement( rk_b.getElement( tmp_index3,1+index1 ) == 0.0 - rk_diffSweep.getElement( index3,index1 ) );
		if( NDX2 == 0 ) loop03.addStatement( rk_b.getElement( tmp_index3,1+index1 ) += rk_diffK.getElement(tmp_index2,index2) );
//		else return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
		loop02.addStatement( loop03 );

		ExportForLoop loop04( index1,0,NU );
		loop04.addStatement( tmp_index2 == tmp_index1*(NX+NU) + NX + index1 );
		loop04.addStatement( rk_b.getElement( tmp_index3,1+NX+index1 ) == 0.0 - rk_diffSweep.getElement( index3,NX+index1 ) );
		if( NDX2 == 0 ) loop04.addStatement( rk_b.getElement( tmp_index3,1+NX+index1 ) += rk_diffK.getElement(tmp_index2,index2) );
//		else return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
		loop02.addStatement( loop04 );
		loop1.addStatement( loop02 );
		block->addStatement( loop1 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::allSensitivitiesImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportIndex& tmp_index3, const ExportIndex& k_index, const ExportVariable& Bh, bool update )
{
	if( NX2 > 0 ) {
		int linSolver;
		get( LINEAR_ALGEBRA_SOLVER, linSolver );
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );  // We compute the algebraic variables at the beginning of the shooting interval !

		// call the linear solver:
		if( NDX2 > 0 ) {
			block->addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_I.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		}
		else {
			block->addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		}

		// update rk_diffK with the new sensitivities:
		ExportForLoop loop20( index2,0,numStages );
		ExportForLoop loop21( index3,0,NX2 );
		if( update ) {
			loop21.addStatement( tmp_index1 == (k_index + NX1 + index3)*(NX+NU) );
		}
		else {
			loop21.addStatement( tmp_index1 == (NX1 + index3)*(NX+NU) );
		}
		if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			loop21.addStatement( tmp_index3 == index2*(NX2+NXA)+index3 );
		}
		else {
			loop21.addStatement( tmp_index3 == index2*NX2+index3 );
		}
		ExportForLoop loop22( index1,0,NX2 );
		loop22.addStatement( tmp_index2 == tmp_index1+index1 );
		if( update ) {
			loop22.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+index1) );
		}
		else {
			loop22.addStatement( rk_diffK_local.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+index1) );
		}
		loop21.addStatement( loop22 );

		ExportForLoop loop23( index1,0,NU );
		loop23.addStatement( tmp_index2 == tmp_index1+NX+index1 );
		if( update ) {
			loop23.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+NX+index1) );
		}
		else {
			loop23.addStatement( rk_diffK_local.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+NX+index1) );
		}
		loop21.addStatement( loop23 );
		loop20.addStatement( loop21 );
		if( NXA > 0 ) {
			ExportForLoop loop24( index3,0,NXA );
			if( update ) {
				loop24.addStatement( tmp_index1 == (k_index + NX + index3)*(NX+NU) );
			}
			else {
				loop24.addStatement( tmp_index1 == (NX + index3)*(NX+NU) );
			}
			if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
				loop24.addStatement( tmp_index3 == index2*(NX2+NXA)+NX2+index3 );
			}
			else {
				loop24.addStatement( tmp_index3 == numStages*NX2+index2*NXA+index3 );
			}
			ExportForLoop loop25( index1,0,NX2 );
			loop25.addStatement( tmp_index2 == tmp_index1+index1 );
			if( update ) {
				loop25.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+index1) );
			}
			else {
				loop25.addStatement( rk_diffK_local.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+index1) );
			}
			loop24.addStatement( loop25 );

			ExportForLoop loop26( index1,0,NU );
			loop26.addStatement( tmp_index2 == tmp_index1+NX+index1 );
			if( update ) {
				loop26.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+NX+index1) );
			}
			else {
				loop26.addStatement( rk_diffK_local.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+NX+index1) );
			}
			loop24.addStatement( loop26 );
			loop20.addStatement( loop24 );
		}
		block->addStatement( loop20 );

		// update rk_diffK USING RK_DIFFK_LOCAL !! (PROPAGATION OF SENSITIVITIES)
		if( !update ) {
			ExportForLoop loop40( index2,0,numStages );
			ExportForLoop loop41( index3,0,NX2+NXA );
			loop41.addStatement( tmp_index1 == (k_index + NX1 + index3)*(NX+NU) );
			loop41.addStatement( tmp_index3 == (NX1 + index3)*(NX+NU) );
			ExportForLoop loop42( index1,0,NX );
			loop42.addStatement( tmp_index2 == tmp_index1+index1 );
			loop42.addStatement( rk_diffK.getElement(tmp_index2,index2) == 0.0 );
			for( uint loop_i = 0; loop_i < NX; loop_i++ ) {
				loop42.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_diffK_local.getElement(tmp_index3+loop_i,index2)*rk_diffsPrev2.getElement(loop_i,index1) );
			}
			loop41.addStatement( loop42 );
			ExportForLoop loop43( index1,0,NU );
			loop43.addStatement( tmp_index2 == tmp_index1+NX+index1 );
			loop43.addStatement( tmp_index3 == (NX1 + index3)*(NX+NU)+NX+index1 );
			loop43.addStatement( rk_diffK.getElement(tmp_index2,index2) == rk_diffK_local.getElement(tmp_index3,index2) );
			loop43.addStatement( tmp_index3 == (NX1 + index3)*(NX+NU) );
			for( uint loop_i = 0; loop_i < NX; loop_i++ ) {
				loop43.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_diffK_local.getElement(tmp_index3+loop_i,index2)*rk_diffsPrev2.getElement(loop_i,NX+index1) );
			}
			loop41.addStatement( loop43 );
			loop40.addStatement( loop41 );
			block->addStatement( loop40 );
		}

		// update rk_diffsNew with the new sensitivities:
		ExportForLoop loop3( index2,0,NX2 );
		loop3.addStatement( tmp_index1 == (k_index + NX1 + index2)*(NX+NU) );
		ExportForLoop loop31( index1,0,NX2+NU );
		loop31.addStatement( tmp_index2 == tmp_index1+index1 );
		loop31.addStatement( rk_diffsNew2.getElement( index2,index1 ) == rk_diffsPrev2.getElement( index2,index1 ) );
		loop31.addStatement( rk_diffsNew2.getElement( index2,index1 ) += rk_diffK.getRow( tmp_index2 )*Bh );
		loop3.addStatement( loop31 );
		block->addStatement( loop3 );

		if( NXA > 0 ) {
			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
				block->addStatement( std::string("if( run == 0 ) {\n") );
			}
			ExportForLoop loop5( index2,0,NXA );
			loop5.addStatement( tmp_index1 == (k_index + NX + index2)*(NX+NU) );
			ExportForLoop loop51( index1,0,NX2+NU );
			loop51.addStatement( tmp_index2 == tmp_index1+index1 );
			loop51.addStatement( rk_diffsNew2.getElement( index2+NX2,index1 ) == rk_diffK.getRow( tmp_index2 )*tempCoefs );
			loop5.addStatement( loop51 );
			block->addStatement( loop5 );
			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
				block->addStatement( std::string("}\n") );
			}
		}
		// TODO: The computation of these derivatives for the algebraic variables should be based on the diffK_local variable !!!
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::evaluateRhsSensitivities( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2 )
{
	if( NX2 > 0 ) {
		ExportForLoop loop1( index2,0,numStages );
		ExportForLoop loop2( index3,0,NX2+NXA );
		loop2.addStatement( tmp_index1 == index2*(NX2+NXA)+index3 );
		ExportForLoop loop3( index1,0,NX2 );
		loop3.addStatement( tmp_index2 == index1+index3*(NVARS2) );
		loop3.addStatement( rk_b.getElement( tmp_index1,1+index1 ) == 0.0 - rk_diffsTemp2.getElement( index2,tmp_index2 ) );
		loop2.addStatement( loop3 );

		ExportForLoop loop4( index1,0,NU );
		loop4.addStatement( tmp_index2 == index1+index3*(NVARS2)+NX1+NX2+NXA );
		loop4.addStatement( rk_b.getElement( tmp_index1,1+NX+index1 ) == 0.0 - rk_diffsTemp2.getElement( index2,tmp_index2 ) );
		loop2.addStatement( loop4 );
		loop1.addStatement( loop2 );
		block->addStatement( loop1 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::updateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
{
	uint grad = 0;
	int gradientUp;
	get( LIFTED_GRADIENT_UPDATE, gradientUp );
	bool gradientUpdate = (bool) gradientUp;
	if( gradientUpdate ) grad = NX;

	if( NX2 > 0 ) {
		ExportForLoop loop01( index1,NX1,NX1+NX2 );
		ExportForLoop loop02( index2,0,NX1+NX2 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA+grad ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX)+grad ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}
	// ALGEBRAIC STATES
	if( NXA > 0 ) {
		ExportForLoop loop01( index1,NX,NX+NXA );
		ExportForLoop loop02( index2,0,NX1+NX2 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+NX+NXA+grad ) == rk_diffsNew2.getSubMatrix( index1-NX1-NX3,index1-NX1-NX3+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX)+grad ) == rk_diffsNew2.getSubMatrix( index1-NX1-NX3,index1-NX1-NX3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardLiftedIRKExport::sensitivitiesOutputSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& index4, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportVariable& Ah, const ExportVariable& Bh, bool STATES, uint number )
{

	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue ForwardLiftedIRKExport::sensitivitiesOutputs( ExportStatementBlock* block, const ExportIndex& index0,
		const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2,
		const ExportIndex& tmp_index3, const ExportVariable& tmp_meas, const ExportVariable& time_tmp, bool STATES, uint base )
{

	return ACADOERROR( RET_NOT_YET_IMPLEMENTED );
}


returnValue ForwardLiftedIRKExport::setup( )
{
	ForwardIRKExport::setup();


	integrate = ExportFunction( "integrate", rk_eta );
	uint i;
	for( i = 0; i < rk_outputs.size(); i++ ) {
		integrate.addArgument( rk_outputs[i] );
	}
	integrate.addArgument( rk_index );
	integrate.setReturnValue( error_code );
	integrate.doc( "Performs the integration and sensitivity propagation for one shooting interval." );
	integrate.addLinebreak( );	// TO MAKE SURE IT GETS EXPORTED

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;

	rk_diffsPrev2 = ExportVariable( "rk_diffsPrev2", NX, NX+NU, REAL, structWspace );

	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen);
	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );
	int liftMode = 1;
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) liftMode = 4;

	if( (ExportSensitivityType)sensGen == INEXACT || liftMode == 4 ) {
		rk_seed = ExportVariable( "rk_seed", 1, NX+(NX+NDX2+NXA)*(NX+NU)+NXA+NU+NOD+NDX+timeDep, REAL, structWspace );
		rk_diffsTemp2 = ExportVariable( "rk_diffsTemp2", NX2+NXA, NVARS2, REAL, structWspace );
		rk_diffSweep = ExportVariable( "rk_diffSweep", NX2+NXA, NX+NU, REAL, structWspace );
	}

	rk_stageValues = ExportVariable( "rk_stageValues", 1, numStages*(NX+NXA), REAL, structWspace );
	rk_kkk = ExportVariable( "rk_Ktraj", N*grid.getNumIntervals()*(NX+NXA), numStages, REAL, ACADO_VARIABLES );
	if( liftMode == 1 || liftMode == 4 ) {
		rk_diffK = ExportVariable( "rk_diffKtraj", N*grid.getNumIntervals()*(NX+NXA)*(NX+NU), numStages, REAL, ACADO_VARIABLES );
	}
	else {
		rk_diffK = ExportVariable( "rk_diffK", (NX+NXA)*(NX+NU), numStages, REAL, structWspace );
	}

	if( (ExportSensitivityType)sensGen != BACKWARD ) {
		rk_Xprev = ExportVariable( "rk_Xprev", N, NX, REAL, ACADO_VARIABLES );
		rk_Uprev = ExportVariable( "rk_Uprev", N, NU, REAL, ACADO_VARIABLES );
		rk_delta = ExportVariable( "rk_delta", 1, NX+NU, REAL, ACADO_WORKSPACE );
	}

	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
		rk_A = ExportVariable( "rk_J", NX2+NXA, NX2+NXA, REAL, structWspace );
		if(NDX2 > 0) rk_I = ExportVariable( "rk_I", NX2+NXA, NX2+NXA, REAL, structWspace );
	}
	rk_b = ExportVariable( "rk_b", numStages*(NX+NXA), 1+NX+NU, REAL, structWspace );

	rk_xxx_lin = ExportVariable( "rk_xxx_lin", 1, NX, REAL, structWspace );

	int gradientUp;
	get( LIFTED_GRADIENT_UPDATE, gradientUp );
	bool gradientUpdate = (bool) gradientUp;

	if( gradientUpdate ) {
		diffsDim = NX + (NX+NXA)*(NX+NU) + NX+NU;
		inputDim = NX + diffsDim + NU + NOD;
		rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );

		rk_b_trans = ExportVariable( "rk_b_trans", 1, numStages*(NX+NXA), REAL, structWspace );
		rk_adj_traj = ExportVariable( "rk_adj_traj", N*grid.getNumIntervals(), numStages*(NX+NXA), REAL, ACADO_VARIABLES );

		rk_adj_diffs_tmp = ExportVariable( "rk_adjoint", 1, NX+NU, REAL, structWspace );

		rk_seed2 = ExportVariable( "rk_seed_b", 1, NX*(2+NX+NU)+NU+NOD+timeDep, REAL, structWspace );

		rk_xxx_traj = ExportVariable( "rk_stageV_traj", 1, grid.getNumIntervals()*numStages*(NX+NXA), REAL, structWspace );
	}

	if( liftMode == 1 ) {
		rk_Khat_traj = ExportVariable( "rk_Khat", grid.getNumIntervals(), numStages*(NX+NXA), REAL, structWspace );
		rk_Xhat_traj = ExportVariable( "rk_Xhat", grid.getNumIntervals()*NX, 1, REAL, structWspace );
	}
	if( liftMode == 1 ) rk_diffK_local = ExportVariable( "rk_diffKtraj_aux", (NX+NXA)*(NX+NU), numStages, REAL, structWspace );

    return SUCCESSFUL_RETURN;
}



// PROTECTED:


CLOSE_NAMESPACE_ACADO

// end of file.
