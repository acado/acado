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
 *    \file src/code_generation/integrators/irk_lifted_fob_export.cpp
 *    \author Rien Quirynen
 *    \date 2015
 */

#include <acado/code_generation/integrators/irk_export.hpp>
#include <acado/code_generation/integrators/irk_lifted_fob_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ForwardBackwardLiftedIRKExport::ForwardBackwardLiftedIRKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ForwardLiftedIRKExport( _userInteraction,_commonHeaderName )
{
}

ForwardBackwardLiftedIRKExport::ForwardBackwardLiftedIRKExport( const ForwardBackwardLiftedIRKExport& arg ) : ForwardLiftedIRKExport( arg )
{
}


ForwardBackwardLiftedIRKExport::~ForwardBackwardLiftedIRKExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}


ForwardBackwardLiftedIRKExport& ForwardBackwardLiftedIRKExport::operator=( const ForwardBackwardLiftedIRKExport& arg ){

    if( this != &arg ){

    	ForwardLiftedIRKExport::operator=( arg );
		copy( arg );
    }
    return *this;
}


ExportVariable ForwardBackwardLiftedIRKExport::getAuxVariable() const
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
		if( diffs_sweep.getGlobalExportVariable().getDim() >= max.getDim() ) {
			max = diffs_sweep.getGlobalExportVariable();
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


returnValue ForwardBackwardLiftedIRKExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	ForwardLiftedIRKExport::getDataDeclarations( declarations, dataStruct );

	declarations.addDeclaration( rk_A_traj,dataStruct );
	declarations.addDeclaration( rk_S_traj,dataStruct );
	declarations.addDeclaration( rk_xxx_traj,dataStruct );

	declarations.addDeclaration( rk_b_trans,dataStruct );

	declarations.addDeclaration( rk_adj_traj,dataStruct );

	declarations.addDeclaration( rk_adj_diffs_tmp,dataStruct );
	declarations.addDeclaration( rk_Khat_traj,dataStruct );
	declarations.addDeclaration( rk_Xhat_traj,dataStruct );

	declarations.addDeclaration( rk_hess_tmp1,dataStruct );
	declarations.addDeclaration( rk_hess_tmp2,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue ForwardBackwardLiftedIRKExport::getFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	ForwardLiftedIRKExport::getFunctionDeclarations( declarations );

    return SUCCESSFUL_RETURN;
}


returnValue ForwardBackwardLiftedIRKExport::setDifferentialEquation(	const Expression& rhs_ )
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

		// FORWARD SWEEP:
		DifferentialState sX("", NX,NX+NU);
		Expression Gx = sX.getCols(0,NX);
		Expression Gu = sX.getCols(NX,NX+NU);
		DifferentialEquation forward;
		for( uint i = 0; i < rhs_.getDim(); i++ ) {
			// NOT YET IMPLEMENTED FOR DAES OR IMPLICIT ODES
			if( NDX2 > 0 || NXA > 0 ) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);
			forward << multipleForwardDerivative( rhs_(i), x, Gx );
			forward << multipleForwardDerivative( rhs_(i), x, Gu ) + forwardDerivative( rhs_(i), u );
		}

		// FIRST ORDER ADJOINT SWEEP:
		DifferentialState lambda("", NX,1);
		DifferentialEquation backward, adj_update;
//		backward << backwardDerivative( rhs_, x, lambda );
		adj_update << backwardDerivative( rhs_, x, lambda );

		// SECOND ORDER ADJOINT SWEEP:
		Expression arg;
		arg << x;
		arg << u;
		Expression S_tmp = sX;
		S_tmp.appendRows(zeros<double>(NU,NX).appendCols(eye<double>(NU)));

		if( NDX2 > 0 || NXA > 0 ) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);
		Expression tmp = backwardDerivative( rhs_, arg, lambda );
		backward << tmp.getRows(0,NX);
		backward << multipleForwardDerivative( tmp, arg, S_tmp );

		// GRADIENT UPDATE:
	    int gradientUp;
	    get( LIFTED_GRADIENT_UPDATE, gradientUp );
	    bool gradientUpdate = (bool) gradientUp;

	    uint nHat = 0;
	    if( gradientUpdate ) {
	    	DifferentialState hat("",1,NX);
			Expression tmp2 = hat*tmp.getRows(0,NX);
			backward << multipleForwardDerivative( tmp2, arg, S_tmp );
			nHat = NX;
	    }

		if( f.getNT() > 0 ) timeDependant = true;

		return (rhs.init( f,"acado_rhs",NX,NXA,NU,NP,NDX,NOD ) &
				diffs_rhs.init( g,"acado_diffs",NX,NXA,NU,NP,NDX,NOD ) &
				forward_sweep.init( forward,"acado_forward",NX*(2+NX+NU)+nHat,NXA,NU,NP,NDX,NOD ) &
				adjoint_sweep.init( backward,"acado_backward",NX*(2+NX+NU)+nHat,NXA,NU,NP,NDX,NOD ) &
				diffs_sweep.init( adj_update,"acado_adjoint_update",NX*(2+NX+NU)+nHat,NXA,NU,NP,NDX,NOD ));
	}
	return SUCCESSFUL_RETURN;
}


Expression ForwardBackwardLiftedIRKExport::returnLowerTriangular( const Expression& expr ) {
//	std::cout << "returnLowerTriangular with " << expr.getNumRows() << " rows and " << expr.getNumCols() << " columns\n";
	ASSERT( expr.getNumRows() == expr.getNumCols() );

	Expression new_expr;
	for( uint i = 0; i < expr.getNumRows(); i++ ) {
		for( uint j = 0; j <= i; j++ ) {
			new_expr << expr(i,j);
		}
	}
	return new_expr;
}


returnValue ForwardBackwardLiftedIRKExport::getCode(	ExportStatementBlock& code )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	int mode;
	get( IMPLICIT_INTEGRATOR_MODE, mode );
//	get( LIFTED_INTEGRATOR_MODE, liftMode );
	if ( (ExportSensitivityType)sensGen != FORWARD_OVER_BACKWARD ) ACADOERROR( RET_INVALID_OPTION );
	if( (ImplicitIntegratorMode)mode != LIFTED ) ACADOERROR( RET_INVALID_OPTION );
//	if( liftMode != 1 && liftMode != 4 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if( NXA > 0) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

    int gradientUp;
    get( LIFTED_GRADIENT_UPDATE, gradientUp );
    bool gradientUpdate = (bool) gradientUp;

	// NOTE: liftMode == 4 --> inexact Newton based implementation

	if( CONTINUOUS_OUTPUT || NX1 > 0 || NX3 > 0 || !equidistantControlGrid() ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	ACADOWARNINGTEXT(RET_INVALID_OPTION, "The FORWARD_OVER_BACKWARD implementation is only for illustrational purposes, since the SYMMETRIC scheme will always outperform this.");

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	if( NX1 > 0 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );
	int liftMode = 1;
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) liftMode = 4;
	if( exportRhs ) {
		if( NX2 > 0 || NXA > 0 ) {
			code.addFunction( rhs );
			code.addStatement( "\n\n" );
			code.addFunction( diffs_rhs );
			code.addStatement( "\n\n" );
			if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
				code.addFunction( diffs_sweep );
				code.addStatement( "\n\n" );
			}
			if( liftMode == 4 ) { // ONLY for the inexact Newton based schemes
				code.addFunction( forward_sweep );
				code.addStatement( "\n\n" );
			}
			code.addFunction( adjoint_sweep );
			code.addStatement( "\n\n" );
		}

		if( NX3 > 0 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

		if( CONTINUOUS_OUTPUT ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
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
	integrate << shooting_index.getFullName() << " = " << rk_index.getFullName() << ";\n";
	integrate.addStatement( rk_ttt == DMatrix(grid.getFirstTime()) );
	if( (inputDim-diffsDim) > NX+NXA ) {
		integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
		if( !gradientUpdate ) integrate.addStatement( rk_seed.getCols( 2*NX+NX*(NX+NU),NX+NX*(NX+NU)+inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
		else integrate.addStatement( rk_seed.getCols( 3*NX+NX*(NX+NU),2*NX+NX*(NX+NU)+inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
	}
	integrate.addLinebreak( );
	if( liftMode == 1 || liftMode == 4 ) {
		integrate.addStatement( rk_delta.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) - rk_Xprev.getRow(shooting_index) );
		integrate.addStatement( rk_Xprev.getRow(shooting_index) == rk_eta.getCols( 0,NX ) );

		integrate.addStatement( rk_delta.getCols( NX,NX+NU ) == rk_eta.getCols( NX+NXA+diffsDim,NX+NXA+diffsDim+NU ) - rk_Uprev.getRow(shooting_index) );
		integrate.addStatement( rk_Uprev.getRow(shooting_index) == rk_eta.getCols( NX+NXA+diffsDim,NX+NXA+diffsDim+NU ) );
	}
	integrate.addStatement( rk_xxx_lin == rk_eta.getCols(0,NX) );

    // integrator FORWARD loop:
	integrate.addComment("------------ Forward loop ------------:");
	ExportForLoop tmpLoop( run, 0, grid.getNumIntervals() );
	ExportStatementBlock *loop;
	if( equidistantControlGrid() ) {
		loop = &tmpLoop;
	}
	else {
	    loop = &integrate;
		loop->addStatement( std::string("for(") + run.getName() + " = 0; " + run.getName() + " < " + numInt.getName() + "; " + run.getName() + "++ ) {\n" );
	}

//	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		// Set rk_diffsPrev:
		loop->addStatement( std::string("if( run > 0 ) {\n") );
		if( NX2 > 0 ) {
			ExportForLoop loopTemp2( i,0,NX2 );
			loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,0,NX1+NX2 ) == rk_eta.getCols( i*NX+2*NX+NXA+NX1*NX,i*NX+2*NX+NXA+NX1*NX+NX1+NX2 ) );
			if( NU > 0 ) loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,NX1+NX2,NX1+NX2+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+NX1*NU+NX,i*NU+(NX+NXA)*(NX+1)+NX1*NU+NU+NX ) );
			loop->addStatement( loopTemp2 );
		}
		loop->addStatement( std::string("}\nelse{\n") );
		DMatrix eyeM = eye<double>(NX);
		eyeM.appendCols(zeros<double>(NX,NU));
		loop->addStatement( rk_diffsPrev2 == eyeM );
		loop->addStatement( std::string("}\n") );
//	}

	// SAVE rk_diffsPrev2 in the rk_S_traj variable:
	loop->addStatement( rk_S_traj.getRows(run*NX,(run+1)*NX) == rk_diffsPrev2 );

	loop->addStatement( k_index == (shooting_index*grid.getNumIntervals()+run)*(NX+NXA) );

	// FIRST update using term from optimization variables:
	if( liftMode == 1 || liftMode == 4 ) {
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
	if( (LinearAlgebraSolver) linSolver != SIMPLIFIED_IRK_NEWTON && (LinearAlgebraSolver) linSolver != SINGLE_IRK_NEWTON && gradientUpdate) { // EXACT LIFTING with GRADIENT UPDATE
		loop->addStatement( std::string("if( run > 0 ) {\n") );
		loop->addStatement( rk_Xhat_traj.getRows(run*NX,(run+1)*NX) == rk_Xhat_traj.getRows((run-1)*NX,run*NX) );
		for( run5 = 0; run5 < numStages; run5++ ) {
			loop->addStatement( rk_Xhat_traj.getRows(run*NX,(run+1)*NX) += rk_Khat_traj.getSubMatrix( run-1,run,run5*(NX2+NXA),(run5+1)*(NX2+NXA) ).getTranspose()*Bh.getElement(run5,0) );
		}
		ExportForLoop deltaLoop( i,0,numStages );
		ExportForLoop deltaLoop2( j,0,NX );
		deltaLoop2.addStatement( tmp_index1 == i*(NX2+NXA)+j );
		deltaLoop2.addStatement( rk_b.getElement(tmp_index1,0) -= rk_diffsTemp2.getSubMatrix(i,i+1,j*NVARS2,j*NVARS2+NX)*rk_Xhat_traj.getRows(run*NX,(run+1)*NX) );
		deltaLoop.addStatement( deltaLoop2 );
		loop->addStatement( deltaLoop );
		loop->addStatement( std::string("}\nelse{\n") );
		loop->addStatement( rk_Xhat_traj.getRows(0,NX) == zeros<double>(NX,1) );
		loop->addStatement( std::string("}\n") );
	}

	// SAVE rk_A in the rk_A_traj variable:
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
		loop->addStatement( rk_A_traj.getRows(run*(NX2+NXA),(run+1)*(NX2+NXA)) == rk_A );
	}
	else {
		loop->addStatement( rk_A_traj.getRows(run*numStages*(NX2+NXA),(run+1)*numStages*(NX2+NXA)) == rk_A );
	}

	if( liftMode == 1 ) {
		// Evaluate sensitivities:
//		// !! NEW !! Let us propagate the forward sensitivities as in a VDE system
//		loop->addStatement( rk_seed.getCols(NX,NX+NX*(NX+NU)) == rk_diffsPrev2.makeRowVector() );
//		ExportForLoop loop_sens( i,0,numStages );
//		loop_sens.addStatement( rk_seed.getCols(0,NX) == rk_stageValues.getCols(i*(NX+NXA),i*(NX+NXA)+NX) );
//		loop_sens.addFunctionCall( forward_sweep.getName(), rk_seed, rk_diffsTemp2.getAddress(i,0) );
//		loop->addStatement( loop_sens );

		// In FOB, we reuse the derivatives computed to form the linear system (in rk_diffsTemp2) !!

		evaluateRhsSensitivities( loop, run1, i, j, tmp_index1, tmp_index2 );
		allSensitivitiesImplicitSystem( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Bh, false );
	}
	else if( liftMode == 4 ) {
		evaluateRhsInexactSensitivities( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Ah );
		allSensitivitiesImplicitSystem( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Bh, true );
	}
	else return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	// !!! update rk_xxx_lin (YOU NEED TO UPDATE THE LINEARIZATION POINT BEFORE YOU UPDATE RK_KKK): !!!
	for( run5 = 0; run5 < NX; run5++ ) {
		loop->addStatement( rk_xxx_lin.getCol( run5 ) += rk_kkk.getRow( k_index+run5 )*Bh );
	}

	// update rk_kkk:
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
	if( gradientUpdate ) {	// save the rk_b first column results:
		loop->addStatement( rk_Khat_traj.getRow(run) == rk_b.getCol(0).getTranspose() );
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

	// Computation of the sensitivities using the CHAIN RULE:
	updateImplicitSystem(loop, i, j, tmp_index2);

//	loop->addStatement( std::string( reset_int.get(0,0) ) + " = 0;\n" );

	loop->addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );

    // end of the forward integrator loop.
    if( !equidistantControlGrid() ) {
		loop->addStatement( "}\n" );
	}
    else {
    	integrate.addStatement( *loop );
    }

    // integrator BACKWARD loop:
	integrate.addComment("------------ BACKWARD loop ------------:");
    // set current Hessian to zero
	DMatrix zeroM;
	if( !gradientUpdate ) 	zeroM = zeros<double>(1,NX*(NX+NU)+NU*NU);
	else 					zeroM = zeros<double>(1,NX*(NX+NU)+NU*NU+NX+NU);
    integrate.addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) == zeroM );
	ExportForLoop tmpLoop2( run, grid.getNumIntervals()-1, -1, -1 );
	ExportStatementBlock *loop2;
	if( equidistantControlGrid() ) {
		loop2 = &tmpLoop2;
	}
	else {
	    return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	}

	loop2->addStatement( k_index == (shooting_index*grid.getNumIntervals()+run)*(NX+NXA)*(NX+NU) );

	// Compute \hat{lambda}:
	// vec(j*NX+1:j*NX+NX) = -Bh_vec(j+1)*dir_tmp;
	for( run5 = 0; run5 < numStages; run5++ ) {
		DMatrix zeroV = zeros<double>(1,NX);
		loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) == zeroV );
		loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) -= Bh.getRow(run5)*rk_eta.getCols(NX,2*NX) );
	}
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
		loop2->addStatement( tmp_index1 == shooting_index*grid.getNumIntervals()+run );
		for( run5 = 0; run5 < numStages; run5++ ) {
			loop2->addStatement( rk_seed.getCols(0,NX) == rk_xxx_traj.getCols((run*numStages+run5)*(NX2+NXA),(run*numStages+run5+1)*(NX2+NXA)) );
			loop2->addStatement( rk_seed.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) == rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*(NX+NXA),(run5+1)*(NX+NXA)) );
			loop2->addFunctionCall( diffs_sweep.getName(), rk_seed, rk_adj_diffs_tmp );
			for( run6 = 0; run6 < numStages; run6++ ) {
				loop2->addStatement( rk_b_trans.getCols(run6*NX,(run6+1)*NX) -= Ah.getElement(run5,run6)*rk_adj_diffs_tmp.getCols(0,NX) );
			}
			loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) += rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*(NX+NXA),(run5+1)*(NX+NXA)) ); // BECAUSE EXPLICIT ODE
		}
		loop2->addFunctionCall( solver->getNameSolveTransposeReuseFunction(),rk_A_traj.getAddress(run*(NX2+NXA),0),rk_b_trans.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		loop2->addStatement( rk_adj_traj.getRow(tmp_index1) += rk_b_trans );
		loop2->addStatement( rk_b_trans == rk_adj_traj.getRow(tmp_index1) );
	}
	else {
		loop2->addFunctionCall( solver->getNameSolveTransposeReuseFunction(),rk_A_traj.getAddress(run*numStages*(NX2+NXA),0),rk_b_trans.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
	}

	zeroM = zeros<double>(NX+NU,NX+NU);
	loop2->addStatement( rk_hess_tmp1 == zeroM );
	for( run5 = 0; run5 < numStages; run5++ ) {
		loop2->addStatement( rk_seed.getCols(0,NX) == rk_xxx_traj.getCols((run*numStages+run5)*(NX2+NXA),(run*numStages+run5+1)*(NX2+NXA)) );
		loop2->addStatement( rk_diffsPrev2 == rk_S_traj.getRows(run*NX,(run+1)*NX) );

		ExportForLoop diffLoop1( i, 0, NX );
		diffLoop1.addStatement( tmp_index1 == k_index+i*(NX+NU) );
		ExportForLoop diffLoop2( j, 0, NX+NU );
		diffLoop2.addStatement( tmp_index2 == tmp_index1+j );
		for( run6 = 0; run6 < numStages; run6++ ) {
			diffLoop2.addStatement( rk_diffsPrev2.getElement(i,j) += Ah.getElement(run5,run6)*rk_diffK.getElement( tmp_index2,run6 ) );
		}
		diffLoop1.addStatement( diffLoop2 );
		loop2->addStatement( diffLoop1 );

		loop2->addStatement( rk_seed.getCols(NX,NX*(1+NX+NU)) == rk_diffsPrev2.makeRowVector() );
		loop2->addStatement( rk_seed.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) == rk_b_trans.getCols(run5*NX,(run5+1)*NX) );
		if( gradientUpdate ) {
			loop2->addStatement( rk_seed.getCols(NX*(2+NX+NU),NX*(3+NX+NU)) == rk_Xhat_traj.getRows(run*NX,(run+1)*NX).getTranspose() );
			ExportForLoop gradLoop( i, 0, numStages );
			gradLoop.addStatement( rk_seed.getCols(NX*(2+NX+NU),NX*(3+NX+NU)) += Ah.getElement(run5,i)*rk_Khat_traj.getSubMatrix( run,run+1,i*(NX2+NXA),(i+1)*(NX2+NXA) ) );
			loop2->addStatement( gradLoop );
		}
		loop2->addFunctionCall( adjoint_sweep.getName(), rk_seed, rk_adj_diffs_tmp.getAddress(0,0) );
		loop2->addStatement( rk_eta.getCols(NX,2*NX) += rk_adj_diffs_tmp.getCols(0,NX) );
		for( run6 = 0; run6 < NX+NU; run6++ ) {
			loop2->addStatement( rk_hess_tmp2.getRow(run6) == rk_adj_diffs_tmp.getCols(NX+run6*(NX+NU),NX+(run6+1)*(NX+NU)) );
		}
		if( gradientUpdate ) {
			loop2->addStatement( rk_eta.getCols(NX+diffsDim-NX-NU,NX+diffsDim) += rk_adj_diffs_tmp.getCols(NX+(NX+NU)*(NX+NU),NX+(NX+NU)*(NX+NU)+NX+NU) );
		}

		// compute the local derivatives in rk_diffsPrev2
		loop2->addStatement( rk_diffsPrev2 == eyeM );
		ExportForLoop diffLoop3( i, 0, NX );
		diffLoop3.addStatement( tmp_index1 == k_index+i*(NX+NU) );
		ExportForLoop diffLoop4( j, 0, NX+NU );
		diffLoop4.addStatement( tmp_index2 == tmp_index1+j );
		for( run6 = 0; run6 < numStages; run6++ ) {
			diffLoop4.addStatement( rk_diffsPrev2.getElement(i,j) += Ah.getElement(run5,run6)*rk_diffK_local.getElement( tmp_index2,run6 ) );
		}
		diffLoop3.addStatement( diffLoop4 );
		loop2->addStatement( diffLoop3 );

		// update of rk_hess_tmp2 from the left and add it to rk_hess_tmp1
		updateHessianTerm( loop2, i, j );
	}


	for( run6 = 0; run6 < NX; run6++ ) {  // NX_NX
		loop2->addStatement( rk_hess_tmp2.getSubMatrix(run6,run6+1,0,NX) == rk_eta.getCols(NX*(2+NX+NU)+run6*NX,NX*(2+NX+NU)+(run6+1)*NX)  );
	}
	for( run6 = 0; run6 < NX; run6++ ) {  // NX_NU
		loop2->addStatement( rk_hess_tmp2.getSubMatrix(run6,run6+1,NX,NX+NU) == rk_eta.getCols(NX*(2+NX+NU)+NX*NX+run6*NU,NX*(2+NX+NU)+NX*NX+(run6+1)*NU) );
		loop2->addStatement( rk_hess_tmp2.getSubMatrix(NX,NX+NU,run6,run6+1) == rk_hess_tmp2.getSubMatrix(run6,run6+1,NX,NX+NU).getTranspose() );
	}
	for( run6 = 0; run6 < NU; run6++ ) {  // NU_NU
		loop2->addStatement( rk_hess_tmp2.getSubMatrix(NX+run6,NX+run6+1,NX,NX+NU) == rk_eta.getCols(NX*(2+NX+NU)+NX*(NX+NU)+run6*NU,NX*(2+NX+NU)+NX*(NX+NU)+(run6+1)*NU) );
	}

	// compute the local result derivatives in rk_diffsPrev2
	loop2->addStatement( rk_diffsPrev2 == eyeM );
	ExportForLoop diffLoop5( i, 0, NX );
	diffLoop5.addStatement( tmp_index1 == k_index+i*(NX+NU) );
	ExportForLoop diffLoop6( j, 0, NX+NU );
	diffLoop6.addStatement( tmp_index2 == tmp_index1+j );
	diffLoop6.addStatement( rk_diffsPrev2.getElement(i,j) += rk_diffK_local.getRow( tmp_index2 )*Bh );
	diffLoop5.addStatement( diffLoop6 );
	loop2->addStatement( diffLoop5 );
	updateHessianTerm( loop2, i, j );

	// UPDATE HESSIAN RESULT
	for( run6 = 0; run6 < NX; run6++ ) {  // NX_NX
		loop2->addStatement( rk_eta.getCols(NX*(2+NX+NU)+run6*NX,NX*(2+NX+NU)+(run6+1)*NX) == rk_hess_tmp1.getSubMatrix(run6,run6+1,0,NX) );
	}
	for( run6 = 0; run6 < NX; run6++ ) {  // NX_NU
		loop2->addStatement( rk_eta.getCols(NX*(2+NX+NU)+NX*NX+run6*NU,NX*(2+NX+NU)+NX*NX+(run6+1)*NU) == rk_hess_tmp1.getSubMatrix(run6,run6+1,NX,NX+NU) );
	}
	for( run6 = 0; run6 < NU; run6++ ) {  // NU_NU
		loop2->addStatement( rk_eta.getCols(NX*(2+NX+NU)+NX*(NX+NU)+run6*NU,NX*(2+NX+NU)+NX*(NX+NU)+(run6+1)*NU) == rk_hess_tmp1.getSubMatrix(NX+run6,NX+run6+1,NX,NX+NU) );
	}

	loop2->addStatement( rk_ttt -= DMatrix(1.0/grid.getNumIntervals()) );
    // end of the backward integrator loop.
    if( !equidistantControlGrid() ) {
    	return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	}
    else {
    	integrate.addStatement( *loop2 );
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


returnValue ForwardBackwardLiftedIRKExport::evaluateAllStatesImplicitSystem( ExportStatementBlock* block, const ExportIndex& k_index, const ExportVariable& Ah, const ExportVariable& C, const ExportIndex& stage, const ExportIndex& i, const ExportIndex& tmp_index )
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


returnValue ForwardBackwardLiftedIRKExport::updateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
{
	if( NX2 > 0 ) {
		ExportForLoop loop01( index1,NX1,NX1+NX2 );
		ExportForLoop loop02( index2,0,NX1+NX2 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+2*NX+NXA ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX)+NX ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}
	// ALGEBRAIC STATES
	if( NXA > 0 ) {
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardBackwardLiftedIRKExport::updateHessianTerm( ExportStatementBlock* block, const ExportIndex& i, const ExportIndex& j ) {
	ExportForLoop loop1( i, 0, NX );
	ExportForLoop loop2( j, 0, NX+NU );
	for( uint index = 0; index < NX; index++ ) {
		loop2.addStatement( rk_hess_tmp1.getElement(i,j) += rk_diffsPrev2.getElement(index,i)*rk_hess_tmp2.getElement(index,j) );
	}
	loop1.addStatement( loop2 );
	block->addStatement( loop1 );

	ExportForLoop loop3( i, 0, NU );
	ExportForLoop loop4( j, 0, NX+NU );
	loop4.addStatement( rk_hess_tmp1.getElement(NX+i,j) += rk_hess_tmp2.getElement(NX+i,j) );
	for( uint index = 0; index < NX; index++ ) {
		loop4.addStatement( rk_hess_tmp1.getElement(NX+i,j) += rk_diffsPrev2.getElement(index,NX+i)*rk_hess_tmp2.getElement(index,j) );
	}
	loop3.addStatement( loop4 );
	block->addStatement( loop3 );

	return SUCCESSFUL_RETURN;
}


returnValue ForwardBackwardLiftedIRKExport::allSensitivitiesImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportIndex& tmp_index3, const ExportIndex& k_index, const ExportVariable& Bh, bool update )
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

		// update rk_diffK_local with the new sensitivities:
		ExportForLoop loop20( index2,0,numStages );
		ExportForLoop loop21( index3,0,NX2 );
		loop21.addStatement( tmp_index1 == (k_index + NX1 + index3)*(NX+NU) );
		loop21.addStatement( tmp_index3 == index2*NX2+index3 );
		ExportForLoop loop22( index1,0,NX2 );
		loop22.addStatement( tmp_index2 == tmp_index1+index1 );
		if( update ) {
			loop22.addStatement( rk_diffK_local.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+index1) );
		}
		else {
			loop22.addStatement( rk_diffK_local.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+index1) );
		}
		loop21.addStatement( loop22 );

		ExportForLoop loop23( index1,0,NU );
		loop23.addStatement( tmp_index2 == tmp_index1+NX+index1 );
		if( update ) {
			loop23.addStatement( rk_diffK_local.getElement(tmp_index2,index2) += rk_b.getElement(tmp_index3,1+NX+index1) );
		}
		else {
			loop23.addStatement( rk_diffK_local.getElement(tmp_index2,index2) == rk_b.getElement(tmp_index3,1+NX+index1) );
		}
		loop21.addStatement( loop23 );
		loop20.addStatement( loop21 );
		block->addStatement( loop20 );

		// update rk_diffK USING RK_DIFFK_LOCAL !! (PROPAGATION OF SENSITIVITIES)
		ExportForLoop loop40( index2,0,numStages );
		ExportForLoop loop41( index3,0,NX2 );
		loop41.addStatement( tmp_index1 == (k_index + NX1 + index3)*(NX+NU) );
		ExportForLoop loop42( index1,0,NX );
		loop42.addStatement( tmp_index2 == tmp_index1+index1 );
		loop42.addStatement( rk_diffK.getElement(tmp_index2,index2) == 0.0 );
		for( uint loop_i = 0; loop_i < NX; loop_i++ ) {
			loop42.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_diffK_local.getElement(tmp_index1+loop_i,index2)*rk_diffsPrev2.getElement(loop_i,index1) );
		}
		loop41.addStatement( loop42 );
		ExportForLoop loop43( index1,0,NU );
		loop43.addStatement( tmp_index2 == tmp_index1+NX+index1 );
		loop43.addStatement( rk_diffK.getElement(tmp_index2,index2) == rk_diffK_local.getElement(tmp_index2,index2) );
		for( uint loop_i = 0; loop_i < NX; loop_i++ ) {
			loop43.addStatement( rk_diffK.getElement(tmp_index2,index2) += rk_diffK_local.getElement(tmp_index1+loop_i,index2)*rk_diffsPrev2.getElement(loop_i,NX+index1) );
		}
		loop41.addStatement( loop43 );
		loop40.addStatement( loop41 );
		block->addStatement( loop40 );

		// update rk_diffsNew with the new sensitivities:
		ExportForLoop loop3( index2,0,NX2 );
		loop3.addStatement( tmp_index1 == (k_index + NX1 + index2)*(NX+NU) );
		ExportForLoop loop31( index1,0,NX2 );
		loop31.addStatement( tmp_index2 == tmp_index1+index1 );
		loop31.addStatement( rk_diffsNew2.getElement( index2,index1 ) == rk_diffsPrev2.getElement( index2,index1 ) );
		loop31.addStatement( rk_diffsNew2.getElement( index2,index1 ) += rk_diffK.getRow( tmp_index2 )*Bh );
		loop3.addStatement( loop31 );

		ExportForLoop loop32( index1,0,NU );
		loop32.addStatement( tmp_index2 == tmp_index1+NX+index1 );
		loop32.addStatement( rk_diffsNew2.getElement( index2,NX+index1 ) == rk_diffsPrev2.getElement( index2,NX+index1 ) );
		loop32.addStatement( rk_diffsNew2.getElement( index2,NX+index1 ) += rk_diffK.getRow( tmp_index2 )*Bh );
		loop3.addStatement( loop32 );
		block->addStatement( loop3 );
		if( NXA > 0 ) {
			ACADOERROR( RET_NOT_IMPLEMENTED_YET );
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardBackwardLiftedIRKExport::evaluateRhsInexactSensitivities( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportIndex& tmp_index3, const ExportIndex& k_index, const ExportVariable& Ah )
{
	if( NX2 > 0 ) {
		uint j;

		ExportForLoop loop1( index2,0,numStages );
		loop1.addStatement( rk_seed.getCols(0,NX) == rk_stageValues.getCols(index2*(NX+NXA),index2*(NX+NXA)+NX) );
		DMatrix eyeM = eye<double>(NX);
		eyeM.appendCols(zeros<double>(NX,NU));
		loop1.addStatement( rk_seed.getCols(NX,NX+NX*(NX+NU)) == eyeM.makeVector().transpose() );

		ExportForLoop loop2( index3,0,NX2 );
		loop2.addStatement( tmp_index1 == k_index + index3 );
		ExportForLoop loop3( index1,0,NX2 );
		loop3.addStatement( tmp_index2 == tmp_index1*(NX+NU) + index1 );
		for( j = 0; j < numStages; j++ ) {
			loop3.addStatement( rk_seed.getCol( NX+index3*(NX+NU)+index1 ) += Ah.getElement(index2,j)*rk_diffK_local.getElement(tmp_index2,j) );
		}
		loop2.addStatement( loop3 );

		ExportForLoop loop4( index1,0,NU );
		loop4.addStatement( tmp_index2 == tmp_index1*(NX+NU) + NX + index1 );
		for( j = 0; j < numStages; j++ ) {
			loop4.addStatement( rk_seed.getCol( NX+index3*(NX+NU)+NX+index1 ) += Ah.getElement(index2,j)*rk_diffK_local.getElement(tmp_index2,j) );
		}
		loop2.addStatement( loop4 );
		loop1.addStatement( loop2 );

		if( NDX2 > 0 ) {
			return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

//			ExportForLoop loop5( index3,0,NDX2 );
//			loop5.addStatement( tmp_index1 == k_index + index3 );
//			ExportForLoop loop51( index1,0,NX2 );
//			loop51.addStatement( tmp_index2 == tmp_index1*(NX+NU) + index1 );
//			loop51.addStatement( rk_seed.getCol( NX+(NX+index3)*(NX+NU)+index1 ) == rk_diffK.getElement(tmp_index2,index2) );
//			loop5.addStatement( loop51 );
//
//			ExportForLoop loop52( index1,0,NU );
//			loop52.addStatement( tmp_index2 == tmp_index1*(NX+NU) + NX + index1 );
//			loop52.addStatement( rk_seed.getCol( NX+(NX+index3)*(NX+NU)+NX+index1 ) == rk_diffK.getElement(tmp_index2,index2) );
//			loop5.addStatement( loop52 );
//			loop1.addStatement( loop5 );
//
//			loop1.addStatement( rk_seed.getCols(NX+(NX+NDX2+NXA)*(NX+NU)+NXA+NU+NOD,rk_seed.getNumCols()) == rk_kkk.getSubMatrix( k_index,k_index+NX,index2,index2+1 ).getTranspose() );
		}

		if( NXA > 0 ) {
			return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

//			ExportForLoop loop6( index3,0,NXA );
//			loop6.addStatement( tmp_index1 == k_index + NX + index3 );
//			ExportForLoop loop61( index1,0,NX2 );
//			loop61.addStatement( tmp_index2 == tmp_index1*(NX+NU) + index1 );
//			loop61.addStatement( rk_seed.getCol( NX+(NX+NDX2+index3)*(NX+NU)+index1 ) == rk_diffK.getElement(tmp_index2,index2) );
//			loop6.addStatement( loop61 );
//
//			ExportForLoop loop62( index1,0,NU );
//			loop62.addStatement( tmp_index2 == tmp_index1*(NX+NU) + NX + index1 );
//			loop62.addStatement( rk_seed.getCol( NX+(NX+NDX2+index3)*(NX+NU)+NX+index1 ) == rk_diffK.getElement(tmp_index2,index2) );
//			loop6.addStatement( loop62 );
//			loop1.addStatement( loop6 );
//
//			loop1.addStatement( rk_seed.getCols(NX+(NX+NDX2+NXA)*(NX+NU),NX+(NX+NDX2+NXA)*(NX+NU)+NXA) == rk_stageValues.getCols(index2*(NX+NXA)+NX,index2*(NX+NXA)+NX+NXA) );
		}

		loop1.addFunctionCall( forward_sweep.getName(), rk_seed, rk_diffsTemp2 );

		ExportForLoop loop02( index3,0,NX2+NXA );
		loop02.addStatement( tmp_index1 == k_index + index3 );
		loop02.addStatement( tmp_index3 == index2*(NX2+NXA)+index3 );
		ExportForLoop loop03( index1,0,NX2 );
		loop03.addStatement( tmp_index2 == tmp_index1*(NX+NU) + index1 );
		loop03.addStatement( rk_b.getElement( tmp_index3,1+index1 ) == 0.0 - rk_diffsTemp2.getElement( index3,index1 ) );
		if( NDX2 == 0 ) loop03.addStatement( rk_b.getElement( tmp_index3,1+index1 ) += rk_diffK_local.getElement(tmp_index2,index2) );
		else return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
		loop02.addStatement( loop03 );

		ExportForLoop loop04( index1,0,NU );
		loop04.addStatement( tmp_index2 == tmp_index1*(NX+NU) + NX + index1 );
		loop04.addStatement( rk_b.getElement( tmp_index3,1+NX+index1 ) == 0.0 - rk_diffsTemp2.getElement( index3,NX+index1 ) );
		if( NDX2 == 0 ) loop04.addStatement( rk_b.getElement( tmp_index3,1+NX+index1 ) += rk_diffK_local.getElement(tmp_index2,index2) );
		else return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
		loop02.addStatement( loop04 );
		loop1.addStatement( loop02 );
		block->addStatement( loop1 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue ForwardBackwardLiftedIRKExport::setup( )
{
	ForwardLiftedIRKExport::setup();

	int gradientUp;
	get( LIFTED_GRADIENT_UPDATE, gradientUp );
	bool gradientUpdate = (bool) gradientUp;

	diffsDim   = NX + NX*(NX+NU) + NX*(NX+NU)+NU*NU;
	if( gradientUpdate ) diffsDim += NX+NU;
	inputDim = NX + diffsDim + NU + NOD;

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;

	rk_diffsPrev2 = ExportVariable( "rk_diffsPrev2", NX, NX+NU, REAL, structWspace );

	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
	rk_xxx_lin = ExportVariable( "rk_xxx_lin", 1, NX, REAL, structWspace );

	rk_b_trans = ExportVariable( "rk_b_trans", 1, numStages*(NX+NXA), REAL, structWspace );

	rk_adj_diffs_tmp = ExportVariable( "rk_adjoint", 1, NX + (NX+NU)*(NX+NU), REAL, structWspace );
	if( gradientUpdate ) {
		rk_adj_diffs_tmp = ExportVariable( "rk_adjoint", 1, NX + (NX+NU)*(NX+NU) + NX+NU, REAL, structWspace );
		rk_Khat_traj = ExportVariable( "rk_Khat", grid.getNumIntervals(), numStages*(NX+NXA), REAL, structWspace );

		rk_Xhat_traj = ExportVariable( "rk_Xhat", grid.getNumIntervals()*NX, 1, REAL, structWspace );
	}

	rk_seed = ExportVariable( "rk_seed", 1, NX+NX*(NX+NU)+NX+NU+NOD+timeDep, REAL, structWspace );
	if( gradientUpdate ) rk_seed = ExportVariable( "rk_seed", 1, NX+NX*(NX+NU)+2*NX+NU+NOD+timeDep, REAL, structWspace );
	rk_Xprev = ExportVariable( "rk_Xprev", N, NX, REAL, ACADO_VARIABLES );
	rk_A_traj = ExportVariable( "rk_A_traj", grid.getNumIntervals()*numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, structWspace );
	rk_xxx_traj = ExportVariable( "rk_stageV_traj", 1, grid.getNumIntervals()*numStages*(NX+NXA), REAL, structWspace );
	rk_S_traj = ExportVariable( "rk_S_traj", grid.getNumIntervals()*NX, NX+NU, REAL, structWspace );

	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
//		rk_A = ExportVariable( "rk_J", NX2+NXA, NX2+NXA, REAL, structWspace );
//		if(NDX2 > 0) rk_I = ExportVariable( "rk_I", NX2+NXA, NX2+NXA, REAL, structWspace );
		rk_A_traj = ExportVariable( "rk_J_traj", grid.getNumIntervals()*(NX2+NXA), NX2+NXA, REAL, structWspace );
		rk_diffsTemp2 = ExportVariable( "rk_diffsTemp2", NX2+NXA, NVARS2, REAL, structWspace );

		rk_adj_traj = ExportVariable( "rk_adj_traj", N*grid.getNumIntervals(), numStages*(NX+NXA), REAL, ACADO_VARIABLES );
	}

	rk_hess_tmp1 = ExportVariable( "rk_hess1", NX+NU, NX+NU, REAL, structWspace );
	rk_hess_tmp2 = ExportVariable( "rk_hess2", NX+NU, NX+NU, REAL, structWspace );
	rk_diffK_local = ExportVariable( "rk_diffKtraj_aux", N*grid.getNumIntervals()*(NX+NXA)*(NX+NU), numStages, REAL, structWspace );

    return SUCCESSFUL_RETURN;
}



// PROTECTED:


CLOSE_NAMESPACE_ACADO

// end of file.
