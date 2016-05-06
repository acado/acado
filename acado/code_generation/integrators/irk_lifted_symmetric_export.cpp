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
 *    \file src/code_generation/integrators/irk_lifted_symmetric_export.cpp
 *    \author Rien Quirynen
 *    \date 2015
 */

#include <acado/code_generation/integrators/irk_export.hpp>
#include <acado/code_generation/integrators/irk_lifted_symmetric_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

SymmetricLiftedIRKExport::SymmetricLiftedIRKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ForwardLiftedIRKExport( _userInteraction,_commonHeaderName )
{
}

SymmetricLiftedIRKExport::SymmetricLiftedIRKExport( const SymmetricLiftedIRKExport& arg ) : ForwardLiftedIRKExport( arg )
{
}


SymmetricLiftedIRKExport::~SymmetricLiftedIRKExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}


SymmetricLiftedIRKExport& SymmetricLiftedIRKExport::operator=( const SymmetricLiftedIRKExport& arg ){

    if( this != &arg ){

    	ForwardLiftedIRKExport::operator=( arg );
		copy( arg );
    }
    return *this;
}


ExportVariable SymmetricLiftedIRKExport::getAuxVariable() const
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


returnValue SymmetricLiftedIRKExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	ForwardLiftedIRKExport::getDataDeclarations( declarations, dataStruct );

	declarations.addDeclaration( rk_A_traj,dataStruct );
	declarations.addDeclaration( rk_aux_traj,dataStruct );
	declarations.addDeclaration( rk_S_traj,dataStruct );

	declarations.addDeclaration( rk_kkk_prev,dataStruct );
	declarations.addDeclaration( rk_kkk_delta,dataStruct );
	declarations.addDeclaration( rk_delta_full,dataStruct );
	declarations.addDeclaration( rk_diff_mu,dataStruct );
	declarations.addDeclaration( rk_diff_lam,dataStruct );

	declarations.addDeclaration( rk_lambda,dataStruct );
	declarations.addDeclaration( rk_b_mu,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue SymmetricLiftedIRKExport::getFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	ForwardLiftedIRKExport::getFunctionDeclarations( declarations );

    return SUCCESSFUL_RETURN;
}


returnValue SymmetricLiftedIRKExport::setDifferentialEquation(	const Expression& rhs_ )
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
//		adj_update << backwardDerivative( rhs_, x, lambda );

		// SECOND ORDER ADJOINT SWEEP:
		Expression arg;
		arg << x;
		arg << u;
		Expression S_tmp = sX;
		S_tmp.appendRows(zeros<double>(NU,NX).appendCols(eye<double>(NU)));

		if( NDX2 > 0 || NXA > 0 ) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);
		Expression dfS, dfL;
		Expression symmetric = symmetricDerivative( rhs_, arg, S_tmp, lambda, &dfS, &dfL );
		backward << dfL.getRows(0,NX);
		backward << returnLowerTriangular( symmetric );

		// GRADIENT UPDATE:
	    int gradientUp;
	    get( LIFTED_GRADIENT_UPDATE, gradientUp );
	    bool gradientUpdate = (bool) gradientUp;

	    uint nHat = 0;
	    if( gradientUpdate ) {
		    DifferentialState hat("",1,NX);
			Expression tmp = hat.getCols(0,NX)*dfL.getRows(0,NX);
			backward << multipleForwardDerivative( tmp, arg, S_tmp );
			nHat = hat.getNumCols();
	    }

		int linSolver;
		get( LINEAR_ALGEBRA_SOLVER, linSolver );
	    if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
			adj_update << dfL.getRows(0,NX);
	    }
	    else if( gradientUpdate ) {
	    	adj_update << (forwardDerivative( dfL, x )).transpose();
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


Expression SymmetricLiftedIRKExport::returnLowerTriangular( const Expression& expr ) {
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


returnValue SymmetricLiftedIRKExport::getCode(	ExportStatementBlock& code )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	int mode;
	get( IMPLICIT_INTEGRATOR_MODE, mode );
	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );
	int liftMode = 1;
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) liftMode = 4;
	if ( (ExportSensitivityType)sensGen != SYMMETRIC ) ACADOERROR( RET_INVALID_OPTION );
	if( (ImplicitIntegratorMode)mode != LIFTED ) ACADOERROR( RET_INVALID_OPTION );
	if( liftMode != 1 && liftMode != 4 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if( NXA > 0) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

    int gradientUp;
    get( LIFTED_GRADIENT_UPDATE, gradientUp );
    bool gradientUpdate = (bool) gradientUp;

	// NOTE: liftMode == 4 --> inexact Newton based implementation

	if( CONTINUOUS_OUTPUT || NX1 > 0 || NX3 > 0 || !equidistantControlGrid() ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	if( NX1 > 0 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	if( exportRhs ) {
		if( NX2 > 0 || NXA > 0 ) {
			code.addFunction( rhs );
			code.addStatement( "\n\n" );
			code.addFunction( diffs_rhs );
			code.addStatement( "\n\n" );
//			if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
				code.addFunction( diffs_sweep );
				code.addStatement( "\n\n" );
//			}
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
	if( liftMode == 1 ) {
		integrate.addStatement( rk_delta_full.getRow(0) == rk_delta );
	}
	integrate.addStatement( rk_xxx_lin == rk_eta.getCols(0,NX) );


	if( liftMode == 1 ) { // Compute mu variables for the EXACT scheme !!!
		// FORWARD loop for rk_delta_full:
		ExportForLoop deltaLoop( run, 0, grid.getNumIntervals() );

		deltaLoop.addStatement( k_index == (shooting_index*grid.getNumIntervals()+run)*(NX+NXA) );
		// FIRST update K variables using term from optimization variables:
		ExportForLoop loopTemp1( i,0,NX+NXA );
		loopTemp1.addStatement( j == k_index+i );
		loopTemp1.addStatement( tmp_index1 == j*(NX+NU) );
		ExportForLoop loopTemp2( run1,0,numStages );
		loopTemp2.addStatement( rk_kkk.getElement( j,run1 ) += rk_delta*rk_diffK.getSubMatrix( tmp_index1,tmp_index1+NX+NU,run1,run1+1 ) );
		loopTemp1.addStatement( loopTemp2 );
		deltaLoop.addStatement( loopTemp1 );

		deltaLoop.addStatement( tmp_index1 == shooting_index*grid.getNumIntervals()+run );
		deltaLoop.addStatement( rk_kkk_delta.getRows(run*NX,(run+1)*NX) == rk_kkk.getRows(tmp_index1*(NX+NXA),(tmp_index1+1)*(NX+NXA))-rk_kkk_prev.getRows(tmp_index1*(NX+NXA),(tmp_index1+1)*(NX+NXA)) );
		deltaLoop.addStatement( rk_kkk_prev.getRows(tmp_index1*(NX+NXA),(tmp_index1+1)*(NX+NXA)) == rk_kkk.getRows(tmp_index1*(NX+NXA),(tmp_index1+1)*(NX+NXA)) );
		// update rk_delta_full:
		deltaLoop.addStatement( rk_delta_full.getRow(run+1) == rk_delta_full.getRow(run) );
		for( run5 = 0; run5 < numStages; run5++ ) {
			deltaLoop.addStatement( rk_delta_full.getSubMatrix(run+1,run+2,0,NX) += Bh.getRow(run5)*(rk_kkk_delta.getSubMatrix( run*NX,(run+1)*NX,run5,run5+1 ).getTranspose()) );
		}
		DMatrix zeroV = zeros<double>(1,numStages*NX);
		deltaLoop.addStatement( rk_b_mu.getCols(run*numStages*NX,(run+1)*numStages*NX) == zeroV );
		integrate.addStatement( deltaLoop );

		// BACKWARD loop for lambda and MU variables:
		integrate.addStatement( rk_lambda.getRow(grid.getNumIntervals()) == rk_eta.getCols(NX,2*NX) );
		ExportForLoop lambdaLoop( run, grid.getNumIntervals()-1, -1, -1 );
		lambdaLoop.addStatement( tmp_index1 == shooting_index*grid.getNumIntervals()+run );
		// Compute \hat{lambda}:
		// vec(j*NX+1:j*NX+NX) = -Bh_vec(j+1)*dir_tmp;
		for( run5 = 0; run5 < numStages; run5++ ) {
			lambdaLoop.addStatement( rk_b_mu.getCols(run*numStages*NX+run5*NX,run*numStages*NX+(run5+1)*NX) -= Bh.getRow(run5)*rk_lambda.getRow(run+1) );
		}
		for( run5 = 0; run5 < numStages; run5++ ) {
			lambdaLoop.addStatement( rk_seed.getCols(0,NX+NU) == rk_delta_full.getRow(run) );
			ExportForLoop deltaLoop0( i, 0, numStages );
			deltaLoop0.addStatement( rk_seed.getCols(0,NX) += Ah.getElement(run5,i)*(rk_kkk_delta.getSubMatrix( run*NX,(run+1)*NX,i,i+1 ).getTranspose()) );
			lambdaLoop.addStatement( deltaLoop0 );

			for( run6 = 0; run6 < NX; run6++ ) {
				lambdaLoop.addStatement( rk_seed.getCol(NX+NU+run6) == rk_diff_mu.getSubMatrix(tmp_index1,tmp_index1+1,(run5*NX+run6)*(NX+NU),(run5*NX+run6+1)*(NX+NU))*(rk_seed.getCols(0,NX+NU).getTranspose()) );
			}
			for( run6 = 0; run6 < numStages; run6++ ) {
			lambdaLoop.addStatement( rk_b_mu.getCols(run*numStages*NX+run6*NX,run*numStages*NX+run6*NX+NX) -= Ah.getElement(run5,run6)*rk_seed.getCols(NX+NU,2*NX+NU) );
			}
			ExportForLoop deltaLoop3( i, 0, run );
			for( run6 = 0; run6 < numStages; run6++ ) {
				deltaLoop3.addStatement( rk_b_mu.getCols(i*numStages*NX+run6*NX,i*numStages*NX+run6*NX+NX) -= Bh.getRow(run6)*rk_seed.getCols(NX+NU,2*NX+NU) );
			}
			lambdaLoop.addStatement( deltaLoop3 );
		}
		lambdaLoop.addFunctionCall( solver->getNameSolveTransposeReuseFunction(),rk_A_traj.getAddress(tmp_index1*numStages*(NX2+NXA),0),rk_b_mu.getAddress(0,run*numStages*NX),rk_aux_traj.getAddress(tmp_index1,0) );
		lambdaLoop.addStatement( rk_adj_traj.getRow(tmp_index1) == rk_b_mu.getCols(run*numStages*NX,(run+1)*numStages*NX) );

		lambdaLoop.addStatement( rk_lambda.getRow(run) == rk_lambda.getRow(run+1) );
		for( run5 = 0; run5 < numStages; run5++ ) {
			lambdaLoop.addStatement( rk_lambda.getRow(run) += rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*NX,run5*NX+NX)*rk_diff_lam.getRows(tmp_index1*numStages*NX+run5*NX,tmp_index1*numStages*NX+run5*NX+NX) );
		}
		integrate.addStatement( lambdaLoop );
	}

	// set current Hessian to zero
	uint numX = NX*(NX+1)/2.0;
	uint numU = NU*(NU+1)/2.0;
	DMatrix zeroM;
	if( !gradientUpdate ) 	zeroM = zeros<double>(1,numX+numU+NX*NU);
	else 					zeroM = zeros<double>(1,numX+numU+NX*NU+NX+NU);
	integrate.addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) == zeroM );

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
		if( NX1 > 0 ) {
			ExportForLoop loopTemp1( i,0,NX1 );
			loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,0,NX1 ) == rk_eta.getCols( i*NX+2*NX+NXA,i*NX+2*NX+NXA+NX1 ) );
			if( NU > 0 ) loopTemp1.addStatement( rk_diffsPrev1.getSubMatrix( i,i+1,NX1,NX1+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+NX,i*NU+(NX+NXA)*(NX+1)+NX+NU ) );
			loop->addStatement( loopTemp1 );
		}
		if( NX2 > 0 ) {
			ExportForLoop loopTemp2( i,0,NX2 );
			loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,0,NX1+NX2 ) == rk_eta.getCols( i*NX+2*NX+NXA+NX1*NX,i*NX+2*NX+NXA+NX1*NX+NX1+NX2 ) );
			if( NU > 0 ) loopTemp2.addStatement( rk_diffsPrev2.getSubMatrix( i,i+1,NX1+NX2,NX1+NX2+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+NX1*NU+NX,i*NU+(NX+NXA)*(NX+1)+NX1*NU+NU+NX ) );
			loop->addStatement( loopTemp2 );
		}
		if( NX3 > 0 ) {
			ExportForLoop loopTemp3( i,0,NX3 );
			loopTemp3.addStatement( rk_diffsPrev3.getSubMatrix( i,i+1,0,NX ) == rk_eta.getCols( i*NX+2*NX+NXA+(NX1+NX2)*NX,i*NX+2*NX+NXA+(NX1+NX2)*NX+NX ) );
			if( NU > 0 ) loopTemp3.addStatement( rk_diffsPrev3.getSubMatrix( i,i+1,NX,NX+NU ) == rk_eta.getCols( i*NU+(NX+NXA)*(NX+1)+(NX1+NX2)*NU+NX,i*NU+(NX+NXA)*(NX+1)+(NX1+NX2)*NU+NU+NX ) );
			loop->addStatement( loopTemp3 );
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
	if( liftMode == 4 ) { // liftMode == 1 --> see earlier!
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
	if( gradientUpdate) { // LIFTING with GRADIENT UPDATE
		loop->addStatement( std::string("if( run > 0 ) {\n") );
		loop->addStatement( rk_Xhat_traj.getRows(run*NX,(run+1)*NX) == rk_Xhat_traj.getRows((run-1)*NX,run*NX) );
		for( run5 = 0; run5 < numStages; run5++ ) {
			loop->addStatement( rk_Xhat_traj.getRows(run*NX,(run+1)*NX) += rk_Khat_traj.getSubMatrix( run-1,run,run5*(NX2+NXA),(run5+1)*(NX2+NXA) ).getTranspose()*Bh.getElement(run5,0) );
		}
		if( (LinearAlgebraSolver) linSolver != SIMPLIFIED_IRK_NEWTON && (LinearAlgebraSolver) linSolver != SINGLE_IRK_NEWTON ) { // YOU NEED THE FULL rk_diffsTemp2 HERE !!
			ExportForLoop deltaLoop( i,0,numStages );
			ExportForLoop deltaLoop2( j,0,NX );
			deltaLoop2.addStatement( tmp_index1 == i*(NX2+NXA)+j );
			deltaLoop2.addStatement( tmp_index2 == numStages*k_index+tmp_index1 );
			// SAVE rk_diff_lam:
			deltaLoop2.addStatement( rk_diff_lam.getRow(tmp_index2) == rk_diffsTemp2.getSubMatrix(i,i+1,j*NVARS2,j*NVARS2+NX) );
			deltaLoop2.addStatement( rk_b.getElement(tmp_index1,0) -= rk_diffsTemp2.getSubMatrix(i,i+1,j*NVARS2,j*NVARS2+NX)*rk_Xhat_traj.getRows(run*NX,(run+1)*NX) );
			deltaLoop.addStatement( deltaLoop2 );
			loop->addStatement( deltaLoop );
		}
		loop->addStatement( std::string("}\nelse{\n") );
		loop->addStatement( rk_Xhat_traj.getRows(0,NX) == zeros<double>(NX,1) );
		loop->addStatement( std::string("}\n") );
	}

	// SAVE rk_A in the rk_A_traj variable:
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
		// YOU DO NOT NEED TO SAVE THE TRAJECTORY FOR THE INEXACT NEWTON SCHEME !
//		loop->addStatement( rk_A_traj.getRows(run*(NX2+NXA),(run+1)*(NX2+NXA)) == rk_A );
//		loop->addStatement( rk_aux_traj.getRow(run) == rk_auxSolver.makeRowVector() );
	}
	else if( liftMode == 1 ) {
		loop->addStatement( tmp_index1 == shooting_index*grid.getNumIntervals()+run );
		loop->addStatement( rk_A_traj.getRows(tmp_index1*numStages*(NX2+NXA),(tmp_index1+1)*numStages*(NX2+NXA)) == rk_A );
		loop->addStatement( rk_aux_traj.getRow(tmp_index1) == rk_auxSolver.makeRowVector() );
	}
	else {
		return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
		loop->addStatement( rk_A_traj.getRows(run*numStages*(NX2+NXA),(run+1)*numStages*(NX2+NXA)) == rk_A );
		loop->addStatement( rk_aux_traj.getRow(run) == rk_auxSolver.makeRowVector() );
	}

	if( liftMode == 1 ) {
//		// Evaluate sensitivities:
//		// !! NEW !! Let us propagate the forward sensitivities as in a VDE system
//		loop->addStatement( rk_seed.getCols(NX,NX+NX*(NX+NU)) == rk_diffsPrev2.makeRowVector() );
//		ExportForLoop loop_sens( i,0,numStages );
//		loop_sens.addStatement( rk_seed.getCols(0,NX) == rk_stageValues.getCols(i*(NX+NXA),i*(NX+NXA)+NX) );
//		loop_sens.addFunctionCall( forward_sweep.getName(), rk_seed, rk_diffsTemp2.getAddress(i,0) );
//		loop->addStatement( loop_sens );

		// Let us reuse the derivatives computed to form the linear system (in rk_diffsTemp2) !!

		evaluateRhsSensitivities( loop, run1, i, j, tmp_index1, tmp_index2 );
		allSensitivitiesImplicitSystem( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Bh, false );
	}
	else if( liftMode == 4 ) {
		evaluateRhsInexactSensitivities( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Ah );

		// GRADIENT UPDATE INIS SCHEME:
		if( gradientUpdate ) {
			loop->addStatement( tmp_index1 == shooting_index*grid.getNumIntervals()+run );
			loop->addStatement( rk_eta.getCols(NX+diffsDim-NX-NU,NX+diffsDim) += rk_adj_traj.getRow(tmp_index1)*rk_b.getCols(1,1+NX+NU) );
		}

		allSensitivitiesImplicitSystem( loop, run1, i, j, tmp_index1, tmp_index2, tmp_index3, k_index, Bh, true );
	}
	else return ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	if( liftMode == 1 ) { // EXACT LIFTING
		// !!! update rk_xxx_lin (YOU NEED TO UPDATE THE LINEARIZATION POINT BEFORE YOU UPDATE RK_KKK): !!!
		for( run5 = 0; run5 < NX; run5++ ) {
			loop->addStatement( rk_xxx_lin.getCol( run5 ) += rk_kkk.getRow( k_index+run5 )*Bh );
		}
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
	if( liftMode != 1 ) { // INEXACT LIFTING
		// !!! update rk_xxx_lin (YOU NEED TO UPDATE THE LINEARIZATION POINT AFTER YOU UPDATE RK_KKK): !!!
		loop->addStatement( rk_xxx_lin == rk_eta.getCols(0,NX) );
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

	ExportForLoop tmpLoop2( run, grid.getNumIntervals()-1, -1, -1 );
	ExportStatementBlock *loop2;
	if( equidistantControlGrid() ) {
		loop2 = &tmpLoop2;
	}
	else {
	    return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	}

	loop2->addStatement( k_index == (shooting_index*grid.getNumIntervals()+run)*(NX+NXA)*(NX+NU) );


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
			loop2->addStatement( rk_seed.getCols(0,NX) == rk_xxx_traj.getCols((run*numStages+run5)*(NX2+NXA),(run*numStages+run5+1)*(NX2+NXA)) );
			loop2->addStatement( rk_seed.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) == rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*(NX+NXA),(run5+1)*(NX+NXA)) );
			loop2->addFunctionCall( diffs_sweep.getName(), rk_seed, rk_adj_diffs_tmp );
			for( run6 = 0; run6 < numStages; run6++ ) {
				loop2->addStatement( rk_b_trans.getCols(run6*NX,(run6+1)*NX) -= Ah.getElement(run5,run6)*rk_adj_diffs_tmp.getCols(0,NX) );
			}
			loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) += rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*(NX+NXA),(run5+1)*(NX+NXA)) ); // BECAUSE EXPLICIT ODE
		}
		loop2->addFunctionCall( solver->getNameSolveTransposeReuseFunction(),rk_A.getAddress(0,0),rk_b_trans.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		loop2->addStatement( rk_adj_traj.getRow(tmp_index1) += rk_b_trans );
		loop2->addStatement( rk_b_trans == rk_adj_traj.getRow(tmp_index1) );
	}
	else { // EXACT SCHEME (liftMode == 1)
		// (RIEN) THIS IS A NEW PART TO MAKE IT EXACTLY EQUIVALENT TO DIRECT COLLOCATION:
		// WE SAVE THE SENSITIVITIES TO COMPUTE THE RIGHT-HAND SIDES TO UPDATE THE MU VARIABLES AT THE BEGINNING OF THE INTEGRATOR CALL
		loop2->addStatement( tmp_index1 == shooting_index*grid.getNumIntervals()+run );
		if( gradientUpdate ) {
			for( run5 = 0; run5 < numStages; run5++ ) {
				loop2->addStatement( rk_seed.getCols(0,NX) == rk_xxx_traj.getCols((run*numStages+run5)*(NX2+NXA),(run*numStages+run5+1)*(NX2+NXA)) );
				loop2->addStatement( rk_seed.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) == rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*(NX+NXA),(run5+1)*(NX+NXA)) );
//				loop2->addStatement( rk_seed.getCols(NX*(2+NX+NU),NX*(3+NX+NU)) == rk_delta );
//				loop2->addStatement( rk_seed.getCols(NX*(2+NX+NU),NX*(3+NX+NU)) += rk_Xhat_traj.getRows(run*NX,(run+1)*NX).getTranspose() );
//				ExportForLoop gradLoop( i, 0, numStages );
//				gradLoop.addStatement( rk_seed.getCols(NX*(2+NX+NU),NX*(3+NX+NU)) += Ah.getElement(run5,i)*rk_Khat_traj.getSubMatrix( run,run+1,i*(NX2+NXA),(i+1)*(NX2+NXA) ) );
//				loop2->addStatement( gradLoop );
				loop2->addFunctionCall( diffs_sweep.getName(), rk_seed, rk_adj_diffs_tmp );
				loop2->addStatement( rk_diff_mu.getSubMatrix(tmp_index1,tmp_index1+1,run5*NX*(NX+NU),(run5+1)*NX*(NX+NU)) == rk_adj_diffs_tmp.getCols(0,NX*(NX+NU)) );
//				for( run6 = 0; run6 < numStages; run6++ ) {
//					loop2->addStatement( rk_b_trans.getCols(run6*NX,(run6+1)*NX) -= Ah.getElement(run5,run6)*rk_adj_diffs_tmp.getCols(0,NX) );
//				}
				//			loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) += rk_adj_traj.getSubMatrix(tmp_index1,tmp_index1+1,run5*(NX+NXA),(run5+1)*(NX+NXA)) ); // BECAUSE EXPLICIT ODE (BUT ZERO FOR SECOND ORDER DERIVATIVES)
			}
		}
	}

	loop2->addStatement( tmp_index3 == shooting_index*grid.getNumIntervals()+run );
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
		loop2->addStatement( rk_seed.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) == rk_adj_traj.getSubMatrix(tmp_index3,tmp_index3+1,run5*NX,(run5+1)*NX) );
		if( gradientUpdate ) {
			loop2->addStatement( rk_seed.getCols(NX*(2+NX+NU),NX*(3+NX+NU)) == rk_Xhat_traj.getRows(run*NX,(run+1)*NX).getTranspose() );
			ExportForLoop gradLoop( i, 0, numStages );
			gradLoop.addStatement( rk_seed.getCols(NX*(2+NX+NU),NX*(3+NX+NU)) += Ah.getElement(run5,i)*rk_Khat_traj.getSubMatrix( run,run+1,i*(NX2+NXA),(i+1)*(NX2+NXA) ) );
			loop2->addStatement( gradLoop );
		}
		loop2->addFunctionCall( adjoint_sweep.getName(), rk_seed, rk_adj_diffs_tmp.getAddress(0,0) );
		loop2->addStatement( rk_eta.getCols(NX,2*NX) += rk_adj_diffs_tmp.getCols(0,NX) );
		if( !gradientUpdate ) {
			loop2->addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) += rk_adj_diffs_tmp.getCols(NX,NX+numX+numU+NX*NU) );
		}
		else {
			loop2->addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) += rk_adj_diffs_tmp.getCols(NX,NX+numX+numU+NX*NU+NX+NU) );
		}
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


returnValue SymmetricLiftedIRKExport::updateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
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


returnValue SymmetricLiftedIRKExport::setup( )
{
	ForwardLiftedIRKExport::setup();

	int gradientUp;
	get( LIFTED_GRADIENT_UPDATE, gradientUp );
	bool gradientUpdate = (bool) gradientUp;

	uint numX = NX*(NX+1)/2.0;
	uint numU = NU*(NU+1)/2.0;
	diffsDim   = NX + NX*(NX+NU) + numX + NX*NU + numU;
	if( gradientUpdate ) diffsDim += NX+NU;
	inputDim = NX + diffsDim + NU + NOD;

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;

	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );

	rk_b_trans = ExportVariable( "rk_b_trans", 1, numStages*(NX+NXA), REAL, structWspace );

	rk_adj_diffs_tmp = ExportVariable( "rk_adjoint", 1, NX + numX + NX*NU + numU, REAL, structWspace );
	if( gradientUpdate ) {
		if( (NX+numX+NX*NU+numU+NX+NU) > NX*(NX+NU) ) {
			rk_adj_diffs_tmp = ExportVariable( "rk_adjoint", 1, NX + numX + NX*NU + numU + NX+NU, REAL, structWspace );
		}
		else {
			rk_adj_diffs_tmp = ExportVariable( "rk_adjoint", 1, NX*(NX+NU), REAL, structWspace );
		}
		rk_Khat_traj = ExportVariable( "rk_Khat", grid.getNumIntervals(), numStages*(NX+NXA), REAL, structWspace );
		rk_Xhat_traj = ExportVariable( "rk_Xhat", grid.getNumIntervals()*NX, 1, REAL, structWspace );
	}

	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );
	int liftMode = 1;
	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) liftMode = 4;

	rk_seed = ExportVariable( "rk_seed", 1, NX+NX*(NX+NU)+NX+NU+NOD+timeDep, REAL, structWspace );
	if( gradientUpdate ) rk_seed = ExportVariable( "rk_seed", 1, NX+NX*(NX+NU)+2*NX+NU+NOD+timeDep, REAL, structWspace );
//	rk_A_traj = ExportVariable( "rk_A_traj", grid.getNumIntervals()*numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, structWspace );
//	rk_aux_traj = ExportVariable( "rk_aux_traj", grid.getNumIntervals(), numStages*(NX2+NXA), INT, structWspace );
	if( liftMode == 1 ) {
		rk_A_traj = ExportVariable( "rk_A_traj", N*grid.getNumIntervals()*numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, structWspace );
		rk_aux_traj = ExportVariable( "rk_aux_traj", N*grid.getNumIntervals(), numStages*(NX2+NXA), INT, structWspace );
	}
	rk_xxx_traj = ExportVariable( "rk_stageV_traj", 1, grid.getNumIntervals()*numStages*(NX+NXA), REAL, structWspace );
	rk_S_traj = ExportVariable( "rk_S_traj", grid.getNumIntervals()*NX, NX+NU, REAL, structWspace );

	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
//		rk_A = ExportVariable( "rk_J", NX2+NXA, NX2+NXA, REAL, structWspace );
//		if(NDX2 > 0) rk_I = ExportVariable( "rk_I", NX2+NXA, NX2+NXA, REAL, structWspace );
//		rk_A_traj = ExportVariable( "rk_J_traj", grid.getNumIntervals()*(NX2+NXA), NX2+NXA, REAL, structWspace );
//		rk_aux_traj = ExportVariable( "rk_aux_traj", grid.getNumIntervals(), rk_auxSolver.getNumRows()*rk_auxSolver.getNumCols(), INT, structWspace );
		rk_diffsTemp2 = ExportVariable( "rk_diffsTemp2", NX2+NXA, NVARS2, REAL, structWspace );
	}
	rk_adj_traj = ExportVariable( "rk_adj_traj", N*grid.getNumIntervals(), numStages*(NX+NXA), REAL, ACADO_VARIABLES );

	if( liftMode == 1 ) {
		rk_kkk_prev = ExportVariable( "rk_Ktraj_prev", N*grid.getNumIntervals()*(NX+NXA), numStages, REAL, ACADO_WORKSPACE );
		rk_kkk_delta = ExportVariable( "rk_K_delta", grid.getNumIntervals()*NX+NXA, numStages, REAL, ACADO_WORKSPACE );
		rk_delta_full = ExportVariable( "rk_delta_full", grid.getNumIntervals()+1, NX+NU, REAL, ACADO_WORKSPACE );
		rk_diff_mu = ExportVariable( "rk_diff_mu", N*grid.getNumIntervals(), numStages*(NX+NXA)*(NX+NU), REAL, ACADO_WORKSPACE );
		rk_diff_lam = ExportVariable( "rk_diff_lam", N*grid.getNumIntervals()*numStages*NX, NX, REAL, ACADO_WORKSPACE );
		rk_lambda = ExportVariable( "rk_lambda", grid.getNumIntervals()+1, NX, REAL, ACADO_WORKSPACE );
		rk_b_mu = ExportVariable( "rk_b_mu", 1, grid.getNumIntervals()*numStages*(NX+NXA), REAL, structWspace );
	}

    return SUCCESSFUL_RETURN;
}



// PROTECTED:


CLOSE_NAMESPACE_ACADO

// end of file.
