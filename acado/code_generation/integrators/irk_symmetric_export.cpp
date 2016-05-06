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
 *    \file src/code_generation/integrators/irk_symmetric_export.cpp
 *    \author Rien Quirynen
 *    \date 2016
 */

#include <acado/code_generation/integrators/irk_export.hpp>
#include <acado/code_generation/integrators/irk_symmetric_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

SymmetricIRKExport::SymmetricIRKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ForwardIRKExport( _userInteraction,_commonHeaderName )
{
}

SymmetricIRKExport::SymmetricIRKExport( const SymmetricIRKExport& arg ) : ForwardIRKExport( arg )
{
}


SymmetricIRKExport::~SymmetricIRKExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}


SymmetricIRKExport& SymmetricIRKExport::operator=( const SymmetricIRKExport& arg ){

    if( this != &arg ){

    	ForwardIRKExport::operator=( arg );
		copy( arg );
    }
    return *this;
}


ExportVariable SymmetricIRKExport::getAuxVariable() const
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
//		if( forward_sweep.getGlobalExportVariable().getDim() >= max.getDim() ) {
//			max = forward_sweep.getGlobalExportVariable();
//		}
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


returnValue SymmetricIRKExport::getDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	ForwardIRKExport::getDataDeclarations( declarations, dataStruct );

	declarations.addDeclaration( rk_A_traj,dataStruct );
	declarations.addDeclaration( rk_aux_traj,dataStruct );
	declarations.addDeclaration( rk_S_traj,dataStruct );
	declarations.addDeclaration( rk_xxx_traj,dataStruct );

	declarations.addDeclaration( rk_b_trans,dataStruct );
	declarations.addDeclaration( rk_seed,dataStruct );
	declarations.addDeclaration( rk_adj_diffs_tmp,dataStruct );

	declarations.addDeclaration( rk_hess_tmp1,dataStruct );
	declarations.addDeclaration( rk_hess_tmp2,dataStruct );

//	declarations.addDeclaration( rk_adj_traj,dataStruct );

//	declarations.addDeclaration( rk_diff_mu,dataStruct );
//	declarations.addDeclaration( rk_diff_lam,dataStruct );
//
//	declarations.addDeclaration( rk_lambda,dataStruct );
//	declarations.addDeclaration( rk_b_mu,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue SymmetricIRKExport::getFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	ForwardIRKExport::getFunctionDeclarations( declarations );

    return SUCCESSFUL_RETURN;
}


returnValue SymmetricIRKExport::setDifferentialEquation(	const Expression& rhs_ )
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
//		DifferentialEquation forward;
//		for( uint i = 0; i < rhs_.getDim(); i++ ) {
//			// NOT YET IMPLEMENTED FOR DAES OR IMPLICIT ODES
//			if( NDX2 > 0 || NXA > 0 ) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);
//			forward << multipleForwardDerivative( rhs_(i), x, Gx );
//			forward << multipleForwardDerivative( rhs_(i), x, Gu ) + forwardDerivative( rhs_(i), u );
//		}

		// FIRST ORDER ADJOINT SWEEP:
		DifferentialState lambda("", NX,1);
		DifferentialEquation backward;
//		, adj_update;
//		backward << backwardDerivative( rhs_, x, lambda );
//		adj_update << backwardDerivative( rhs_, x, lambda );

		// SECOND ORDER ADJOINT SWEEP:
		Expression arg;
		arg << x;
		arg << u;
		Expression S_tmp = sX;
		S_tmp.appendRows(zeros<double>(NU,NX).appendCols(eye<double>(NU)));

		if( NDX2 > 0 || NXA > 0 ) return ACADOERROR(RET_NOT_YET_IMPLEMENTED);
		if( (ExportSensitivityType)sensGen == SYMMETRIC ) {
			Expression dfS, dfL;
			Expression symmetric = symmetricDerivative( rhs_, arg, S_tmp, lambda, &dfS, &dfL );
			backward << dfL.getRows(0,NX);
			backward << returnLowerTriangular( symmetric );
		}
		else if( (ExportSensitivityType)sensGen == FORWARD_OVER_BACKWARD ) {
			Expression tmp = backwardDerivative( rhs_, arg, lambda );
			backward << tmp.getRows(0,NX);
			backward << multipleForwardDerivative( tmp, arg, S_tmp );
		}
		else {
			return ACADOERROR(RET_INVALID_OPTION);
		}

		// GRADIENT UPDATE:
//	    int gradientUp;
//	    get( LIFTED_GRADIENT_UPDATE, gradientUp );
//	    bool gradientUpdate = (bool) gradientUp;

	    uint nHat = 0;
//	    if( gradientUpdate ) {
//		    DifferentialState hat("",1,NX);
//			Expression tmp = hat.getCols(0,NX)*dfL.getRows(0,NX);
//			backward << multipleForwardDerivative( tmp, arg, S_tmp );
//			nHat = hat.getNumCols();
//	    }

//		int linSolver;
//		get( LINEAR_ALGEBRA_SOLVER, linSolver );
//	    if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
//			adj_update << dfL.getRows(0,NX);
//	    }
//	    else if( gradientUpdate ) {
//	    	adj_update << (forwardDerivative( dfL, x )).transpose();
//	    }

		if( f.getNT() > 0 ) timeDependant = true;

		return (rhs.init( f,"acado_rhs",NX,NXA,NU,NP,NDX,NOD ) &
				diffs_rhs.init( g,"acado_diffs",NX,NXA,NU,NP,NDX,NOD ) &
//				forward_sweep.init( forward,"acado_forward",NX*(2+NX+NU)+nHat,NXA,NU,NP,NDX,NOD ) &
				adjoint_sweep.init( backward,"acado_backward",NX*(2+NX+NU)+nHat,NXA,NU,NP,NDX,NOD ));
//				diffs_sweep.init( adj_update,"acado_adjoint_update",NX*(2+NX+NU)+nHat,NXA,NU,NP,NDX,NOD ));
	}
	return SUCCESSFUL_RETURN;
}


Expression SymmetricIRKExport::returnLowerTriangular( const Expression& expr ) {
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


returnValue SymmetricIRKExport::getCode(	ExportStatementBlock& code )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY, sensGen );
	int mode;
	get( IMPLICIT_INTEGRATOR_MODE, mode );
//	int liftMode;
//	get( LIFTED_INTEGRATOR_MODE, liftMode );
	if ( (ExportSensitivityType)sensGen != SYMMETRIC && (ExportSensitivityType)sensGen != FORWARD_OVER_BACKWARD ) ACADOERROR( RET_INVALID_OPTION );
	if( (ImplicitIntegratorMode)mode == LIFTED ) ACADOERROR( RET_INVALID_OPTION );
//	if( liftMode != 1 && liftMode != 4 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	if( NXA > 0) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

//    int gradientUp;
//    get( LIFTED_GRADIENT_UPDATE, gradientUp );
//    bool gradientUpdate = (bool) gradientUp;

	// NOTE: liftMode == 4 --> inexact Newton based implementation

	if( CONTINUOUS_OUTPUT || NX1 > 0 || NX3 > 0 || !equidistantControlGrid() ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	if( NX1 > 0 ) ACADOERROR( RET_NOT_IMPLEMENTED_YET );

	int linSolver;
	get( LINEAR_ALGEBRA_SOLVER, linSolver );
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
//			if( liftMode == 4 ) { // ONLY for the inexact Newton based schemes
//				code.addFunction( forward_sweep );
//				code.addStatement( "\n\n" );
//			}
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
//	ExportIndex tmp_index3("tmp_index3");
//	ExportIndex tmp_index4("tmp_index4");
//	ExportIndex shooting_index("shoot_index");
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
//	integrate.addIndex( tmp_index3 );
//	integrate.addIndex( shooting_index );
//	if( rk_outputs.size() > 0 && (grid.getNumIntervals() > 1 || !equidistantControlGrid()) ) {
//		integrate.addIndex( tmp_index4 );
//	}
//	integrate << shooting_index.getFullName() << " = " << rk_index.getFullName() << ";\n";
	integrate.addStatement( rk_ttt == DMatrix(grid.getFirstTime()) );
	if( (inputDim-diffsDim) > NX+NXA ) {
		integrate.addStatement( rk_xxx.getCols( NX+NXA,inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
//		if( !gradientUpdate )
			integrate.addStatement( rk_seed.getCols( 2*NX+NX*(NX+NU),NX+NX*(NX+NU)+inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
//		else integrate.addStatement( rk_seed.getCols( 3*NX+NX*(NX+NU),2*NX+NX*(NX+NU)+inputDim-diffsDim ) == rk_eta.getCols( NX+NXA+diffsDim,inputDim ) );
	}
	integrate.addLinebreak( );

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

	// SAVE rk_diffsPrev2 in the rk_S_traj variable:
	loop->addStatement( rk_S_traj.getRows(run*NX,(run+1)*NX) == rk_diffsPrev2 );

	// PART 2: The fully implicit system
	loop->addStatement( tmp_index2 == run*(NX+NXA) );
	solveImplicitSystem( loop, i, run1, j, tmp_index1, tmp_index2, Ah, C, determinant, true );

	// Evaluate all stage values for reuse:
	evaluateAllStatesImplicitSystem( loop, tmp_index2, Ah, C, run1, j, tmp_index1, i );

	// DERIVATIVES wrt the states (IFT):
	if( NX2 > 0 ) {
		ExportForLoop loop4( run1,NX1,NX1+NX2 );
		// PART 2: The fully implicit system
		sensitivitiesImplicitSystem( &loop4, run1, i, j, tmp_index1, tmp_index2, run, Ah, Bh, determinant, true, 2 );
		loop->addStatement( loop4 );
	}

	// DERIVATIVES wrt the control inputs (IFT):
	if( NU > 0 ) {
		ExportForLoop loop5( run1,0,NU );
		// PART 2: The fully implicit system
		sensitivitiesImplicitSystem( &loop5, run1, i, j, tmp_index1, tmp_index2, run, Ah, Bh, determinant, false, 0 );
		loop->addStatement( loop5 );
	}

	// SAVE rk_A in the rk_A_traj variable:
	loop->addStatement( rk_A_traj.getRows(run*numStages*(NX2+NXA),(run+1)*numStages*(NX2+NXA)) == rk_A );
	loop->addStatement( rk_aux_traj.getRow(run) == rk_auxSolver.makeRowVector() );

//	// update rk_diffsNew with the new sensitivities:
//	ExportForLoop loop3( i,0,NX2 );
//	loop3.addStatement( tmp_index1 == (run*(NX+NXA) + NX1 + i)*(NX+NU) );
//	ExportForLoop loop31( j,0,NX2+NU );
//	loop31.addStatement( tmp_index2 == tmp_index1+j );
//	loop31.addStatement( rk_diffsNew2.getElement( i,j ) == rk_diffsPrev2.getElement( i,j ) );
//	loop31.addStatement( rk_diffsNew2.getElement( i,j ) += rk_diffK.getRow( tmp_index2 )*Bh );
//	loop3.addStatement( loop31 );
//	loop->addStatement( loop3 );
//
//	if( NXA > 0 ) {
//		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );
//		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
//			loop->addStatement( std::string("if( run == 0 ) {\n") );
//		}
//		ExportForLoop loop5( i,0,NXA );
//		loop5.addStatement( tmp_index1 == (run*(NX+NXA) + NX + i)*(NX+NU) );
//		ExportForLoop loop51( j,0,NX2+NU );
//		loop51.addStatement( tmp_index2 == tmp_index1+j );
//		loop51.addStatement( rk_diffsNew2.getElement( i+NX2,j ) == rk_diffK.getRow( tmp_index2 )*tempCoefs );
//		loop5.addStatement( loop51 );
//		loop->addStatement( loop5 );
//		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
//			loop->addStatement( std::string("}\n") );
//		}
//	}

	// update rk_eta:
	for( run5 = 0; run5 < NX; run5++ ) {
		loop->addStatement( rk_eta.getCol( run5 ) += rk_kkk.getRow( run*(NX+NXA)+run5 )*Bh );
	}
	if( NXA > 0) {
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );
		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
			loop->addStatement( std::string("if( run == 0 ) {\n") );
		}
		for( run5 = 0; run5 < NXA; run5++ ) {
			loop->addStatement( rk_eta.getCol( NX+run5 ) == rk_kkk.getRow( run*(NX+NXA)+NX+run5 )*tempCoefs );
		}
		if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
			loop->addStatement( std::string("}\n") );
		}
	}

	// Computation of the sensitivities using the CHAIN RULE:
	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( std::string( "if( run == 0 ) {\n" ) );
	}
	// PART 2
	updateImplicitSystem(loop, i, j, tmp_index2);

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( std::string( "}\n" ) );
		loop->addStatement( std::string( "else {\n" ) );
		// PART 2
		propagateImplicitSystem(loop, i, j, k, tmp_index2);
	}

	if( grid.getNumIntervals() > 1 || !equidistantControlGrid() ) {
		loop->addStatement( std::string( "}\n" ) );
	}

	loop->addStatement( std::string( reset_int.get(0,0) ) + " = 0;\n" );

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
	uint numX = NX*(NX+1)/2.0;
	uint numU = NU*(NU+1)/2.0;
	DMatrix zeroM;
	if( (ExportSensitivityType)sensGen == SYMMETRIC ) {
		zeroM = zeros<double>(1,numX+numU+NX*NU);
	}
	else {
		zeroM = zeros<double>(1,NX*(NX+NU)+NU*NU);
	}
    integrate.addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) == zeroM );
	ExportForLoop tmpLoop2( run, grid.getNumIntervals()-1, -1, -1 );
	ExportStatementBlock *loop2;
	if( equidistantControlGrid() ) {
		loop2 = &tmpLoop2;
	}
	else {
	    return ACADOERROR( RET_NOT_IMPLEMENTED_YET );
	}


	// Compute \hat{lambda}:
	// vec(j*NX+1:j*NX+NX) = -Bh_vec(j+1)*dir_tmp;
	for( run5 = 0; run5 < numStages; run5++ ) {
		DMatrix zeroV = zeros<double>(1,NX);
		loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) == zeroV );
		loop2->addStatement( rk_b_trans.getCols(run5*NX,(run5+1)*NX) -= Bh.getRow(run5)*rk_eta.getCols(NX,2*NX) );
	}
	loop2->addFunctionCall( solver->getNameSolveTransposeReuseFunction(),rk_A_traj.getAddress(run*numStages*(NX+NXA),0),rk_b_trans.getAddress(0,0),rk_aux_traj.getAddress(run,0) );

	if( (ExportSensitivityType)sensGen == FORWARD_OVER_BACKWARD ) {
		zeroM = zeros<double>(NX+NU,NX+NU);
		loop2->addStatement( rk_hess_tmp1 == zeroM );
	}
	for( run5 = 0; run5 < numStages; run5++ ) {
		loop2->addStatement( rk_seed.getCols(0,NX) == rk_xxx_traj.getCols((run*numStages+run5)*(NX2+NXA),(run*numStages+run5+1)*(NX2+NXA)) );
		loop2->addStatement( rk_diffsPrev2 == rk_S_traj.getRows(run*NX,(run+1)*NX) );

		ExportForLoop diffLoop1( i, 0, NX );
		diffLoop1.addStatement( tmp_index1 == (run*(NX+NXA)+i)*(NX+NU) );
		ExportForLoop diffLoop2( j, 0, NX+NU );
		diffLoop2.addStatement( tmp_index2 == tmp_index1+j );
		for( run6 = 0; run6 < numStages; run6++ ) {
			diffLoop2.addStatement( rk_diffsPrev2.getElement(i,j) += Ah.getElement(run5,run6)*rk_diffK.getElement( tmp_index2,run6 ) );
		}
		diffLoop1.addStatement( diffLoop2 );
		loop2->addStatement( diffLoop1 );

		loop2->addStatement( rk_seed.getCols(NX,NX*(1+NX+NU)) == rk_diffsPrev2.makeRowVector() );
		loop2->addStatement( rk_seed.getCols(NX*(1+NX+NU),NX*(2+NX+NU)) == rk_b_trans.getCols(run5*NX,(run5+1)*NX) );
		loop2->addFunctionCall( adjoint_sweep.getName(), rk_seed, rk_adj_diffs_tmp.getAddress(0,0) );
		loop2->addStatement( rk_eta.getCols(NX,2*NX) += rk_adj_diffs_tmp.getCols(0,NX) );
		if( (ExportSensitivityType)sensGen == SYMMETRIC ) {
			loop2->addStatement( rk_eta.getCols(NX*(2+NX+NU),NX+diffsDim) += rk_adj_diffs_tmp.getCols(NX,NX+numX+numU+NX*NU) );
		}
		else {
			for( run6 = 0; run6 < NX+NU; run6++ ) {
				loop2->addStatement( rk_hess_tmp2.getRow(run6) == rk_adj_diffs_tmp.getCols(NX+run6*(NX+NU),NX+(run6+1)*(NX+NU)) );
			}
			// compute the local derivatives in rk_diffsPrev2
			loop2->addStatement( rk_diffsPrev2 == eyeM );
			ExportForLoop diffLoop3( i, 0, NX );
			diffLoop3.addStatement( tmp_index1 == (run*(NX+NXA)+i)*(NX+NU) );
			ExportForLoop diffLoop4( j, 0, NX+NU );
			diffLoop4.addStatement( tmp_index2 == tmp_index1+j );
			for( run6 = 0; run6 < numStages; run6++ ) {
				diffLoop4.addStatement( rk_diffsPrev2.getElement(i,j) += Ah.getElement(run5,run6)*rk_diffK.getElement( tmp_index2,run6 ) );
			}
			diffLoop3.addStatement( diffLoop4 );
			loop2->addStatement( diffLoop3 );

			// update of rk_hess_tmp2 from the left and add it to rk_hess_tmp1
			updateHessianTerm( loop2, i, j );
		}
	}

	if( (ExportSensitivityType)sensGen == FORWARD_OVER_BACKWARD ) {
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
		diffLoop5.addStatement( tmp_index1 == (run*(NX+NXA)+i)*(NX+NU) );
		ExportForLoop diffLoop6( j, 0, NX+NU );
		diffLoop6.addStatement( tmp_index2 == tmp_index1+j );
		diffLoop6.addStatement( rk_diffsPrev2.getElement(i,j) += rk_diffK.getRow( tmp_index2 )*Bh );
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


returnValue SymmetricIRKExport::updateHessianTerm( ExportStatementBlock* block, const ExportIndex& i, const ExportIndex& j ) {
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


returnValue SymmetricIRKExport::evaluateAllStatesImplicitSystem( ExportStatementBlock* block, const ExportIndex& k_index, const ExportVariable& Ah, const ExportVariable& C, const ExportIndex& stage, const ExportIndex& i, const ExportIndex& tmp_index, const ExportIndex& tmp_index2 )
{
	ExportForLoop loop0( stage,0,numStages );
	ExportForLoop loop1( i, 0, NX1+NX2 );
	loop1.addStatement( tmp_index == k_index*numStages + stage*(NX+NXA) );
	loop1.addStatement( tmp_index2 == k_index+i );
	loop1.addStatement( rk_xxx_traj.getCol( tmp_index+i ) == rk_eta.getCol( i ) );
	for( uint j = 0; j < numStages; j++ ) {
		loop1.addStatement( rk_xxx_traj.getCol( tmp_index+i ) += Ah.getElement(stage,j)*rk_kkk.getElement( tmp_index2,j ) );
	}
	loop0.addStatement( loop1 );

	ExportForLoop loop3( i, 0, NXA );
	loop3.addStatement( tmp_index == k_index*numStages + stage*(NX+NXA)+NX );
	loop3.addStatement( tmp_index2 == k_index+NX+i );
	loop3.addStatement( rk_xxx_traj.getCol( tmp_index+i ) == rk_kkk.getElement( tmp_index2,stage ) );
	loop0.addStatement( loop3 );
	block->addStatement( loop0 );

	return SUCCESSFUL_RETURN;
}


returnValue SymmetricIRKExport::updateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& tmp_index )
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
		ExportForLoop loop01( index1,NX,NX+NXA );
		ExportForLoop loop02( index2,0,NX1+NX2 );
		loop02.addStatement( tmp_index == index2+index1*NX );
		loop02.addStatement( rk_eta.getCol( tmp_index+2*NX+NXA ) == rk_diffsNew2.getSubMatrix( index1-NX1-NX3,index1-NX1-NX3+1,index2,index2+1 ) );
		loop01.addStatement( loop02 );

		if( NU > 0 ) {
			ExportForLoop loop03( index2,0,NU );
			loop03.addStatement( tmp_index == index2+index1*NU );
			loop03.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX)+NX ) == rk_diffsNew2.getSubMatrix( index1-NX1-NX3,index1-NX1-NX3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			loop01.addStatement( loop03 );
		}
		block->addStatement( loop01 );
	}

	return SUCCESSFUL_RETURN;
}


returnValue SymmetricIRKExport::propagateImplicitSystem(	ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2,
																const ExportIndex& _index3, const ExportIndex& tmp_index )
{
	uint index3; // index3 instead of _index3 to unroll loops
	if( NX2 > 0 ) {
		ExportForLoop loop01( index1,NX1,NX1+NX2 );
		if( NX1 > 0 ) {
			ExportForLoop loop02( index2,0,NX1 );
			loop02.addStatement( tmp_index == index2+index1*NX );
			loop02.addStatement( rk_eta.getCol( tmp_index+2*NX+NXA ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,0,1 )*rk_diffsPrev1.getSubMatrix( 0,1,index2,index2+1 ) );
			for( index3 = 1; index3 < NX1; index3++ ) {
				loop02.addStatement( rk_eta.getCol( tmp_index+2*NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,index2,index2+1 ) );
			}
			for( index3 = 0; index3 < NX2; index3++ ) {
				loop02.addStatement( rk_eta.getCol( tmp_index+2*NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
			}
			loop01.addStatement( loop02 );
		}

		ExportForLoop loop05( index2,NX1,NX1+NX2 );
		loop05.addStatement( tmp_index == index2+index1*NX );
		loop05.addStatement( rk_eta.getCol( tmp_index+2*NX+NXA ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1,NX1+1 )*rk_diffsPrev2.getSubMatrix( 0,1,index2,index2+1 ) );
		for( index3 = 1; index3 < NX2; index3++ ) {
			loop05.addStatement( rk_eta.getCol( tmp_index+2*NX+NXA ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,index2,index2+1 ) );
		}
		loop01.addStatement( loop05 );

		if( NU > 0 ) {
			ExportForLoop loop07( index2,0,NU );
			loop07.addStatement( tmp_index == index2+index1*NU );
			loop07.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX)+NX ) == rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			for( index3 = 0; index3 < NX1; index3++ ) {
				loop07.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX)+NX ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,index3,index3+1 )*rk_diffsPrev1.getSubMatrix( index3,index3+1,NX1+index2,NX1+index2+1 ) );
			}
			for( index3 = 0; index3 < NX2; index3++ ) {
				loop07.addStatement( rk_eta.getCol( tmp_index+(NX+NXA)*(1+NX)+NX ) += rk_diffsNew2.getSubMatrix( index1-NX1,index1-NX1+1,NX1+index3,NX1+index3+1 )*rk_diffsPrev2.getSubMatrix( index3,index3+1,NX1+NX2+index2,NX1+NX2+index2+1 ) );
			}
			loop01.addStatement( loop07 );
		}
		block->addStatement( loop01 );
	}
	// ALGEBRAIC STATES: NO PROPAGATION OF SENSITIVITIES NEEDED

	return SUCCESSFUL_RETURN;
}


returnValue SymmetricIRKExport::sensitivitiesImplicitSystem( ExportStatementBlock* block, const ExportIndex& index1, const ExportIndex& index2, const ExportIndex& index3, const ExportIndex& tmp_index1, const ExportIndex& tmp_index2, const ExportIndex& stepI, const ExportVariable& Ah, const ExportVariable& Bh, const ExportVariable& det, bool STATES, uint number )
{
	if( NX2 > 0 ) {
		DMatrix zeroM = zeros<double>( NX2+NXA,1 );
		DMatrix tempCoefs( evaluateDerivedPolynomial( 0.0 ) );
		uint i;

		ExportForLoop loop1( index2,0,numStages );
		if( STATES && number == 1 ) {
//			ExportForLoop loop2( index3,0,NX1 );
//			loop2.addStatement( std::string(rk_rhsTemp.get( index3,0 )) + " = -(" + index3.getName() + " == " + index1.getName() + ");\n" );
//			for( i = 0; i < numStages; i++ ) {
//				loop2.addStatement( rk_rhsTemp.getRow( index3 ) -= rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
//			}
//			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,0,NX2+NXA );
			loop3.addStatement( tmp_index1 == index2*(NX2+NXA)+index3 );
			loop3.addStatement( rk_b.getRow( tmp_index1 ) == rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2),index3*(NVARS2)+NX1 )*rk_rhsTemp.getRows(0,NX1) );
//			if( NDX2 > 0 ) {
//				loop3.addStatement( rk_b.getRow( tmp_index1 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2)+NVARS2-NX1-NX2,index3*(NVARS2)+NVARS2-NX2 )*rk_diffK.getSubMatrix( 0,NX1,index2,index2+1 ) );
//			}
			loop1.addStatement( loop3 );
		}
		else if( STATES && number == 2 ) {
			for( i = 0; i < NX2+NXA; i++ ) {
				loop1.addStatement( rk_b.getRow( index2*(NX2+NXA)+i ) == zeroM.getRow( 0 ) - rk_diffsTemp2.getElement( index2,index1+i*(NVARS2) ) );
			}
		}
		else { // ~= STATES
//			ExportForLoop loop2( index3,0,NX1 );
//			loop2.addStatement( rk_rhsTemp.getRow( index3 ) == rk_diffK.getElement( index3,0 )*Ah.getElement(index2,0) );
//			for( i = 1; i < numStages; i++ ) {
//				loop2.addStatement( rk_rhsTemp.getRow( index3 ) += rk_diffK.getElement( index3,i )*Ah.getElement(index2,i) );
//			}
//			loop1.addStatement( loop2 );
			ExportForLoop loop3( index3,0,NX2+NXA );
			loop3.addStatement( tmp_index1 == index2*(NX2+NXA)+index3 );
			loop3.addStatement( tmp_index2 == index1+index3*(NVARS2) );
			loop3.addStatement( rk_b.getRow( tmp_index1 ) == zeroM.getRow( 0 ) - rk_diffsTemp2.getElement( index2,tmp_index2+NX1+NX2+NXA ) );
			loop3.addStatement( rk_b.getRow( tmp_index1 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2),index3*(NVARS2)+NX1 )*rk_rhsTemp.getRows(0,NX1) );
//			if( NDX2 > 0 ) {
//				loop3.addStatement( rk_b.getRow( tmp_index1 ) -= rk_diffsTemp2.getSubMatrix( index2,index2+1,index3*(NVARS2)+NVARS2-NX1-NX2,index3*(NVARS2)+NVARS2-NX2 )*rk_diffK.getSubMatrix( 0,NX1,index2,index2+1 ) );
//			}
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
		for( i = 0; i < NX; i++ ) {
			if( STATES ) 	loop2.addStatement( tmp_index1 == (stepI*(NX+NXA)+i)*(NX+NU)+index1);
			else			loop2.addStatement( tmp_index1 == (stepI*(NX+NXA)+i)*(NX+NU)+NX+index1);
			loop2.addStatement( rk_diffK.getElement(tmp_index1,index2) == rk_b.getRow(index2*NX2+i) );
		}
		for( i = 0; i < NXA; i++ ) {
			if( STATES ) 	loop2.addStatement( tmp_index1 == (stepI*(NX+NXA)+NX+i)*(NX+NU)+index1);
			else			loop2.addStatement( tmp_index1 == (stepI*(NX+NXA)+NX+i)*(NX+NU)+NX+index1);
			loop2.addStatement( rk_diffK.getElement(tmp_index1,index2) == rk_b.getRow(numStages*NX+index2*NXA+i) );
		}
		block->addStatement( loop2 );
		// update rk_diffsNew with the new sensitivities:
		ExportForLoop loop3( index2,0,NX2 );
		if( STATES && number == 2 ) loop3.addStatement( std::string(rk_diffsNew2.get( index2,index1 )) + " = (" + index2.getName() + " == " + index1.getName() + "-" + toString(NX1) + ");\n" );

		if( STATES && number == 2 ) {
			loop3.addStatement( tmp_index1 == (stepI*(NX+NXA)+index2)*(NX+NU));
			loop3.addStatement( tmp_index2 == tmp_index1+index1);
			loop3.addStatement( rk_diffsNew2.getElement( index2,index1 ) += rk_diffK.getRow( tmp_index2 )*Bh );
		}
		else if( STATES ) return ACADOERROR( RET_INVALID_OPTION );
		else {
			loop3.addStatement( tmp_index1 == (stepI*(NX+NXA)+index2)*(NX+NU));
			loop3.addStatement( tmp_index2 == tmp_index1+NX+index1);
			loop3.addStatement( rk_diffsNew2.getElement( index2,NX+index1 ) == rk_diffK.getRow( tmp_index2 )*Bh );
		}
//		if( STATES && number == 2 ) loop3.addStatement( rk_diffsNew2.getElement( index2,index1 ) += rk_diffK.getRow( NX1+index2 )*Bh );
//		else if( STATES )	loop3.addStatement( rk_diffsNew2.getElement( index2,index1 ) == rk_diffK.getRow( NX1+index2 )*Bh );
//		else		 		loop3.addStatement( rk_diffsNew2.getElement( index2,index1+NX1+NX2 ) == rk_diffK.getRow( NX1+index2 )*Bh );

		block->addStatement( loop3 );
		if( NXA > 0 ) {
			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
				block->addStatement( std::string("if( run == 0 ) {\n") );
			}
			ExportForLoop loop4( index2,0,NXA );

			if( STATES ) {
				loop4.addStatement( tmp_index1 == (stepI*(NX+NXA)+NX+index2)*(NX+NU));
				loop4.addStatement( tmp_index2 == tmp_index1+index1);
				loop4.addStatement( rk_diffsNew2.getElement( NX+index2,index1 ) == rk_diffK.getRow( tmp_index2 )*tempCoefs );
			}
			else {
				loop4.addStatement( tmp_index1 == (stepI*(NX+NXA)+NX+index2)*(NX+NU));
				loop4.addStatement( tmp_index2 == tmp_index1+NX+index1);
				loop4.addStatement( rk_diffsNew2.getElement( NX+index2,NX+index1 ) == rk_diffK.getRow( tmp_index2 )*tempCoefs );
			}

//			if( STATES ) loop4.addStatement( rk_diffsNew2.getElement( index2+NX2,index1 ) == rk_diffK.getRow( NX+index2 )*tempCoefs );
//			else 		 loop4.addStatement( rk_diffsNew2.getElement( index2+NX2,index1+NX1+NX2 ) == rk_diffK.getRow( NX+index2 )*tempCoefs );
			block->addStatement( loop4 );
			if( !equidistantControlGrid() || grid.getNumIntervals() > 1 ) {
				block->addStatement( std::string("}\n") );
			}
		}
	}

	return SUCCESSFUL_RETURN;
}


returnValue SymmetricIRKExport::setup( )
{
	ForwardIRKExport::setup();
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );

//	int gradientUp;
//	get( LIFTED_GRADIENT_UPDATE, gradientUp );
//	bool gradientUpdate = (bool) gradientUp;

	uint numX = NX*(NX+1)/2.0;
	uint numU = NU*(NU+1)/2.0;
	if( (ExportSensitivityType)sensGen == SYMMETRIC ) {
		diffsDim   = NX + NX*(NX+NU) + numX + NX*NU + numU;
	}
	else {
		diffsDim   = NX + NX*(NX+NU) + NX*(NX+NU)+NU*NU;
	}
//	if( gradientUpdate ) diffsDim += NX+NU;
	inputDim = NX + diffsDim + NU + NOD;

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;

	rk_eta = ExportVariable( "rk_eta", 1, inputDim, REAL );
	rk_kkk = ExportVariable( "rk_Ktraj", grid.getNumIntervals()*(NX+NXA), numStages, REAL, structWspace );
	rk_diffK = ExportVariable( "rk_diffKtraj", grid.getNumIntervals()*(NX+NXA)*(NX+NU), numStages, REAL, structWspace );

	rk_seed = ExportVariable( "rk_seed", 1, NX+NX*(NX+NU)+NX+NU+NOD+timeDep, REAL, structWspace );
	rk_b_trans = ExportVariable( "rk_b_trans", 1, numStages*(NX+NXA), REAL, structWspace );

//	int liftMode;
//	get( LIFTED_INTEGRATOR_MODE, liftMode );

	rk_seed = ExportVariable( "rk_seed", 1, NX+NX*(NX+NU)+NX+NU+NOD+timeDep, REAL, structWspace );
	if( (ExportSensitivityType)sensGen == SYMMETRIC ) {
		rk_adj_diffs_tmp = ExportVariable( "rk_adjoint", 1, NX + numX + NX*NU + numU, REAL, structWspace );
	}
	else {
		rk_adj_diffs_tmp = ExportVariable( "rk_adjoint", 1, NX + (NX+NU)*(NX+NU), REAL, structWspace );
	}
//	if( gradientUpdate ) rk_seed = ExportVariable( "rk_seed", 1, NX+NX*(NX+NU)+2*NX+NU+NOD+timeDep, REAL, structWspace );
//	rk_A_traj = ExportVariable( "rk_A_traj", grid.getNumIntervals()*numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, structWspace );
//	rk_aux_traj = ExportVariable( "rk_aux_traj", grid.getNumIntervals(), numStages*(NX2+NXA), INT, structWspace );

	rk_A_traj = ExportVariable( "rk_A_traj", grid.getNumIntervals()*numStages*(NX2+NXA), numStages*(NX2+NXA), REAL, structWspace );
	rk_aux_traj = ExportVariable( "rk_aux_traj", grid.getNumIntervals(), numStages*(NX2+NXA), INT, structWspace );

	rk_xxx_traj = ExportVariable( "rk_stageV_traj", 1, grid.getNumIntervals()*numStages*(NX+NXA), REAL, structWspace );
	rk_diffsPrev2 = ExportVariable( "rk_diffsPrev2", NX2, NX1+NX2+NU, REAL, structWspace );
	rk_S_traj = ExportVariable( "rk_S_traj", grid.getNumIntervals()*NX, NX+NU, REAL, structWspace );

//	int linSolver;
//	get( LINEAR_ALGEBRA_SOLVER, linSolver );
//	if( (LinearAlgebraSolver) linSolver == SIMPLIFIED_IRK_NEWTON || (LinearAlgebraSolver) linSolver == SINGLE_IRK_NEWTON ) {
//		rk_A = ExportVariable( "rk_J", NX2+NXA, NX2+NXA, REAL, structWspace );
//		if(NDX2 > 0) rk_I = ExportVariable( "rk_I", NX2+NXA, NX2+NXA, REAL, structWspace );
//		rk_A_traj = ExportVariable( "rk_J_traj", grid.getNumIntervals()*(NX2+NXA), NX2+NXA, REAL, structWspace );
//		rk_aux_traj = ExportVariable( "rk_aux_traj", grid.getNumIntervals(), rk_auxSolver.getNumRows()*rk_auxSolver.getNumCols(), INT, structWspace );
//		rk_diffsTemp2 = ExportVariable( "rk_diffsTemp2", NX2+NXA, NVARS2, REAL, structWspace );
//	}
//	rk_adj_traj = ExportVariable( "rk_adj_traj", N*grid.getNumIntervals(), numStages*(NX+NXA), REAL, ACADO_VARIABLES );

	rk_hess_tmp1 = ExportVariable( "rk_hess1", NX+NU, NX+NU, REAL, structWspace );
	rk_hess_tmp2 = ExportVariable( "rk_hess2", NX+NU, NX+NU, REAL, structWspace );

    return SUCCESSFUL_RETURN;
}



// PROTECTED:


CLOSE_NAMESPACE_ACADO

// end of file.
