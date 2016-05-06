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
 *    \file src/code_generation/integrators/lifted_erk4_export.cpp
 *    \author Rien Quirynen
 *    \date 2015
 */

#include <acado/code_generation/integrators/erk_export.hpp>
#include <acado/code_generation/integrators/lifted_erk_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

LiftedERKExport::LiftedERKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ExplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
	solver = 0;
}


LiftedERKExport::LiftedERKExport(	const LiftedERKExport& arg
									) : ExplicitRungeKuttaExport( arg )
{
	solver = arg.solver;
}


LiftedERKExport::~LiftedERKExport( )
{
	if ( solver )
		delete solver;
	solver = 0;

	clear( );
}



returnValue LiftedERKExport::setDifferentialEquation(	const Expression& rhs_ )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );

	int liftMode = 1;
	// liftMode == 1 --> EXACT LIFTING
	// liftMode == 2 --> INEXACT LIFTING
	if( (ExportSensitivityType)sensGen == INEXACT ) liftMode = 2;

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

	x = DifferentialState("", NX, 1);
	dx = DifferentialStateDerivative("", NDX, 1);
	z = AlgebraicState("", NXA, 1);
	u = Control("", NU, 1);
	od = OnlineData("", NOD, 1);

	if( NDX > 0 && NDX != NX ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	if( rhs_.getNumRows() != (NX+NXA) ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	// collect the algebraic equations:
	Expression g_, rhs2_;
	if( NXA == 0 ) return ACADOERRORTEXT( RET_INVALID_OPTION, "Only DAE systems supported for the lifted ERK integrator!");
	else {
		g_ = rhs_.getRows(NX,NX+NXA);
		rhs2_ = rhs_.getRows(0,NX);
	}

	DifferentialEquation f, f_ODE, g;
	// add usual ODE
	f_ODE << rhs2_;
	if( f_ODE.getNDX() > 0 ) {
		return ACADOERRORTEXT( RET_INVALID_OPTION, "No implicit systems supported when using an explicit integration method!");
	}

	if( (ExportSensitivityType)sensGen == FORWARD || (ExportSensitivityType)sensGen == INEXACT ) {
		DifferentialState Gx("", NX,NX), Gu("", NX,NU);
		AlgebraicState deltaZ("", NXA,1), Zx("", NXA,NX), Zu("", NXA,NU);
		// no free parameters yet!
		// DifferentialState Gp(NX,NP);

		f << rhs2_ + multipleForwardDerivative( rhs2_, z, deltaZ );
		/*	if ( f.getDim() != f.getNX() )
			return ACADOERROR( RET_ILLFORMED_ODE );*/

		// add VDE for differential states
		f << multipleForwardDerivative( rhs2_, x, Gx ) + multipleForwardDerivative( rhs2_, z, Zx );
		/*	if ( f.getDim() != f.getNX() )
			return ACADOERROR( RET_ILLFORMED_ODE );*/


		// add VDE for control inputs
		f << multipleForwardDerivative( rhs2_, x, Gu ) + multipleForwardDerivative( rhs2_, z, Zu ) + forwardDerivative( rhs2_, u );
		// 	if ( f.getDim() != f.getNX() )
		// 		return ACADOERROR( RET_ILLFORMED_ODE );

		// no free parameters yet!
		// f << forwardDerivative( rhs_, x ) * Gp + forwardDerivative( rhs_, p );

		g << g_;
		if( liftMode == 1 ) {
			g << multipleForwardDerivative( g_, x, Gx );
		}
		else {
			g << multipleForwardDerivative( g_, x, Gx ) + multipleForwardDerivative( g_, z, Zx );
		}
		g << forwardDerivative( g_, z );
		if( liftMode == 1 ) {
			g << forwardDerivative( g_, u ) + multipleForwardDerivative( g_, x, Gu );
		}
		else {
			g << forwardDerivative( g_, u ) + multipleForwardDerivative( g_, x, Gu ) + multipleForwardDerivative( g_, z, Zu );
		}
	}
	else {
		return ACADOERRORTEXT( RET_NOT_YET_IMPLEMENTED, "The lifted ERK integrator is currently only implemented with forward sensitivity propagation!" );
	}
	if( f.getNT() > 0 ) timeDependant = true;

	return diffs_rhs.init(f, "acado_rhs_ext", NX * (1 + NX + NU), NXA * (2 + NX + NU), NU, NP, NDX, NOD)
				& alg_rhs.init(g, "acado_alg_rhs", NX * (1 + NX + NU), NXA * (2 + NX + NU), NU, NP, NDX, NOD);

	return SUCCESSFUL_RETURN;
}


returnValue LiftedERKExport::setup( )
{
	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if ( (ExportSensitivityType)sensGen != FORWARD && (ExportSensitivityType)sensGen != NO_SENSITIVITY && (ExportSensitivityType)sensGen != INEXACT ) ACADOERROR( RET_INVALID_OPTION );

	int liftMode = 1;
	// liftMode == 1 --> EXACT LIFTING
	// liftMode == 2 --> INEXACT LIFTING
	if( (ExportSensitivityType)sensGen == INEXACT ) liftMode = 2;

	bool DERIVATIVES = ((ExportSensitivityType)sensGen != NO_SENSITIVITY);

	LOG( LVL_DEBUG ) << "Preparing to export ExplicitRungeKuttaExport... " << endl;

	// setup linear solver:
	int solverType;
	userInteraction->get( LINEAR_ALGEBRA_SOLVER,solverType );

	if ( solver )
		delete solver;
	solver = 0;

	switch( (LinearAlgebraSolver) solverType ) {
	case GAUSS_LU:
		solver = new ExportGaussElim( userInteraction,commonHeaderName );
		solver->init( NXA, NX+NU+1 );
		solver->setReuse( true ); 	// IFTR method
		solver->setup();
		rk_auxSolver = solver->getGlobalExportVariable( 1 );
		break;
	default:
		return ACADOERROR( RET_INVALID_OPTION );
	}
	rk_A = ExportVariable( "rk_A", NXA, NXA, REAL, structWspace );
	rk_b = ExportVariable( "rk_b", NXA, 1+NX+NU, REAL, structWspace );

	// export RK scheme
	uint rhsDim   = NX*(NX+NU+1);
	uint zDim = NXA*(NX+NU+1);
	inputDim = (NX+NXA)*(NX+NU+1) + NU + NOD;
	if( !DERIVATIVES ) return ACADOERROR( RET_INVALID_OPTION );
	const uint rkOrder  = getNumStages();

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();

	ExportVariable Ah ( "A*h",  DMatrix( AA )*=h );
	ExportVariable b4h( "b4*h", DMatrix( bb )*=h );

	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, true );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim );

	rk_ttt.setup( "rk_ttt", 1, 1, REAL, structWspace, true );
	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;

	rk_xxx.setup("rk_xxx", 1, inputDim+NXA+timeDep, REAL, structWspace);
	rk_kkk.setup("rk_kkk", rkOrder, rhsDim, REAL, structWspace);

	rk_zzz.setup("rk_zzz", getN()*grid.getNumIntervals()*rkOrder, NXA, REAL, structWspace);
	rk_zTemp.setup("rk_zTemp", 1, NXA*(1+NX+NXA+NU), REAL, structWspace);
//	if( liftMode == 1 ) {
		rk_diffZ.setup("rk_diffZ", getN()*grid.getNumIntervals()*rkOrder*NXA, NX+NU, REAL, structWspace);
		rk_delta.setup( "rk_delta", 1, NX+NU, REAL, ACADO_WORKSPACE );
		rk_prev.setup( "rk_prev", getN(), NX+NU, REAL, ACADO_WORKSPACE );
//	}

	if ( useOMP )
	{
		ExportVariable auxVar;

		auxVar = getAuxVariable();
		auxVar.setName( "odeAuxVar" );
		auxVar.setDataStruct( ACADO_LOCAL );
		rhs.setGlobalExportVariable( auxVar );
		diffs_rhs.setGlobalExportVariable( auxVar );
	}

	ExportIndex run( "run1" );
	ExportIndex shoot_index( "shoot_index" );
	ExportIndex k_index( "k_index" );

	// setup INTEGRATE function
	integrate = ExportFunction( "integrate", rk_eta, rk_index ); // you need the rk_index for LIFTING !
	integrate.setReturnValue( error_code );
	rk_eta.setDoc( "Working array to pass the input values and return the results." );
	reset_int.setDoc( "The internal memory of the integrator can be reset." );
	rk_index.setDoc( "Number of the shooting interval." );
	error_code.setDoc( "Status code of the integrator." );
	integrate.doc( "Performs the integration and sensitivity propagation for one shooting interval." );
	integrate.addIndex( run );
	integrate.addIndex( shoot_index );
	integrate.addIndex( k_index );

	ExportVariable det( "det", 1, 1, REAL, ACADO_LOCAL, true );
	integrate.addDeclaration( det );

	integrate << shoot_index.getFullName() << " = " << rk_index.getFullName() << ";\n";

	ExportVariable numInt( "numInts", 1, 1, INT );
	if( !equidistantControlGrid() ) {
		integrate.addStatement( std::string( "int numSteps[" ) + toString( numSteps.getDim() ) + "] = {" + toString( numSteps(0) ) );
		uint i;
		for( i = 1; i < numSteps.getDim(); i++ ) {
			integrate.addStatement( std::string( ", " ) + toString( numSteps(i) ) );
		}
		integrate.addStatement( std::string( "};\n" ) );
		integrate.addStatement( std::string( "int " ) + numInt.getName() + " = numSteps[" + rk_index.getName() + "];\n" );
	}

	integrate.addStatement( rk_ttt == DMatrix(grid.getFirstTime()) );

	if( DERIVATIVES ) {
		// initialize sensitivities:
		DMatrix idX    = eye<double>( NX );
		DMatrix zeroXU = zeros<double>( NX,NU );
		integrate.addStatement( rk_eta.getCols( NX+NXA,NXA+NX*(1+NX) ) == idX.makeVector().transpose() );
		integrate.addStatement( rk_eta.getCols( (NX+NXA)*(1+NX),(NX+NXA)*(1+NX)+NX*NU ) == zeroXU.makeVector().transpose() );
	}

	if( inputDim > (rhsDim+zDim) ) {
		integrate.addStatement( rk_xxx.getCols( NXA+rhsDim+zDim,NXA+inputDim ) == rk_eta.getCols( rhsDim+zDim,inputDim ) );
	}
//	if( liftMode == 1 ) {
		integrate.addStatement( rk_delta.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) - rk_prev.getSubMatrix(shoot_index,shoot_index+1,0,NX) );
		integrate.addStatement( rk_delta.getCols( NX,NX+NU ) == rk_eta.getCols( rhsDim+zDim,rhsDim+zDim+NU ) - rk_prev.getSubMatrix(shoot_index,shoot_index+1,NX,NX+NU) );

		integrate.addStatement( rk_prev.getSubMatrix(shoot_index,shoot_index+1,0,NX) == rk_eta.getCols( 0,NX ) );
		integrate.addStatement( rk_prev.getSubMatrix(shoot_index,shoot_index+1,NX,NX+NU) == rk_eta.getCols( rhsDim+zDim,rhsDim+zDim+NU ) );
//	}
	integrate.addLinebreak( );

	// integrator loop
	ExportForLoop loop;
	if( equidistantControlGrid() ) {
		loop = ExportForLoop( run, 0, grid.getNumIntervals() );
	}
	else {
		loop = ExportForLoop( run, 0, 1 );
		loop.addStatement( std::string("for(") + run.getName() + " = 0; " + run.getName() + " < " + numInt.getName() + "; " + run.getName() + "++ ) {\n" );
	}
	loop.addStatement( k_index == (shoot_index*grid.getNumIntervals()+run) );

	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		loop.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
		loop.addStatement( rk_xxx.getCols( NX,NX*(1+NX) ) == rk_eta.getCols( NX+NXA,NXA+NX*(1+NX) ) + Ah.getRow(run1)*rk_kkk.getCols( NX,NX*(1+NX) ) );
		loop.addStatement( rk_xxx.getCols( NX*(1+NX),rhsDim ) == rk_eta.getCols( (NX+NXA)*(1+NX),(NX+NXA)*(1+NX)+NX*NU ) + Ah.getRow(run1)*rk_kkk.getCols( NX*(1+NX),rhsDim ) );
		if( timeDependant ) loop.addStatement( rk_xxx.getCol( NXA+inputDim ) == rk_ttt + ((double)cc(run1))/grid.getNumIntervals() );

		// update algebraic variables based on previous SQP step:
//		if( liftMode == 1 ) {
			loop.addStatement( rk_zzz.getRow(k_index*rkOrder+run1) += rk_delta*rk_diffZ.getRows( k_index*rkOrder*NXA+run1*NXA,k_index*rkOrder*NXA+run1*NXA+NXA ).getTranspose() );
//		}

		// evaluate the right algebraic variables and equations:
		loop.addStatement( rk_xxx.getCols( rhsDim,rhsDim+NXA ) == rk_zzz.getRow(k_index*rkOrder+run1) );
		if( liftMode == 2 ) {
			for( uint i = 0; i < NXA; i++ ) {
				for( uint j = 0; j < NX; j++ ) {
					loop.addStatement( rk_xxx.getCol( rhsDim+2*NXA+i*NX+j ) == rk_diffZ.getElement( k_index*rkOrder*NXA+run1*NXA+i,j ) );
				}
				for( uint j = 0; j < NU; j++ ) {
					loop.addStatement( rk_xxx.getCol( rhsDim+NXA*(2+NX)+i*NU+j ) == rk_diffZ.getElement( k_index*rkOrder*NXA+run1*NXA+i,NX+j ) );
				}
			}
		}
		loop.addFunctionCall( alg_rhs.getName(),rk_xxx,rk_zTemp );

		// JACOBIAN FACTORIZATION ALGEBRAIC EQUATIONS
		if( liftMode == 1 ) { // EXACT
			for( uint i = 0; i < NXA; i++ ) {
				for( uint j = 0; j < NXA; j++ ) {
					loop.addStatement( rk_A.getElement(i,j) == rk_zTemp.getCol(NXA*(1+NX)+i*NXA+j) );
				}
			}
			loop.addStatement( det.getFullName() + " = " + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
		}
		else { // INEXACT
			if( run1 == 0 ) {
				loop.addStatement( std::string("if(") + run.getName() + " == 0) {\n" );
				for( uint i = 0; i < NXA; i++ ) {
					for( uint j = 0; j < NXA; j++ ) {
						loop.addStatement( rk_A.getElement(i,j) == rk_zTemp.getCol(NXA*(1+NX)+i*NXA+j) );
					}
				}
				loop.addStatement( det.getFullName() + " = " + solver->getNameSolveFunction() + "( " + rk_A.getFullName() + ", " + rk_auxSolver.getFullName() + " );\n" );
				loop.addStatement( std::string("}\n") );
			}
		}
		for( uint i = 0; i < NXA; i++ ) {
			loop.addStatement( rk_b.getElement(i,0) == 0.0 - rk_zTemp.getCol(i) );
			for( uint j = 0; j < NX; j++ ) {
				loop.addStatement( rk_b.getElement(i,1+j) == 0.0 - rk_zTemp.getCol(NXA+i*NX+j) );
			}
			for( uint j = 0; j < NU; j++ ) {
				loop.addStatement( rk_b.getElement(i,1+NX+j) == 0.0 - rk_zTemp.getCol(NXA*(1+NX+NXA)+i*NU+j) );
			}
		}

		// NEWTON STEP ON THE ALGEBRAIC EQUATIONS
		loop.addFunctionCall( solver->getNameSolveReuseFunction(),rk_A.getAddress(0,0),rk_b.getAddress(0,0),rk_auxSolver.getAddress(0,0) );
		loop.addStatement( rk_zzz.getRow(k_index*rkOrder+run1) += rk_b.getCol( 0 ).getTranspose() );
		loop.addStatement( rk_xxx.getCols( rhsDim+NXA,rhsDim+2*NXA ) == rk_b.getCol( 0 ).getTranspose() );
		for( uint i = 0; i < NXA; i++ ) {
			for( uint j = 0; j < NX; j++ ) {
				if( liftMode == 1 ) loop.addStatement( rk_diffZ.getElement( k_index*rkOrder*NXA+run1*NXA+i,j ) == rk_b.getElement(i,1+j) );
				else				loop.addStatement( rk_diffZ.getElement( k_index*rkOrder*NXA+run1*NXA+i,j ) += rk_b.getElement(i,1+j) );
				loop.addStatement( rk_xxx.getCol( rhsDim+2*NXA+i*NX+j ) == rk_diffZ.getElement( k_index*rkOrder*NXA+run1*NXA+i,j ) );
			}
			for( uint j = 0; j < NU; j++ ) {
				if( liftMode == 1 ) loop.addStatement( rk_diffZ.getElement( k_index*rkOrder*NXA+run1*NXA+i,NX+j ) == rk_b.getElement(i,1+NX+j) );
				else  				loop.addStatement( rk_diffZ.getElement( k_index*rkOrder*NXA+run1*NXA+i,NX+j ) += rk_b.getElement(i,1+NX+j) );
				loop.addStatement( rk_xxx.getCol( rhsDim+NXA*(2+NX)+i*NU+j ) == rk_diffZ.getElement( k_index*rkOrder*NXA+run1*NXA+i,NX+j ) );
			}
		}

		loop.addFunctionCall( getNameDiffsRHS(),rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop.addStatement( rk_eta.getCols( 0,NX ) += b4h^rk_kkk.getCols(0,NX) );
	loop.addStatement( rk_eta.getCols( NX+NXA,NXA+NX*(1+NX) ) += b4h^rk_kkk.getCols(NX,NX*(1+NX)) );
	loop.addStatement( rk_eta.getCols( (NX+NXA)*(1+NX),(NX+NXA)*(1+NX)+NX*NU ) += b4h^rk_kkk.getCols(NX*(1+NX),rhsDim) );
	loop.addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );
	// end of integrator loop

	if( !equidistantControlGrid() ) {
		loop.addStatement( "}\n" );
		//		loop.unrollLoop();
	}
	integrate.addStatement( loop );

	integrate.addStatement( error_code == 0 );

	LOG( LVL_DEBUG ) << "done" << endl;

	return SUCCESSFUL_RETURN;
}


returnValue LiftedERKExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	ExplicitRungeKuttaExport::getDataDeclarations( declarations, dataStruct );

	solver->getDataDeclarations( declarations,dataStruct );

	declarations.addDeclaration( rk_A,dataStruct );
	declarations.addDeclaration( rk_b,dataStruct );
	declarations.addDeclaration( rk_auxSolver,dataStruct );

	declarations.addDeclaration( rk_zzz,dataStruct );
	declarations.addDeclaration( rk_zTemp,dataStruct );

	declarations.addDeclaration( rk_diffZ,dataStruct );
	declarations.addDeclaration( rk_delta,dataStruct );
	declarations.addDeclaration( rk_prev,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue LiftedERKExport::getCode(	ExportStatementBlock& code
										)
{
	// 	int printLevel;
	// 	get( PRINTLEVEL,printLevel );
	//
	// 	if ( (PrintLevel)printLevel >= HIGH )
	// 		acadoPrintf( "--> Exporting %s... ",fileName.getName() );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP )
	{
		getDataDeclarations( code, ACADO_LOCAL );

		code << "#pragma omp threadprivate( "
				<< getAuxVariable().getFullName()  << ", "
				<< rk_xxx.getFullName() << ", "
				<< rk_ttt.getFullName() << ", "
				<< rk_kkk.getFullName()
				<< " )\n\n";
	}

	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if( exportRhs ) {
		code.addFunction( rhs );
		code.addFunction( diffs_rhs );
		code.addFunction( alg_rhs );
		solver->getCode( code ); // solver for the algebraic equations
	}
	else {
		return ACADOERRORTEXT( RET_NOT_YET_IMPLEMENTED, "Externally defined dynamics not yet supported for the lifted ERK integrator!");
	}

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	code.addComment(std::string("Fixed step size:") + toString(h));
	code.addFunction( integrate );


	// 	if ( (PrintLevel)printLevel >= HIGH )
	// 		acadoPrintf( "done.\n" );

	return SUCCESSFUL_RETURN;
}


ExportVariable LiftedERKExport::getAuxVariable() const
{
	ExportVariable max;
	max = rhs.getGlobalExportVariable();
	if( diffs_rhs.getGlobalExportVariable().getDim() > max.getDim() ) {
		max = diffs_rhs.getGlobalExportVariable();
	}
	if( alg_rhs.getGlobalExportVariable().getDim() > max.getDim() ) {
		max = alg_rhs.getGlobalExportVariable();
	}
	return max;
}


// PROTECTED:



CLOSE_NAMESPACE_ACADO

// end of file.
