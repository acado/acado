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
 *    \file src/code_generation/integrators/erk_adjoint_export.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/integrators/erk_export.hpp>
#include <acado/code_generation/integrators/erk_adjoint_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

AdjointERKExport::AdjointERKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : ExplicitRungeKuttaExport( _userInteraction,_commonHeaderName )
{
}


AdjointERKExport::AdjointERKExport(	const AdjointERKExport& arg
									) : ExplicitRungeKuttaExport( arg )
{
}


AdjointERKExport::~AdjointERKExport( )
{
	clear( );
}



returnValue AdjointERKExport::setDifferentialEquation(	const Expression& rhs_ )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );

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

	DifferentialEquation f, f_ODE;
	// add usual ODE
	f_ODE << rhs_;
	if( f_ODE.getNDX() > 0 ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	if( (ExportSensitivityType)sensGen == BACKWARD ) {
		DifferentialState lx("", NX,1), lu("", NU,1);

		f << backwardDerivative(rhs_, x, lx);
		f << backwardDerivative(rhs_, u, lx);
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	if( f.getNT() > 0 ) timeDependant = true;

	return rhs.init(f_ODE, "rhs", NX, 0, NU, NP, NDX, NOD)
			& diffs_rhs.init(f, "rhs_back", 2*NX + NU, 0, NU, NP, NDX, NOD);
}


returnValue AdjointERKExport::setup( )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if ( (ExportSensitivityType)sensGen != BACKWARD ) ACADOERROR( RET_INVALID_OPTION );

	// NOT SUPPORTED: since the forward sweep needs to be saved
	if( !equidistantControlGrid() ) 	ACADOERROR( RET_INVALID_OPTION );

	// NOT SUPPORTED: since the adjoint derivatives could be 'arbitrarily bad'
	if( !is_symmetric ) 				ACADOERROR( RET_INVALID_OPTION );

	LOG( LVL_DEBUG ) << "Preparing to export AdjointERKExport... " << endl;

	// export RK scheme
	uint rhsDim   = 2*NX+NU;
	inputDim = 2*NX+NU + NU + NOD;
	const uint rkOrder  = getNumStages();

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();    

	ExportVariable Ah ( "A*h",  DMatrix( AA )*=h );
	ExportVariable b4h( "b4*h", DMatrix( bb )*=h );

	rk_index = ExportVariable( "rk_index", 1, 1, INT, ACADO_LOCAL, true );
	rk_eta = ExportVariable( "rk_eta", 1, inputDim );
//	seed_backward.setup( "seed", 1, NX );

	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	ExportStruct structWspace;
	structWspace = useOMP ? ACADO_LOCAL : ACADO_WORKSPACE;

	rk_ttt.setup( "rk_ttt", 1, 1, REAL, structWspace, true );
	uint timeDep = 0;
	if( timeDependant ) timeDep = 1;
	
	rk_xxx.setup("rk_xxx", 1, inputDim+timeDep, REAL, structWspace);
	rk_kkk.setup("rk_kkk", rkOrder, NX+NU, REAL, structWspace);
	rk_forward_sweep.setup("rk_sweep1", 1, grid.getNumIntervals()*rkOrder*NX, REAL, structWspace);

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

	// setup INTEGRATE function
	integrate = ExportFunction( "integrate", rk_eta, reset_int );
	integrate.setReturnValue( error_code );
	rk_eta.setDoc( "Working array to pass the input values and return the results." );
	reset_int.setDoc( "The internal memory of the integrator can be reset." );
	rk_index.setDoc( "Number of the shooting interval." );
	error_code.setDoc( "Status code of the integrator." );
	integrate.doc( "Performs the integration and sensitivity propagation for one shooting interval." );
	integrate.addIndex( run );
	
	integrate.addStatement( rk_ttt == DMatrix(grid.getFirstTime()) );

	if( inputDim > rhsDim ) {
//		integrate.addStatement( rk_eta.getCols( NX,2*NX ) == seed_backward );
		integrate.addStatement( rk_eta.getCols( 2*NX,2*NX+NU ) == zeros<double>( 1,NU ) );
		// FORWARD SWEEP FIRST
		integrate.addStatement( rk_xxx.getCols( NX,NX+NU+NOD ) == rk_eta.getCols( rhsDim,inputDim ) );
	}
	integrate.addLinebreak( );

    // integrator loop: FORWARD SWEEP
	ExportForLoop loop = ExportForLoop( run, 0, grid.getNumIntervals() );
	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		loop.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
		// save forward trajectory
		loop.addStatement( rk_forward_sweep.getCols( run*rkOrder*NX+run1*NX,run*rkOrder*NX+run1*NX+NX ) == rk_xxx.getCols( 0,NX ) );
		if( timeDependant ) loop.addStatement( rk_xxx.getCol( NX+NU+NOD ) == rk_ttt + ((double)cc(run1))/grid.getNumIntervals() );
		loop.addFunctionCall( getNameRHS(),rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop.addStatement( rk_eta.getCols( 0,NX ) += b4h^rk_kkk.getCols( 0,NX ) );
	loop.addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );
    // end of integrator loop: FORWARD SWEEP
	integrate.addStatement( loop );

//	if( !is_symmetric ) {
//		integrate.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) );
//	}
	if( inputDim > rhsDim ) {
		// BACKWARD SWEEP NEXT
		integrate.addStatement( rk_xxx.getCols( rhsDim,inputDim ) == rk_eta.getCols( rhsDim,inputDim ) );
	}
    // integrator loop: BACKWARD SWEEP
	ExportForLoop loop2 = ExportForLoop( run, 0, grid.getNumIntervals() );
	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		// load forward trajectory
//		if( is_symmetric ) {
			loop2.addStatement( rk_xxx.getCols( 0,NX ) == rk_forward_sweep.getCols( (grid.getNumIntervals()-run)*rkOrder*NX-run1*NX-NX,(grid.getNumIntervals()-run)*rkOrder*NX-run1*NX ) );
//		}
		loop2.addStatement( rk_xxx.getCols( NX,2*NX+NU ) == rk_eta.getCols( NX,2*NX+NU ) + Ah.getRow(run1)*rk_kkk );
		if( timeDependant ) loop2.addStatement( rk_xxx.getCol( inputDim ) == rk_ttt - ((double)cc(run1))/grid.getNumIntervals() );
		loop2.addFunctionCall( getNameDiffsRHS(),rk_xxx,rk_kkk.getAddress(run1,0) );
//		if( !is_symmetric ) {
//			loop2.addStatement( rk_xxx.getCols( 0,NX ) == rk_forward_sweep.getCols( (grid.getNumIntervals()-run)*rkOrder*NX-run1*NX-NX,(grid.getNumIntervals()-run)*rkOrder*NX-run1*NX ) );
//		}
	}
	loop2.addStatement( rk_eta.getCols( NX,2*NX+NU ) += b4h^rk_kkk );
	loop2.addStatement( rk_ttt -= DMatrix(1.0/grid.getNumIntervals()) );
    // end of integrator loop: BACKWARD SWEEP
	integrate.addStatement( loop2 );

	integrate.addStatement( error_code == 0 );

	LOG( LVL_DEBUG ) << "done" << endl;

	return SUCCESSFUL_RETURN;
}


returnValue AdjointERKExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	ExplicitRungeKuttaExport::getDataDeclarations( declarations, dataStruct );

	declarations.addDeclaration( rk_forward_sweep,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue AdjointERKExport::getCode(	ExportStatementBlock& code
										)
{
	int useOMP;
	get(CG_USE_OPENMP, useOMP);
	if ( useOMP )
	{
		getDataDeclarations( code, ACADO_LOCAL );

		code << "#pragma omp threadprivate( "
				<< getAuxVariable().getFullName()  << ", "
				<< rk_xxx.getFullName() << ", "
				<< rk_ttt.getFullName() << ", "
				<< rk_kkk.getFullName() << ", "
				<< rk_forward_sweep.getFullName()
				<< " )\n\n";
	}

	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if( exportRhs ) {
		code.addFunction( rhs );
		code.addFunction( diffs_rhs );
	}

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	code.addComment(std::string("Fixed step size:") + toString(h));
	code.addFunction( integrate );

	return SUCCESSFUL_RETURN;
}


// PROTECTED:



CLOSE_NAMESPACE_ACADO

// end of file.
