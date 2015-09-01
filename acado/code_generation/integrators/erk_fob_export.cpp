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
 *    \file src/code_generation/integrators/erk_fob_export.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/integrators/erk_export.hpp>
#include <acado/code_generation/integrators/erk_fob_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ForwardOverBackwardERKExport::ForwardOverBackwardERKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : AdjointERKExport( _userInteraction,_commonHeaderName )
{
}


ForwardOverBackwardERKExport::ForwardOverBackwardERKExport(	const ForwardOverBackwardERKExport& arg
									) : AdjointERKExport( arg )
{
}


ForwardOverBackwardERKExport::~ForwardOverBackwardERKExport( )
{
	clear( );
}



returnValue ForwardOverBackwardERKExport::setDifferentialEquation(	const Expression& rhs_ )
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

	DifferentialEquation f, g, f_ODE;
	// add usual ODE
	f_ODE << rhs_;
	if( f_ODE.getNDX() > 0 ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	if( (ExportSensitivityType)sensGen == FORWARD_OVER_BACKWARD ) {
		DifferentialState Gx("", NX,NX), Gu("", NX,NU);
		// no free parameters yet!
		// DifferentialState Gp(NX,NP);

		f << rhs_;
		/*	if ( f.getDim() != f.getNX() )
		return ACADOERROR( RET_ILLFORMED_ODE );*/

		Expression arg;
		arg << x;
		arg << u;
		Expression S_tmp = Gx;
		S_tmp.appendCols( Gu );
		S_tmp.appendRows(zeros<double>(NU,NX).appendCols(eye<double>(NU)));
		Expression dfS = multipleForwardDerivative( rhs_, arg, S_tmp );
		// add VDE for differential states
		f << dfS.getCols(0,NX);
//		f << multipleForwardDerivative( rhs_, x, Gx );
		/*	if ( f.getDim() != f.getNX() )
		return ACADOERROR( RET_ILLFORMED_ODE );*/

		// add VDE for control inputs
		f << dfS.getCols(NX,NX+NU);
//		f << multipleForwardDerivative( rhs_, x, Gu ) + forwardDerivative( rhs_, u );
		// 	if ( f.getDim() != f.getNX() )
		// 		return ACADOERROR( RET_ILLFORMED_ODE );

		// no free parameters yet!
		// f << forwardDerivative( rhs_, x ) * Gp + forwardDerivative( rhs_, p );


		DifferentialState lx("", NX,1);

		Expression tmp = backwardDerivative( rhs_, arg, lx );

		g << tmp.getRows(0,NX);

		Expression tmp2 = multipleForwardDerivative( tmp, arg, S_tmp );

		DifferentialState Sxx("", NX,NX), Sux("", NU,NX), Suu("", NU,NU);
		Expression SS_tmp = Sxx;
		SS_tmp.appendCols( Sux.transpose() );
		Expression tmp3 = multipleBackwardDerivative( rhs_, arg, SS_tmp );

		g << tmp2.getSubMatrix(0,NX,0,NX) + tmp3.getSubMatrix(0,NX,0,NX);
		g << tmp2.getSubMatrix(0,NX,NX,NX+NU).transpose() + tmp3.getSubMatrix(0,NX,NX,NX+NU).transpose();
		g << tmp2.getSubMatrix(NX,NX+NU,NX,NX+NU) + tmp3.getSubMatrix(NX,NX+NU,NX,NX+NU);

//		g << multipleForwardDerivative(tmp, x, Gx) + multipleBackwardDerivative(rhs_, x, Sxx);
//		g << multipleBackwardDerivative(tmp, x, Gu).transpose() + forwardDerivative(tmp, u).transpose() + multipleBackwardDerivative(rhs_, x, Sux.transpose()).transpose();
//		g << forwardDerivative(backwardDerivative(rhs_, u, lx), u) + multipleBackwardDerivative(tmp, u, Gu) + multipleBackwardDerivative(rhs_, u, Sux.transpose());
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	if( f.getNT() > 0 ) timeDependant = true;

	return rhs.init(f, "acado_forward", NX*(NX+NU+1), 0, NU, NP, NDX, NOD)
				& diffs_rhs.init(g, "acado_backward", NX*(NX+NU+1) + NX + NX*NX + NX*NU + NU*NU, 0, NU, NP, NDX, NOD);
}


returnValue ForwardOverBackwardERKExport::setup( )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if ( (ExportSensitivityType)sensGen != FORWARD_OVER_BACKWARD ) ACADOERROR( RET_INVALID_OPTION );

	// NOT SUPPORTED: since the forward sweep needs to be saved
	if( !equidistantControlGrid() ) 	ACADOERROR( RET_INVALID_OPTION );

	// NOT SUPPORTED: since the adjoint derivatives could be 'arbitrarily bad'
	if( !is_symmetric ) 				ACADOERROR( RET_INVALID_OPTION );

	LOG( LVL_DEBUG ) << "Preparing to export ForwardOverBackwardERKExport... " << endl;

	// export RK scheme
	uint rhsDim   = NX*(NX+NU+1) + NX + NX*NX + NX*NU + NU*NU;
	inputDim = rhsDim + NU + NOD;
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
	rk_kkk.setup("rk_kkk", rkOrder, NX+NX*NX+NX*NU+NU*NU, REAL, structWspace);
	rk_forward_sweep.setup("rk_sweep1", 1, grid.getNumIntervals()*rkOrder*NX*(NX+NU+1), REAL, structWspace);

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


	// initialize sensitivities:
	DMatrix idX    = eye<double>( NX );
	DMatrix zeroXU = zeros<double>( NX,NU );
	integrate.addStatement( rk_eta.getCols( 2*NX,NX*(2+NX) ) == idX.makeVector().transpose() );
	integrate.addStatement( rk_eta.getCols( NX*(2+NX),NX*(2+NX+NU) ) == zeroXU.makeVector().transpose() );

//		integrate.addStatement( rk_eta.getCols( NX*(1+NX+NU),NX*(2+NX+NU) ) == seed_backward );
	integrate.addStatement( rk_eta.getCols( NX*(2+NX+NU),rhsDim ) == zeros<double>( 1,NX*NX+NX*NU+NU*NU ) );
	if( inputDim > rhsDim ) {
		// FORWARD SWEEP FIRST
		integrate.addStatement( rk_xxx.getCols( NX*(1+NX+NU),NX*(1+NX+NU)+NU+NOD ) == rk_eta.getCols( rhsDim,inputDim ) );
	}
	integrate.addLinebreak( );

    // integrator loop: FORWARD SWEEP
	ExportForLoop loop = ExportForLoop( run, 0, grid.getNumIntervals() );
	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		loop.addStatement( rk_xxx.getCols( 0,NX ) == rk_eta.getCols( 0,NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
		loop.addStatement( rk_xxx.getCols( NX,NX*(1+NX+NU) ) == rk_eta.getCols( 2*NX,NX*(2+NX+NU) ) + Ah.getRow(run1)*rk_kkk.getCols( NX,NX*(1+NX+NU) ) );
		// save forward trajectory
		loop.addStatement( rk_forward_sweep.getCols( run*rkOrder*NX*(1+NX+NU)+run1*NX*(1+NX+NU),run*rkOrder*NX*(1+NX+NU)+(run1+1)*NX*(1+NX+NU) ) == rk_xxx.getCols( 0,NX*(1+NX+NU) ) );
		if( timeDependant ) loop.addStatement( rk_xxx.getCol( NX*(NX+NU+1)+NU+NOD ) == rk_ttt + ((double)cc(run1))/grid.getNumIntervals() );
		loop.addFunctionCall( getNameRHS(),rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop.addStatement( rk_eta.getCols( 0,NX ) += b4h^rk_kkk.getCols( 0,NX ) );
	loop.addStatement( rk_eta.getCols( NX*2,NX*(2+NX+NU) ) += b4h^rk_kkk.getCols( NX,NX*(1+NX+NU) ) );
	loop.addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );
    // end of integrator loop: FORWARD SWEEP
	integrate.addStatement( loop );

	if( inputDim > rhsDim ) {
		// BACKWARD SWEEP NEXT
		integrate.addStatement( rk_xxx.getCols( rhsDim,inputDim ) == rk_eta.getCols( rhsDim,inputDim ) );
	}
    // integrator loop: BACKWARD SWEEP
	ExportForLoop loop2 = ExportForLoop( run, 0, grid.getNumIntervals() );
	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		// load forward trajectory
		loop2.addStatement( rk_xxx.getCols( 0,NX*(1+NX+NU) ) == rk_forward_sweep.getCols( (grid.getNumIntervals()-run)*rkOrder*NX*(1+NX+NU)-(run1+1)*NX*(1+NX+NU),(grid.getNumIntervals()-run)*rkOrder*NX*(1+NX+NU)-run1*NX*(1+NX+NU) ) );
		loop2.addStatement( rk_xxx.getCols( NX*(1+NX+NU),NX*(2+NX+NU) ) == rk_eta.getCols( NX,NX*2 ) + Ah.getRow(run1)*rk_kkk.getCols(0,NX) );
		loop2.addStatement( rk_xxx.getCols( NX*(2+NX+NU),rhsDim ) == rk_eta.getCols( NX*(2+NX+NU),rhsDim ) + Ah.getRow(run1)*rk_kkk.getCols(NX,rk_kkk.getNumCols()) );
		if( timeDependant ) loop2.addStatement( rk_xxx.getCol( inputDim ) == rk_ttt - ((double)cc(run1))/grid.getNumIntervals() );
		loop2.addFunctionCall( getNameDiffsRHS(),rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop2.addStatement( rk_eta.getCols( NX,2*NX ) += b4h^rk_kkk.getCols(0,NX) );
	loop2.addStatement( rk_eta.getCols( NX*(2+NX+NU),rhsDim ) += b4h^rk_kkk.getCols(NX,rk_kkk.getNumCols()) );
	loop2.addStatement( rk_ttt -= DMatrix(1.0/grid.getNumIntervals()) );
    // end of integrator loop: BACKWARD SWEEP
	integrate.addStatement( loop2 );

	integrate.addStatement( error_code == 0 );

	LOG( LVL_DEBUG ) << "done" << endl;

	return SUCCESSFUL_RETURN;
}


// PROTECTED:



CLOSE_NAMESPACE_ACADO

// end of file.
