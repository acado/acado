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
 *    \file src/code_generation/integrators/erk_3sweep_export.cpp
 *    \author Rien Quirynen
 *    \date 2014
 */

#include <acado/code_generation/integrators/erk_export.hpp>
#include <acado/code_generation/integrators/erk_3sweep_export.hpp>

using namespace std;

BEGIN_NAMESPACE_ACADO

//
// PUBLIC MEMBER FUNCTIONS:
//

ThreeSweepsERKExport::ThreeSweepsERKExport(	UserInteraction* _userInteraction,
									const std::string& _commonHeaderName
									) : AdjointERKExport( _userInteraction,_commonHeaderName )
{
}


ThreeSweepsERKExport::ThreeSweepsERKExport(	const ThreeSweepsERKExport& arg
									) : AdjointERKExport( arg )
{
}


ThreeSweepsERKExport::~ThreeSweepsERKExport( )
{
	clear( );
}



returnValue ThreeSweepsERKExport::setDifferentialEquation(	const Expression& rhs_ )
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

	DifferentialEquation f, g, h, f_ODE;
	// add usual ODE
	f_ODE << rhs_;
	if( f_ODE.getNDX() > 0 ) {
		return ACADOERROR( RET_INVALID_OPTION );
	}

	uint numX = NX*(NX+1)/2.0;
	uint numU = NU*(NU+1)/2.0;
	uint numZ = (NX+NU)*(NX+NU+1)/2.0;
	if( (ExportSensitivityType)sensGen == SYMMETRIC ) {
		// SWEEP 1:
		// ---------
		f << rhs_;


		// SWEEP 2:
		// ---------
		DifferentialState lx("", NX,1);

		Expression tmp = backwardDerivative(rhs_, x, lx);
		g << tmp;


		// SWEEP 3:
		// ---------
		DifferentialState Gx("", NX,NX), Gu("", NX,NU);
		DifferentialState H("", numZ,1);

		Expression S = Gx;
		S.appendCols(Gu);
		Expression arg;
		arg << x;
		arg << u;

		// SYMMETRIC DERIVATIVES
		Expression S_tmp = S;
		S_tmp.appendRows(zeros<double>(NU,NX).appendCols(eye<double>(NU)));

		Expression dfS;
		Expression h_tmp = symmetricDerivative( rhs_, arg, S_tmp, lx, &dfS );
		h << dfS.getCols(0,NX);
		h << dfS.getCols(NX,NX+NU);
		h << returnLowerTriangular( h_tmp );

		// OLD VERSION:
//		// add VDE for differential states
//		h << multipleForwardDerivative( rhs_, x, Gx );
//
//		// add VDE for control inputs
//		h << multipleForwardDerivative( rhs_, x, Gu ) + forwardDerivative( rhs_, u );
//
//		IntermediateState tmp2 = forwardDerivative(tmp, x);
//		Expression tmp3 = backwardDerivative(rhs_, u, lx);
//		Expression tmp4 = multipleForwardDerivative(tmp3, x, Gu);
//
//		// TODO: include a symmetric_AD_operator to strongly improve the symmetric left-right multiplied second order derivative computations !!
////		Expression tmp6 = Gx.transpose()*tmp2*Gx;
//		h << symmetricDoubleProduct(tmp2, Gx);
//		h << Gu.transpose()*tmp2*Gx + multipleForwardDerivative(tmp3, x, Gx);
//		Expression tmp7 = tmp4 + tmp4.transpose() + forwardDerivative(tmp3, u);
//		h << symmetricDoubleProduct(tmp2, Gu) + returnLowerTriangular(tmp7);
	}
	else {
		return ACADOERROR( RET_INVALID_OPTION );
	}
	if( f.getNT() > 0 ) timeDependant = true;

	return rhs.init(f, "acado_forward", NX, 0, NU, NP, NDX, NOD)
			& diffs_rhs.init(g, "acado_backward", 2*NX, 0, NU, NP, NDX, NOD)
			& diffs_sweep3.init(h, "acado_forward_sweep3", 2*NX + NX*(NX+NU) + numX + NX*NU + numU, 0, NU, NP, NDX, NOD);
}


returnValue ThreeSweepsERKExport::setup( )
{
	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if ( (ExportSensitivityType)sensGen != SYMMETRIC ) ACADOERROR( RET_INVALID_OPTION );

	// NOT SUPPORTED: since the forward sweep needs to be saved
	if( !equidistantControlGrid() ) 	ACADOERROR( RET_INVALID_OPTION );

	// NOT SUPPORTED: since the adjoint derivatives could be 'arbitrarily bad'
	if( !is_symmetric ) 				ACADOERROR( RET_INVALID_OPTION );

	LOG( LVL_DEBUG ) << "Preparing to export ThreeSweepsERKExport... " << endl;

	// export RK scheme
	uint numX = NX*(NX+1)/2.0;
	uint numU = NU*(NU+1)/2.0;
	uint rhsDim   = NX + NX + NX*(NX+NU) + numX + NX*NU + numU;
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
	uint numK = NX*(NX+NU)+numX+NX*NU+numU;
	rk_kkk.setup("rk_kkk", rkOrder, numK, REAL, structWspace);
	rk_forward_sweep.setup("rk_sweep1", 1, grid.getNumIntervals()*rkOrder*NX, REAL, structWspace);
	rk_backward_sweep.setup("rk_sweep2", 1, grid.getNumIntervals()*rkOrder*NX, REAL, structWspace);

	if ( useOMP )
	{
		ExportVariable auxVar;

		auxVar = diffs_rhs.getGlobalExportVariable();
		auxVar.setName( "odeAuxVar" );
		auxVar.setDataStruct( ACADO_LOCAL );
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
		// initialize sensitivities:
//		integrate.addStatement( rk_eta.getCols( NX,2*NX ) == seed_backward );
		DMatrix idX    = eye<double>( NX );
		DMatrix zeroXU = zeros<double>( NX,NU );
		integrate.addStatement( rk_eta.getCols( 2*NX,NX*(2+NX) ) == idX.makeVector().transpose() );
		integrate.addStatement( rk_eta.getCols( NX*(2+NX),NX*(2+NX+NU) ) == zeroXU.makeVector().transpose() );

		integrate.addStatement( rk_eta.getCols( NX*(2+NX+NU),rhsDim ) == zeros<double>( 1,numX+NX*NU+numU ) );
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
		loop.addStatement( rk_forward_sweep.getCols( run*rkOrder*NX+run1*NX,run*rkOrder*NX+(run1+1)*NX ) == rk_xxx.getCols( 0,NX ) );
		if( timeDependant ) loop.addStatement( rk_xxx.getCol( NX+NU+NOD ) == rk_ttt + ((double)cc(run1))/grid.getNumIntervals() );
		loop.addFunctionCall( getNameRHS(),rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop.addStatement( rk_eta.getCols( 0,NX ) += b4h^rk_kkk.getCols( 0,NX ) );
	loop.addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );
    // end of integrator loop: FORWARD SWEEP
	integrate.addStatement( loop );

	if( inputDim > rhsDim ) {
		// BACKWARD SWEEP NEXT
		integrate.addStatement( rk_xxx.getCols( 2*NX,2*NX+NU+NOD ) == rk_eta.getCols( rhsDim,inputDim ) );
	}
    // integrator loop: BACKWARD SWEEP
	ExportForLoop loop2 = ExportForLoop( run, 0, grid.getNumIntervals() );
	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		// load forward trajectory
		loop2.addStatement( rk_xxx.getCols( 0,NX ) == rk_forward_sweep.getCols( (grid.getNumIntervals()-run)*rkOrder*NX-(run1+1)*NX,(grid.getNumIntervals()-run)*rkOrder*NX-run1*NX ) );
		loop2.addStatement( rk_xxx.getCols( NX,2*NX ) == rk_eta.getCols( NX,2*NX ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX ) );
		// save backward trajectory
		loop2.addStatement( rk_backward_sweep.getCols( run*rkOrder*NX+run1*NX,run*rkOrder*NX+(run1+1)*NX ) == rk_xxx.getCols( NX,2*NX ) );
		if( timeDependant ) loop2.addStatement( rk_xxx.getCol( 2*NX+NU+NOD ) == rk_ttt - ((double)cc(run1))/grid.getNumIntervals() );
		loop2.addFunctionCall( getNameDiffsRHS(),rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop2.addStatement( rk_eta.getCols( NX,2*NX ) += b4h^rk_kkk.getCols( 0,NX ) );
	loop2.addStatement( rk_ttt -= DMatrix(1.0/grid.getNumIntervals()) );
    // end of integrator loop: BACKWARD SWEEP
	integrate.addStatement( loop2 );

	if( inputDim > rhsDim ) {
		// THIRD SWEEP NEXT
		integrate.addStatement( rk_xxx.getCols( rhsDim,inputDim ) == rk_eta.getCols( rhsDim,inputDim ) );
	}
    // integrator loop: THIRD SWEEP
	ExportForLoop loop3 = ExportForLoop( run, 0, grid.getNumIntervals() );
	for( uint run1 = 0; run1 < rkOrder; run1++ )
	{
		// load forward trajectory
		loop3.addStatement( rk_xxx.getCols( 0,NX ) == rk_forward_sweep.getCols( run*rkOrder*NX+run1*NX,run*rkOrder*NX+(run1+1)*NX ) );
		// load backward trajectory
		loop3.addStatement( rk_xxx.getCols( NX,2*NX ) == rk_backward_sweep.getCols( (grid.getNumIntervals()-run)*rkOrder*NX-(run1+1)*NX,(grid.getNumIntervals()-run)*rkOrder*NX-run1*NX ) );
		loop3.addStatement( rk_xxx.getCols( 2*NX,rhsDim ) == rk_eta.getCols( 2*NX,rhsDim ) + Ah.getRow(run1)*rk_kkk.getCols( 0,NX*(NX+NU)+numX+NX*NU+numU ) );
		if( timeDependant ) loop3.addStatement( rk_xxx.getCol( inputDim ) == rk_ttt + ((double)cc(run1))/grid.getNumIntervals() );
		loop3.addFunctionCall( diffs_sweep3.getName(),rk_xxx,rk_kkk.getAddress(run1,0) );
	}
	loop3.addStatement( rk_eta.getCols( 2*NX,rhsDim ) += b4h^rk_kkk.getCols( 0,NX*(NX+NU)+numX+NX*NU+numU ) );
	loop3.addStatement( rk_ttt += DMatrix(1.0/grid.getNumIntervals()) );
    // end of integrator loop: THIRD SWEEP
	integrate.addStatement( loop3 );

	integrate.addStatement( error_code == 0 );

	LOG( LVL_DEBUG ) << "done" << endl;

	return SUCCESSFUL_RETURN;
}


returnValue ThreeSweepsERKExport::getDataDeclarations(	ExportStatementBlock& declarations,
														ExportStruct dataStruct
														) const
{
	AdjointERKExport::getDataDeclarations( declarations, dataStruct );

	declarations.addDeclaration( rk_backward_sweep,dataStruct );

    return SUCCESSFUL_RETURN;
}


returnValue ThreeSweepsERKExport::getCode(	ExportStatementBlock& code
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
				<< rk_forward_sweep.getFullName() << ", "
				<< rk_backward_sweep.getFullName()
				<< " )\n\n";
	}

	int sensGen;
	get( DYNAMIC_SENSITIVITY,sensGen );
	if( exportRhs ) {
		code.addFunction( rhs );
		code.addFunction( diffs_rhs );
		code.addFunction( diffs_sweep3 );
	}

	double h = (grid.getLastTime() - grid.getFirstTime())/grid.getNumIntervals();
	code.addComment(std::string("Fixed step size:") + toString(h));
	code.addFunction( integrate );

	return SUCCESSFUL_RETURN;
}


Expression ThreeSweepsERKExport::returnLowerTriangular( const Expression& expr ) {
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


Expression ThreeSweepsERKExport::symmetricDoubleProduct( const Expression& expr, const Expression& arg ) {

	// NOTE: the speedup of the three-sweeps-propagation approach is strongly dependent on the support for this specific operator which shows many symmetries
	uint dim = arg.getNumCols();
	uint dim2 = arg.getNumRows();

	IntermediateState inter_res = zeros<double>(dim2,dim);
	for( uint i = 0; i < dim; i++ ) {
		for( uint k1 = 0; k1 < dim2; k1++ ) {
			for( uint k2 = 0; k2 <= k1; k2++ ) {
				inter_res(k1,i) += expr(k1,k2)*arg(k2,i);
			}
			for( uint k2 = k1+1; k2 < dim2; k2++ ) {
				inter_res(k1,i) += expr(k2,k1)*arg(k2,i);
			}
		}
	}

	Expression new_expr;
	for( uint i = 0; i < dim; i++ ) {
		for( uint j = 0; j <= i; j++ ) {
			Expression new_tmp = 0;
			for( uint k1 = 0; k1 < dim2; k1++ ) {
				new_tmp = new_tmp+arg(k1,i)*inter_res(k1,j);
			}
			new_expr << new_tmp;
		}
	}
	return new_expr;
//	return returnLowerTriangular(arg.transpose()*expr*arg, dim);
}


ExportVariable ThreeSweepsERKExport::getAuxVariable() const
{
	ExportVariable max;
	max = rhs.getGlobalExportVariable();
	if( diffs_rhs.getGlobalExportVariable().getDim() > max.getDim() ) {
		max = diffs_rhs.getGlobalExportVariable();
	}
	if( diffs_sweep3.getGlobalExportVariable().getDim() > max.getDim() ) {
		max = diffs_sweep3.getGlobalExportVariable();
	}
	return max;
}

// PROTECTED:



CLOSE_NAMESPACE_ACADO

// end of file.
